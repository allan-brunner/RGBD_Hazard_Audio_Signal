#include "vision.hpp"
#include "hazard_types.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <vector>

const bool DEBUG = true;
const bool SHOW_LIVE_WINDOW = true;
const bool SAVE_VIDEO = true;

// Physical size thresholds for street-like objects (in meters)
const float MIN_OBJ_WIDTH_M = 0.25f;
const float MIN_OBJ_HEIGHT_M = 0.35f;
const float MIN_POLE_WIDTH_M = 0.08f;
const float MIN_POLE_HEIGHT_M = 0.60f;

// --- Tuning Parameters ---
const int SCALE = 4; // 640x480 -> 160x120 for fast BFS on Cortex-A53
const uint16_t MIN_DIST_MM = 300;  // 0.3 m
const uint16_t MAX_DIST_MM = 2500; // 2.5 m (consistent with detection range)
const int DEPTH_JUMP_MM =
    80; // Max depth diff between adjacent pixels in same object
const int MIN_CLUSTER_PIXELS =
    60; // Minimum pixels in 160x120 grid (~960 px in 640x480)
const double MIN_RGB_EDGE_MEAN =
    6.0; // RGB validation: min average gradient to reject flat glare/noise
const float FLOOR_THRESHOLD_METERS = -0.6f; // Ignore points below this Y height
const float DISTANCE_THRESHOLD = 0.4f; // 40 cm 3D merge radius for tracking
const int MAX_AGE = 30;                // Delete if not seen for 30 frames

struct ClusterBox {
  cv::Rect rect;
  bool valid;
  float w_m;
  float h_m;
};

void vision_loop() try {
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

  rs2::pipeline_profile profile = pipe.start(cfg);
  rs2::align align_to_color(RS2_STREAM_COLOR);

  // Fetch color-aligned intrinsics once instead of every frame/object
  auto color_stream =
      profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  rs2_intrinsics intrinsics = color_stream.get_intrinsics();

  std::cout << "[Vision] Streaming started (Depth-BFS + RGB Validation)\n";

  cv::VideoWriter color_writer("color_debug.avi",
                               cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                               cv::Size(640, 480));
  cv::VideoWriter depth_writer("depth_debug.avi",
                               cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                               cv::Size(640, 480));

  int next_id = 1;
  std::vector<HazardTarget> active_hazards;

  const int small_w = 640 / SCALE;
  const int small_h = 480 / SCALE;

  // Reusable buffers to avoid per-frame heap allocations
  cv::Mat small_depth(small_h, small_w, CV_16UC1);
  cv::Mat small_color(small_h, small_w, CV_8UC3);
  cv::Mat small_gray, grad_x, grad_y, edge_mag;
  std::vector<uint8_t> visited(small_w * small_h, 0);

  // 4-connected neighbor offsets for BFS
  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};

  while (g_is_running) {
    rs2::frameset frames = pipe.wait_for_frames();
    frames = align_to_color.process(frames);

    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::video_frame color = frames.get_color_frame();
    if (!depth || !color) {
      continue;
    }

    int width = color.get_width();
    int height = color.get_height();
    cv::Mat color_mat(cv::Size(width, height), CV_8UC3,
                      const_cast<void *>(color.get_data()), cv::Mat::AUTO_STEP);
    cv::Mat depth_mat(cv::Size(width, height), CV_16UC1,
                      const_cast<void *>(depth.get_data()), cv::Mat::AUTO_STEP);

    // 1. Downsample depth (INTER_NEAREST preserves sharp object edges)
    cv::resize(depth_mat, small_depth, cv::Size(small_w, small_h), 0, 0,
               cv::INTER_NEAREST);

    // 2. Downsample RGB and compute fast edge magnitude for validation
    cv::resize(color_mat, small_color, cv::Size(small_w, small_h), 0, 0,
               cv::INTER_LINEAR);
    cv::cvtColor(small_color, small_gray, cv::COLOR_BGR2GRAY);
    cv::Sobel(small_gray, grad_x, CV_16S, 1, 0, 3);
    cv::Sobel(small_gray, grad_y, CV_16S, 0, 1, 3);
    cv::Mat abs_x, abs_y;
    cv::convertScaleAbs(grad_x, abs_x);
    cv::convertScaleAbs(grad_y, abs_y);
    cv::addWeighted(abs_x, 0.5, abs_y, 0.5, 0, edge_mag);

    // Reset visited array
    std::fill(visited.begin(), visited.end(), 0);

    std::vector<HazardTarget> frame_detections;
    std::vector<ClusterBox> debug_boxes;

    // 3. Depth-based BFS Region Growing
    std::queue<std::pair<int, int>> q;

    for (int y = 0; y < small_h; ++y) {
      const uint16_t *d_row = small_depth.ptr<uint16_t>(y);
      for (int x = 0; x < small_w; ++x) {
        int idx = y * small_w + x;
        uint16_t d = d_row[x];

        if (visited[idx] || d < MIN_DIST_MM || d > MAX_DIST_MM) {
          continue;
        }

        // Start a new cluster
        visited[idx] = 1;
        q.push({x, y});

        int count = 0;
        long long sum_x = 0, sum_y = 0, sum_d = 0;
        double sum_edge = 0.0;
        int min_x = x, max_x = x, min_y = y, max_y = y;

        while (!q.empty()) {
          auto [cx, cy] = q.front();
          q.pop();

          uint16_t curr_d = small_depth.at<uint16_t>(cy, cx);
          count++;
          sum_x += cx;
          sum_y += cy;
          sum_d += curr_d;
          sum_edge += edge_mag.at<uint8_t>(cy, cx);

          min_x = std::min(min_x, cx);
          max_x = std::max(max_x, cx);
          min_y = std::min(min_y, cy);
          max_y = std::max(max_y, cy);

          for (int k = 0; k < 4; ++k) {
            int nx = cx + dx[k];
            int ny = cy + dy[k];
            if (nx < 0 || nx >= small_w || ny < 0 || ny >= small_h) {
              continue;
            }
            int n_idx = ny * small_w + nx;
            if (visited[n_idx]) {
              continue;
            }
            uint16_t nd = small_depth.at<uint16_t>(ny, nx);
            if (nd >= MIN_DIST_MM && nd <= MAX_DIST_MM &&
                std::abs(static_cast<int>(nd) - static_cast<int>(curr_d)) <=
                    DEPTH_JUMP_MM) {
              visited[n_idx] = 1;
              q.push({nx, ny});
            }
          }
        }

        if (count < MIN_CLUSTER_PIXELS) {
          continue;
        }

        // 4. Compute physical size in meters using camera intrinsics
        float dist_meters = (static_cast<float>(sum_d) / count) / 1000.0f;

        cv::Rect full_bbox(min_x * SCALE, min_y * SCALE,
                           (max_x - min_x + 1) * SCALE,
                           (max_y - min_y + 1) * SCALE);

        float real_w = (full_bbox.width * dist_meters) / intrinsics.fx;
        float real_h = (full_bbox.height * dist_meters) / intrinsics.fy;

        bool is_general_obj =
            (real_w >= MIN_OBJ_WIDTH_M && real_h >= MIN_OBJ_HEIGHT_M);
        bool is_pole_obj =
            (real_w >= MIN_POLE_WIDTH_M && real_h >= MIN_POLE_HEIGHT_M);
        bool size_valid = (is_general_obj || is_pole_obj);

        // 5. RGB Validation
        double mean_edge = sum_edge / count;
        bool rgb_valid = (mean_edge >= MIN_RGB_EDGE_MEAN);

        bool is_valid = size_valid && rgb_valid;
        debug_boxes.push_back({full_bbox, is_valid, real_w, real_h});

        if (!is_valid) {
          continue;
        }

        // Deproject cluster centroid
        int full_cx = static_cast<int>((sum_x / count) * SCALE);
        int full_cy = static_cast<int>((sum_y / count) * SCALE);

        float pixel[2] = {static_cast<float>(full_cx),
                          static_cast<float>(full_cy)};
        float point3d[3];
        rs2_deproject_pixel_to_point(point3d, &intrinsics, pixel, dist_meters);

        float real_y = -point3d[1];
        if (real_y > FLOOR_THRESHOLD_METERS) {
          HazardTarget det;
          det.detected = true;
          det.x = point3d[0];
          det.y = real_y;
          det.z = point3d[2];
          det.cx = full_cx;
          det.cy = full_cy;
          frame_detections.push_back(det);
        }
      }
    }

    // 6. Track & Match Hazards in 3D
    for (auto &active : active_hazards) {
      active.frames_since_seen++;
    }

    for (auto &det : frame_detections) {
      bool matched = false;
      for (auto &active : active_hazards) {
        float dx3 = det.x - active.x;
        float dy3 = det.y - active.y;
        float dz3 = det.z - active.z;
        float distance = std::sqrt(dx3 * dx3 + dy3 * dy3 + dz3 * dz3);

        if (distance < DISTANCE_THRESHOLD) {
          active.x = det.x;
          active.y = det.y;
          active.z = det.z;
          active.cx = det.cx; // Keep 2D debug overlay synced
          active.cy = det.cy;
          active.frames_since_seen = 0;
          matched = true;
          break;
        }
      }
      if (!matched) {
        det.id = next_id++;
        det.frames_since_seen = 0;
        active_hazards.push_back(det);
      }
    }

    active_hazards.erase(std::remove_if(active_hazards.begin(),
                                        active_hazards.end(),
                                        [](const HazardTarget &h) {
                                          return h.frames_since_seen > MAX_AGE;
                                        }),
                         active_hazards.end());

    // 7. Live Console Output
    if (DEBUG) {
      if (active_hazards.empty()) {
        std::cout << "[Live Test] Path clear...\n";
      } else {
        std::cout << "[Live Test] Tracking " << active_hazards.size()
                  << " objects:\n";
        for (const auto &h : active_hazards) {
          std::cout << "  -> ID: " << h.id << " | X: " << h.x << "m, Y: " << h.y
                    << "m, Z: " << h.z
                    << "m | Hidden for: " << h.frames_since_seen << " frames\n";
        }
        std::cout << "---------------------------------\n";
      }
    }

    // 8. Debug Visualization, Live Window & Video Buffer Save
    cv::Mat depth_8u, depth_bgr;
    depth_mat.convertTo(depth_8u, CV_8UC1, 255.0 / MAX_DIST_MM);
    cv::cvtColor(depth_8u, depth_bgr, cv::COLOR_GRAY2BGR);

    for (const auto &box : debug_boxes) {
      // Green = valid street obstacle, Gray = too small or failed RGB check
      cv::Scalar color_box =
          box.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
      cv::rectangle(color_mat, box.rect, color_box, box.valid ? 2 : 1);
      cv::rectangle(depth_bgr, box.rect, color_box, box.valid ? 2 : 1);

      // Show physical dimensions (WxH in meters) on valid boxes for easy tuning
      if (box.valid) {
        char dim_buf[32];
        std::snprintf(dim_buf, sizeof(dim_buf), "%.2fx%.2fm", box.w_m, box.h_m);
        cv::putText(color_mat, dim_buf,
                    cv::Point(box.rect.x, std::max(15, box.rect.y - 5)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1);
      }
    }

    for (const auto &h : active_hazards) {
      if (h.frames_since_seen == 0) {
        cv::circle(color_mat, cv::Point(h.cx, h.cy), 5, cv::Scalar(0, 0, 255),
                   -1);
        cv::circle(depth_bgr, cv::Point(h.cx, h.cy), 5, cv::Scalar(0, 0, 255),
                   -1);

        std::string label = "ID:" + std::to_string(h.id) + " (" +
                            std::to_string(h.z).substr(0, 4) + "m)";
        cv::putText(color_mat, label, cv::Point(h.cx - 20, h.cy - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
        cv::putText(depth_bgr, label, cv::Point(h.cx - 20, h.cy - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
      }
    }

    if (SAVE_VIDEO) {
      color_writer.write(color_mat);
      depth_writer.write(depth_bgr);
    }

    if (SHOW_LIVE_WINDOW) {
      cv::Mat combined;
      cv::hconcat(color_mat, depth_bgr, combined);
      cv::imshow("Hazard Detector (RGB | Depth) - Press Q to quit", combined);
      int key = cv::waitKey(1);
      if (key == 'q' || key == 27) { // 'q' or ESC cleanly stops both threads
        g_is_running = false;
      }
    }

    // 9. Notify Audio Thread
    {
      std::lock_guard<std::mutex> lock(g_mtx);
      g_hazards = active_hazards;
      g_has_new_frame = true;
    }
    g_cv.notify_one();
  }

  std::cout << "[Vision] Stopped.\n";
} catch (const rs2::error &e) {
  std::cerr << "[Vision] RealSense error calling " << e.get_failed_function()
            << "(" << e.get_failed_args() << "):\n    " << e.what()
            << std::endl;
  g_is_running = false;
  g_cv.notify_all();
}
