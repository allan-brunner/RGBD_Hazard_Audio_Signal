#include "vision.hpp"
#include "hazard_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <vector>

namespace {

// --- Runtime Toggles ---
constexpr bool kDebugConsole = true;
constexpr bool kShowLiveWindow = true; // Set false on headless Arduino UNO Q
constexpr bool kSaveVideo = true;      // Set false to disable .avi recording

// --- Physical Size Thresholds for Street-Like Objects (meters) ---
constexpr float kMinObjWidthM = 0.25f; // Chairs, people, bins
constexpr float kMinObjHeightM = 0.35f;
constexpr float kMinPoleWidthM = 0.08f; // Lamp posts, sign poles
constexpr float kMinPoleHeightM = 0.60f;

// --- Depth-BFS & Tracking Parameters ---
constexpr int kScale = 4; // 640x480 -> 160x120 for fast BFS on Cortex-A53
constexpr uint16_t kMinDistMm = 300;
constexpr uint16_t kMaxDistMm = 2500;
constexpr int kDepthJumpMm = 80;
constexpr int kMinClusterPixels = 60;
constexpr double kMinRgbEdgeMean = 6.0;
constexpr float kFloorThresholdM = -0.6f;
constexpr float kMergeRadiusM = 0.4f;
constexpr int kMaxAgeFrames = 30;   // Keep ID alive briefly across occlusions
constexpr int kAudioHoldFrames = 5; // Drop from audio quickly once out of view

struct TrackedHazard {
  int id = 0;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float distance = 0.0f;
  int cx = 0;
  int cy = 0;
  int frames_since_seen = 0;
};

struct ClusterBox {
  cv::Rect rect;
  bool valid;
  float w_m;
  float h_m;
};

} // namespace

void vision_loop() try {
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

  rs2::pipeline_profile profile = pipe.start(cfg);
  rs2::align align_to_color(RS2_STREAM_COLOR);

  auto color_stream =
      profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  rs2_intrinsics intrinsics = color_stream.get_intrinsics();

  std::cout << "[Vision] Streaming started (Depth-BFS + RGB Validation)\n";

  cv::VideoWriter color_writer;
  cv::VideoWriter depth_writer;
  if (kSaveVideo) {
    const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    color_writer.open("color_debug.avi", fourcc, 30, cv::Size(640, 480));
    depth_writer.open("depth_debug.avi", fourcc, 30, cv::Size(640, 480));
  }

  int next_id = 1;
  std::vector<TrackedHazard> active_hazards;

  const int small_w = 640 / kScale;
  const int small_h = 480 / kScale;

  cv::Mat small_depth(small_h, small_w, CV_16UC1);
  cv::Mat small_color(small_h, small_w, CV_8UC3);
  cv::Mat small_gray, grad_x, grad_y, abs_x, abs_y, edge_mag;
  std::vector<uint8_t> visited(small_w * small_h, 0);

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

    const int width = color.get_width();
    const int height = color.get_height();
    cv::Mat color_mat(cv::Size(width, height), CV_8UC3,
                      const_cast<void *>(color.get_data()), cv::Mat::AUTO_STEP);
    cv::Mat depth_mat(cv::Size(width, height), CV_16UC1,
                      const_cast<void *>(depth.get_data()), cv::Mat::AUTO_STEP);

    // 1. Downsample depth (INTER_NEAREST preserves sharp edges) & RGB edge map
    cv::resize(depth_mat, small_depth, cv::Size(small_w, small_h), 0, 0,
               cv::INTER_NEAREST);
    cv::resize(color_mat, small_color, cv::Size(small_w, small_h), 0, 0,
               cv::INTER_LINEAR);
    cv::cvtColor(small_color, small_gray, cv::COLOR_BGR2GRAY);
    cv::Sobel(small_gray, grad_x, CV_16S, 1, 0, 3);
    cv::Sobel(small_gray, grad_y, CV_16S, 0, 1, 3);
    cv::convertScaleAbs(grad_x, abs_x);
    cv::convertScaleAbs(grad_y, abs_y);
    cv::addWeighted(abs_x, 0.5, abs_y, 0.5, 0, edge_mag);

    std::fill(visited.begin(), visited.end(), 0);
    std::vector<TrackedHazard> frame_detections;
    std::vector<ClusterBox> debug_boxes;
    std::queue<std::pair<int, int>> q;

    // 2. Depth-based BFS clustering + physical size & RGB validation
    for (int y = 0; y < small_h; ++y) {
      const uint16_t *d_row = small_depth.ptr<uint16_t>(y);
      for (int x = 0; x < small_w; ++x) {
        const int idx = y * small_w + x;
        const uint16_t d = d_row[x];

        if (visited[idx] || d < kMinDistMm || d > kMaxDistMm) {
          continue;
        }

        visited[idx] = 1;
        q.push({x, y});

        int count = 0;
        long long sum_x = 0, sum_y = 0, sum_d = 0;
        double sum_edge = 0.0;
        int min_x = x, max_x = x, min_y = y, max_y = y;

        while (!q.empty()) {
          auto [cx, cy] = q.front();
          q.pop();

          const uint16_t curr_d = small_depth.at<uint16_t>(cy, cx);
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
            const int nx = cx + dx[k];
            const int ny = cy + dy[k];
            if (nx < 0 || nx >= small_w || ny < 0 || ny >= small_h) {
              continue;
            }
            const int n_idx = ny * small_w + nx;
            if (visited[n_idx]) {
              continue;
            }
            const uint16_t nd = small_depth.at<uint16_t>(ny, nx);
            if (nd >= kMinDistMm && nd <= kMaxDistMm &&
                std::abs(static_cast<int>(nd) - static_cast<int>(curr_d)) <=
                    kDepthJumpMm) {
              visited[n_idx] = 1;
              q.push({nx, ny});
            }
          }
        }

        if (count < kMinClusterPixels) {
          continue;
        }

        const float dist_m = (static_cast<float>(sum_d) / count) / 1000.0f;
        const cv::Rect full_bbox(min_x * kScale, min_y * kScale,
                                 (max_x - min_x + 1) * kScale,
                                 (max_y - min_y + 1) * kScale);

        const float real_w = (full_bbox.width * dist_m) / intrinsics.fx;
        const float real_h = (full_bbox.height * dist_m) / intrinsics.fy;

        const bool is_general =
            (real_w >= kMinObjWidthM && real_h >= kMinObjHeightM);
        const bool is_pole =
            (real_w >= kMinPoleWidthM && real_h >= kMinPoleHeightM);
        const bool is_valid =
            (is_general || is_pole) && ((sum_edge / count) >= kMinRgbEdgeMean);

        if (kShowLiveWindow || kSaveVideo) {
          debug_boxes.push_back({full_bbox, is_valid, real_w, real_h});
        }

        if (!is_valid) {
          continue;
        }

        const int full_cx = static_cast<int>((sum_x / count) * kScale);
        const int full_cy = static_cast<int>((sum_y / count) * kScale);

        float pixel[2] = {static_cast<float>(full_cx),
                          static_cast<float>(full_cy)};
        float point3d[3];
        rs2_deproject_pixel_to_point(point3d, &intrinsics, pixel, dist_m);

        const float real_y = -point3d[1];
        if (real_y > kFloorThresholdM) {
          TrackedHazard det;
          det.x = point3d[0];
          det.y = real_y;
          det.z = point3d[2];
          det.distance =
              std::sqrt(det.x * det.x + det.y * det.y + det.z * det.z);
          det.cx = full_cx;
          det.cy = full_cy;
          frame_detections.push_back(det);
        }
      }
    }

    // 3. Track & Match Hazards in 3D
    for (auto &active : active_hazards) {
      active.frames_since_seen++;
    }

    for (auto &det : frame_detections) {
      bool matched = false;
      for (auto &active : active_hazards) {
        const float dx3 = det.x - active.x;
        const float dy3 = det.y - active.y;
        const float dz3 = det.z - active.z;
        if (std::sqrt(dx3 * dx3 + dy3 * dy3 + dz3 * dz3) < kMergeRadiusM) {
          active.x = det.x;
          active.y = det.y;
          active.z = det.z;
          active.distance = det.distance;
          active.cx = det.cx;
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

    active_hazards.erase(
        std::remove_if(active_hazards.begin(), active_hazards.end(),
                       [](const TrackedHazard &h) {
                         return h.frames_since_seen > kMaxAgeFrames;
                       }),
        active_hazards.end());

    // 4. Publish only recently seen hazards, sorted closest-first, to Audio
    std::vector<HazardTarget> audio_hazards;
    audio_hazards.reserve(active_hazards.size());
    for (const auto &h : active_hazards) {
      if (h.frames_since_seen <= kAudioHoldFrames) {
        audio_hazards.push_back({h.id, h.x, h.y, h.z, h.distance});
      }
    }
    std::sort(audio_hazards.begin(), audio_hazards.end(),
              [](const HazardTarget &a, const HazardTarget &b) {
                return a.distance < b.distance;
              });

    {
      std::lock_guard<std::mutex> lock(g_mtx);
      g_hazards = std::move(audio_hazards);
      g_has_new_frame = true;
    }
    g_cv.notify_one();

    // 5. Optional Console Output
    if (kDebugConsole) {
      if (active_hazards.empty()) {
        std::cout << "[Vision] Path clear...\n";
      } else {
        std::cout << "[Vision] Tracking " << active_hazards.size()
                  << " objects:\n";
        for (const auto &h : active_hazards) {
          std::cout << "  -> ID: " << h.id << " | X: " << h.x << "m, Y: " << h.y
                    << "m, Z: " << h.z
                    << "m | Hidden for: " << h.frames_since_seen << " frames\n";
        }
        std::cout << "---------------------------------\n";
      }
    }

    // 6. Optional Visualization & Video Buffer Save
    if (kShowLiveWindow || kSaveVideo) {
      cv::Mat depth_8u, depth_bgr;
      depth_mat.convertTo(depth_8u, CV_8UC1, 255.0 / kMaxDistMm);
      cv::cvtColor(depth_8u, depth_bgr, cv::COLOR_GRAY2BGR);

      for (const auto &box : debug_boxes) {
        const cv::Scalar color_box =
            box.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
        cv::rectangle(color_mat, box.rect, color_box, box.valid ? 2 : 1);
        cv::rectangle(depth_bgr, box.rect, color_box, box.valid ? 2 : 1);

        if (box.valid) {
          char dim_buf[32];
          std::snprintf(dim_buf, sizeof(dim_buf), "%.2fx%.2fm", box.w_m,
                        box.h_m);
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

          char lbl_buf[32];
          std::snprintf(lbl_buf, sizeof(lbl_buf), "ID:%d (%.2fm)", h.id, h.z);
          cv::putText(color_mat, lbl_buf, cv::Point(h.cx - 20, h.cy - 15),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0),
                      2);
          cv::putText(depth_bgr, lbl_buf, cv::Point(h.cx - 20, h.cy - 15),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0),
                      2);
        }
      }

      if (kSaveVideo) {
        color_writer.write(color_mat);
        depth_writer.write(depth_bgr);
      }

      if (kShowLiveWindow) {
        cv::Mat combined;
        cv::hconcat(color_mat, depth_bgr, combined);
        cv::imshow("Hazard Detector (RGB | Depth) - Press Q to quit", combined);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
          g_is_running = false;
        }
      }
    }
  }

  if (kShowLiveWindow) {
    cv::destroyAllWindows();
  }
  g_cv.notify_all();
  std::cout << "[Vision] Stopped.\n";
} catch (const rs2::error &e) {
  std::cerr << "[Vision] RealSense error calling " << e.get_failed_function()
            << "(" << e.get_failed_args() << "):\n    " << e.what()
            << std::endl;
  g_is_running = false;
  g_cv.notify_all();
}
