#include "vision.hpp"
#include "hazard_types.hpp"

#include <iostream>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

const bool DEBUG = true;

void vision_loop() try {
  // Initialize RealSense pipeline and color alignment
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

  pipe.start(cfg);
  rs2::align align_to_color(RS2_STREAM_COLOR);

  std::cout << "[Vision] Streaming started\n";

  int next_id = 1;
  std::vector<HazardTarget> active_hazards;
  const float DISTANCE_THRESHOLD = 0.4f; // 40 cm merge radius
  const int MAX_AGE = 30;                // Delete if not seen for 30 frames

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

    // Define hazard thresholds in millimeters
    const uint16_t MIN_DIST_MM = 300;
    const uint16_t MAX_DIST_MM = 3500;

    // Create a binary mask for pixels within the hazard range
    cv::Mat hazard_mask;
    cv::inRange(depth_mat, MIN_DIST_MM, MAX_DIST_MM, hazard_mask);

    cv::erode(hazard_mask, hazard_mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(hazard_mask, hazard_mask, cv::Mat(), cv::Point(-1, -1), 2);

    // Find contours in the mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(hazard_mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    std::vector<HazardTarget> frame_detections;

    // Find ALL valid objects in the current frame
    for (const auto &contour : contours) {
      if (cv::contourArea(contour) > 1500.0) {
        cv::Moments m = cv::moments(contour);
        int cx = static_cast<int>(m.m10 / (m.m00 + 1e-5));
        int cy = static_cast<int>(m.m01 / (m.m00 + 1e-5));
        float dist_meters = depth.get_distance(cx, cy);

        if (dist_meters > 0.0f && dist_meters < 2.0f) {
          auto profile = depth.get_profile().as<rs2::video_stream_profile>();
          rs2_intrinsics intrinsics = profile.get_intrinsics();
          float pixel[2] = {static_cast<float>(cx), static_cast<float>(cy)};
          float point3d[3];
          rs2_deproject_pixel_to_point(point3d, &intrinsics, pixel,
                                       dist_meters);

          // Calculate the real Y relative to the camera (+Up, -Down)
          float real_y = -point3d[1];

          const float FLOOR_THRESHOLD_METERS = -0.6f;

          if (real_y > FLOOR_THRESHOLD_METERS) {
            HazardTarget det;
            det.detected = true;
            det.x = point3d[0];
            det.y = real_y;
            det.z = point3d[2];
            frame_detections.push_back(det);
          }
        }
      }
    }

    // Age existing hazards
    for (auto &active : active_hazards) {
      active.frames_since_seen++;
    }

    // Match new detections to existing objects using 3D distance
    for (auto &det : frame_detections) {
      bool matched = false;
      for (auto &active : active_hazards) {
        float dx = det.x - active.x;
        float dy = det.y - active.y;
        float dz = det.z - active.z;
        float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance < DISTANCE_THRESHOLD) {
          // Same object! Update coordinates and reset age
          active.x = det.x;
          active.y = det.y;
          active.z = det.z;
          active.frames_since_seen = 0;
          matched = true;
          break;
        }
      }
      if (!matched) {
        // Brand new object
        det.id = next_id++;
        det.frames_since_seen = 0;
        active_hazards.push_back(det);
      }
    }

    // Delete objects that left the frame or timed out
    active_hazards.erase(std::remove_if(active_hazards.begin(),
                                        active_hazards.end(),
                                        [](const HazardTarget &h) {
                                          return h.frames_since_seen > MAX_AGE;
                                        }),
                         active_hazards.end());

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
