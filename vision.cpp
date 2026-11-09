#include "vision.hpp"
#include "hazard_types.hpp"

#include <iostream>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

void vision_loop() try {
  // 1. Initialize RealSense pipeline and color alignment
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

  pipe.start(cfg);
  rs2::align align_to_color(RS2_STREAM_COLOR);

  std::cout << "[Vision] Streaming started\n";

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

    // --------------------------------------------------------------------
    // TODO (Vision Team):
    // Analyze depth_mat / color_mat and fill current_hazard {detected, x, y, z}
    // --------------------------------------------------------------------
    (void)color_mat;
    (void)depth_mat;
    HazardTarget current_hazard = {false, 0.0f, 0.0f, 0.0f};

    // Push latest coordinate safely to the shared state
    {
      std::lock_guard<std::mutex> lock(g_mtx);
      g_latest_hazard = current_hazard;
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
