#include <condition_variable>
#include <iostream>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <queue>
#include <thread>

struct HazardTarget {
  float x, y, z;
};

std::queue<HazardTarget> hazard_queue;
std::mutex mtx;
std::condition_variable cv;

void vision_loop() try {
  // Create a Pipeline - top-level API for streaming and processing frames
  rs2::pipeline pipe;

  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

  // Start streaming with the configuration
  pipe.start(cfg);

  rs2::align align_to_color(RS2_STREAM_COLOR);

  std::cout << "Streaming started\n";

  while (true) {
    // Block program until a new coherent set of frames arrives
    rs2::frameset frames = pipe.wait_for_frames();

    frames = align_to_color.process(frames);

    // Extract the depth and color frames
    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::video_frame color = frames.get_color_frame();

    if (!depth || !color)
      continue;

    // Query frame dimensions
    int width = depth.get_width();
    int height = depth.get_height();

    // Query the distance from the camera to the object in the center of the
    // image
    float dist_to_center = depth.get_distance(width / 2, height / 2);

    std::cout << "Frame #" << depth.get_frame_number()
              << " - Center distance: " << dist_to_center << " meters\r"
              << std::flush;

    HazardTarget current_hazard = {0, 1, 2};

    // Safe push to audio thread
    {
      std::lock_guard<std::mutex> lock(mtx);
      hazard_queue.push(current_hazard);
    }
    cv.notify_one();
  }

  std::cout << "\nDone.\n";
} catch (const rs2::error &e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "("
            << e.get_failed_args() << "):\n    " << e.what() << std::endl;
}

void audio_loop() {
  // Init things

  // Loop
  while (true) {
    HazardTarget hazard;

    // Wait safely fo ra coordinate
    {
      std::unique_lock<std::mutex> lock(mtx);
      cv.wait(lock, [] { return !hazard_queue.empty(); });
      hazard = hazard_queue.front();
      hazard_queue.pop();
    }
  }

  // Update the position

  // Play the sound
}

int main() try {
  std::thread vision_thread(vision_loop);
  std::thread audio_thread(audio_loop);

  vision_thread.join();
  audio_thread.join();

  return 0;
} catch (const std::exception &e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
