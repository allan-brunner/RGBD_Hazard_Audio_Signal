#include <iostream>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs.hpp>

int main() try {
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
  }

  std::cout << "\nDone.\n";
  return EXIT_SUCCESS;
} catch (const rs2::error &e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "("
            << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception &e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
