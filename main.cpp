#include "audio.hpp"
#include "hazard_types.hpp"
#include "vision.hpp"

#include <csignal>
#include <iostream>
#include <thread>

// Define global shared variables
std::mutex g_mtx;
std::condition_variable g_cv;
HazardTarget g_latest_hazard;
bool g_has_new_frame = false;
std::atomic<bool> g_is_running{true};

void signal_handler(int) {
  g_is_running = false;
  g_cv.notify_all();
}

int main() try {
  std::signal(SIGINT, signal_handler);

  std::thread vision_thread(vision_loop);
  std::thread audio_thread(audio_loop);

  vision_thread.join();
  audio_thread.join();

  return 0;
} catch (const std::exception &e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
