#include "audio.hpp"
#include "hazard_types.hpp"
#include "vision.hpp"

#include <csignal>
#include <iostream>
#include <thread>

std::mutex g_mtx;
std::condition_variable g_cv;
std::vector<HazardTarget> g_hazards;
bool g_has_new_frame = false;
std::atomic<bool> g_is_running{true};

// Async-signal-safe: worker threads poll g_is_running.
void signal_handler(int) { g_is_running = false; }

int main() try {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler); // Sent by `docker stop`

  std::thread audio_thread(audio_loop);
  std::thread vision_thread(vision_loop);

  vision_thread.join();
  audio_thread.join();

  return 0;
} catch (const std::exception &e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
