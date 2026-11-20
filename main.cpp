#include "audio.hpp"
#include "hazard_types.hpp"
#include "vision.hpp"

#include <csignal>
#include <cstring>
#include <iostream>
#include <thread>

// Define global shared variables
std::mutex g_mtx;
std::condition_variable g_cv;
std::vector<HazardTarget> g_hazards;
bool g_has_new_frame = false;
std::atomic<bool> g_is_running{true};

// Only async-signal-safe work here: the worker threads poll g_is_running
// (the audio thread wakes up at least once per beep period).
void signal_handler(int) { g_is_running = false; }

int main(int argc, char **argv) try {
  bool mock = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--mock") == 0) {
      mock = true;
    }
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler); // Sent by `docker stop`

  std::thread audio_thread(audio_loop);
  std::thread vision_thread;

  if (!mock) {
    vision_thread = std::thread(vision_loop);
  } else {
    std::cout << "[Main] --mock: camera disabled, audio only\n";
  }

  if (vision_thread.joinable()) {
    vision_thread.join();
  }
  audio_thread.join();

  return 0;
} catch (const std::exception &e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
