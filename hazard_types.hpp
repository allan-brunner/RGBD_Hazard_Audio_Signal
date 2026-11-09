#ifndef HAZARD_TYPES_HPP
#define HAZARD_TYPES_HPP

#include <atomic>
#include <condition_variable>
#include <mutex>

struct HazardTarget {
  bool detected = false;
  float x = 0.0f; // Horizontal offset in meters (-left, +right)
  float y = 0.0f; // Vertical offset in meters (-down, +up)
  float z = 0.0f; // Forward distance in meters
};

// Shared state extern declarations
extern std::mutex g_mtx;
extern std::condition_variable g_cv;
extern HazardTarget g_latest_hazard;
extern bool g_has_new_frame;
extern std::atomic<bool> g_is_running;

#endif // HAZARD_TYPES_HPP
