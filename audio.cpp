#include "audio.hpp"
#include "hazard_types.hpp"

#include <AL/al.h>
#include <AL/alc.h>
#include <iostream>

void audio_loop() {
  // ------------------------------------------------------------------------
  // TODO (Audio Team):
  // Initialize OpenAL device, context, sound buffers, and 3D source here
  // ------------------------------------------------------------------------
  std::cout << "[Audio] Worker started\n";

  while (g_is_running) {
    HazardTarget hazard;

    // Safely wait for a new coordinate from the Vision thread
    {
      std::unique_lock<std::mutex> lock(g_mtx);
      g_cv.wait(lock, [] { return g_has_new_frame || !g_is_running; });
      if (!g_is_running) {
        break;
      }
      hazard = g_latest_hazard;
      g_has_new_frame = false;
    }

    // --------------------------------------------------------------------
    // TODO (Audio Team):
    // If hazard.detected is true, update 3D source position (hazard.x,
    // hazard.y, hazard.z) and trigger the audio alert
    // --------------------------------------------------------------------
    (void)hazard;
  }

  // ------------------------------------------------------------------------
  // TODO (Audio Team):
  // Clean up OpenAL sources, buffers, context, and device here
  // ------------------------------------------------------------------------
  std::cout << "[Audio] Stopped.\n";
}
