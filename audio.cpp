#include "audio.hpp"
#include "hazard_types.hpp"

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using Ms = std::chrono::milliseconds;

// --- Debug Toggle ---
constexpr bool kDebugAudio = true;

// --- Sound Synthesis Parameters ---
constexpr int kSampleRate = 48000;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kBaseFreqHz = 660.0f;
constexpr float kBeepDurationS =
    0.10f; // Short crisp ping for clean 3D separation
constexpr float kFadeDurationS = 0.005f;
constexpr float kBeepAmplitude = 0.75f;

// --- Fairness & Anti-Spam Scheduler Parameters ---
constexpr std::size_t kMaxAudioHazards =
    3;                           // Notify at most the 3 closest objects
constexpr Ms kMinGlobalGap{220}; // Min silence+beep spacing between any 2 pings
constexpr float kNearDistM = 0.4f;
constexpr float kFarDistM = 2.5f;
constexpr int kMinObjCooldownMs =
    550; // Urgent repeat rate for one very close object
constexpr int kMaxObjCooldownMs = 2200; // Calm repeat rate for one far object
constexpr float kApproachTriggerM =
    0.40f;                       // Early alert if object moves >= 40 cm closer
constexpr Ms kStateExpiry{3000}; // Remember IDs briefly across depth flicker
constexpr Ms kPollSlice{100};    // Max sleep so SIGINT/SIGTERM stays responsive

struct ObjectAudioState {
  bool has_beeped = false;
  Clock::time_point last_beep{};
  Clock::time_point last_seen{};
  float last_beep_dist = 0.0f;
};

// Harmonic-rich MONO 16-bit ping (OpenAL HRTF requires mono, and harmonics
// dramatically improve 3D left/right/front/back localization over a pure sine).
std::vector<int16_t> make_beep() {
  const int n = static_cast<int>(kSampleRate * kBeepDurationS);
  const int fade = static_cast<int>(kSampleRate * kFadeDurationS);
  std::vector<int16_t> samples(n);

  for (int i = 0; i < n; ++i) {
    float env = 1.0f;
    if (i < fade) {
      env = static_cast<float>(i) / fade;
    } else if (i >= n - fade) {
      env = static_cast<float>(n - 1 - i) / fade;
    }

    const float t = static_cast<float>(i) / kSampleRate;
    const float wave = 0.55f * std::sin(2.0f * kPi * kBaseFreqHz * t) +
                       0.30f * std::sin(2.0f * kPi * (kBaseFreqHz * 2.0f) * t) +
                       0.15f * std::sin(2.0f * kPi * (kBaseFreqHz * 3.0f) * t);

    samples[i] = static_cast<int16_t>(wave * env * kBeepAmplitude * 32767.0f);
  }
  return samples;
}

float normalized_distance(float dist_m) {
  return std::clamp((dist_m - kNearDistM) / (kFarDistM - kNearDistM), 0.0f,
                    1.0f);
}

Ms cooldown_for_distance(float dist_m) {
  const float t = normalized_distance(dist_m);
  const int ms = kMinObjCooldownMs +
                 static_cast<int>(t * (kMaxObjCooldownMs - kMinObjCooldownMs));
  return Ms(ms);
}

// Slightly higher pitch for closer hazards (0.85 far -> 1.25 close)
float pitch_for_distance(float dist_m) {
  const float t = normalized_distance(dist_m);
  return 1.25f - 0.40f * t;
}

void check_al(const char *what) {
  const ALenum err = alGetError();
  if (err != AL_NO_ERROR) {
    throw std::runtime_error(std::string(what) +
                             " failed: " + alGetString(err));
  }
}

class AudioOutput {
public:
  AudioOutput() {
    device_ = alcOpenDevice(nullptr);
    if (!device_) {
      throw std::runtime_error("No audio output device found");
    }
    context_ = alcCreateContext(device_, nullptr);
    if (!context_ || !alcMakeContextCurrent(context_)) {
      release();
      throw std::runtime_error("Could not create OpenAL context");
    }
    print_device_info();

    alDistanceModel(AL_INVERSE_DISTANCE_CLAMPED);
    alListener3f(AL_POSITION, 0.0f, 0.0f, 0.0f);

    const std::vector<int16_t> beep = make_beep();
    alGenBuffers(1, &buffer_);
    alBufferData(buffer_, AL_FORMAT_MONO16, beep.data(),
                 static_cast<ALsizei>(beep.size() * sizeof(int16_t)),
                 kSampleRate);

    alGenSources(1, &source_);
    alSourcei(source_, AL_BUFFER, static_cast<ALint>(buffer_));
    alSourcei(source_, AL_SOURCE_RELATIVE, AL_TRUE);
    alSourcef(source_, AL_REFERENCE_DISTANCE, 0.6f);
    alSourcef(source_, AL_ROLLOFF_FACTOR, 0.8f);
    alSourcef(source_, AL_MAX_DISTANCE, 3.0f);

    try {
      check_al("Source/buffer setup");
    } catch (...) {
      release();
      throw;
    }
  }

  ~AudioOutput() { release(); }

  AudioOutput(const AudioOutput &) = delete;
  AudioOutput &operator=(const AudioOutput &) = delete;

  // RealSense (+X right, +Y up, +Z forward) -> OpenAL (+X right, +Y up, -Z
  // forward)
  void play_hazard_ping(const HazardTarget &h) {
    alSource3f(source_, AL_POSITION, h.x, h.y, -h.z);
    alSourcef(source_, AL_PITCH, pitch_for_distance(h.distance));
    alSourcePlay(source_);
    check_al("play_hazard_ping");
  }

private:
  void print_device_info() const {
    const ALCchar *name = nullptr;
    if (alcIsExtensionPresent(device_, "ALC_ENUMERATE_ALL_EXT")) {
      name = alcGetString(device_, ALC_ALL_DEVICES_SPECIFIER);
    }
    if (!name) {
      name = alcGetString(device_, ALC_DEVICE_SPECIFIER);
    }
    std::cout << "[Audio] Output device: " << (name ? name : "unknown") << "\n";

    if (alcIsExtensionPresent(device_, "ALC_SOFT_HRTF")) {
      ALCint hrtf = ALC_FALSE;
      alcGetIntegerv(device_, ALC_HRTF_SOFT, 1, &hrtf);
      std::cout << "[Audio] HRTF "
                << (hrtf ? std::string("enabled: ") +
                               alcGetString(device_, ALC_HRTF_SPECIFIER_SOFT)
                         : "disabled")
                << "\n";
    }
  }

  void release() {
    if (source_) {
      alSourceStop(source_);
      alDeleteSources(1, &source_);
      source_ = 0;
    }
    if (buffer_) {
      alDeleteBuffers(1, &buffer_);
      buffer_ = 0;
    }
    alcMakeContextCurrent(nullptr);
    if (context_) {
      alcDestroyContext(context_);
      context_ = nullptr;
    }
    if (device_) {
      alcCloseDevice(device_);
      device_ = nullptr;
    }
  }

  ALCdevice *device_ = nullptr;
  ALCcontext *context_ = nullptr;
  ALuint buffer_ = 0;
  ALuint source_ = 0;
};

} // namespace

void audio_loop() try {
  AudioOutput output;
  std::cout << "[Audio] Worker started (fair 3D hazard scheduler)\n";

  std::vector<HazardTarget> hazards;
  std::unordered_map<int, ObjectAudioState> states;
  auto last_any_beep = Clock::now() - kMinGlobalGap;

  while (g_is_running) {
    // 1. Grab latest hazard snapshot from Vision
    {
      std::lock_guard<std::mutex> lock(g_mtx);
      if (g_has_new_frame) {
        hazards = g_hazards;
        g_has_new_frame = false;
      }
    }

    const auto now = Clock::now();
    const std::size_t active_count = std::min(hazards.size(), kMaxAudioHazards);

    // 2. Refresh last_seen timestamps and prune expired object history
    for (std::size_t i = 0; i < active_count; ++i) {
      states[hazards[i].id].last_seen = now;
    }
    for (auto it = states.begin(); it != states.end();) {
      if (now - it->second.last_seen > kStateExpiry) {
        it = states.erase(it);
      } else {
        ++it;
      }
    }

    // Stay silent when the path is clear
    if (active_count == 0) {
      std::unique_lock<std::mutex> lock(g_mtx);
      g_cv.wait_for(lock, kPollSlice,
                    [] { return g_has_new_frame || !g_is_running; });
      continue;
    }

    // 3. Pick the most deserving hazard if the global audio slot is open
    if (now >= last_any_beep + kMinGlobalGap) {
      int best_idx = -1;
      float best_score = -1.0f;

      for (std::size_t i = 0; i < active_count; ++i) {
        const HazardTarget &h = hazards[i];
        const ObjectAudioState &st = states[h.id];

        float score = -1.0f;
        if (!st.has_beeped) {
          // Brand-new object: announce immediately (closest first)
          score = 100.0f - h.distance;
        } else if (st.last_beep_dist - h.distance >= kApproachTriggerM) {
          // Significant sudden approach: bypass cooldown
          score = 50.0f - h.distance;
        } else {
          const auto elapsed_ms =
              std::chrono::duration_cast<Ms>(now - st.last_beep).count();
          const auto period_ms = cooldown_for_distance(h.distance).count();
          if (elapsed_ms >= period_ms) {
            // Overdue ratio: fair across objects while favoring closer ones
            score =
                static_cast<float>(elapsed_ms) / static_cast<float>(period_ms);
          }
        }

        if (score > best_score) {
          best_score = score;
          best_idx = static_cast<int>(i);
        }
      }

      if (best_idx >= 0) {
        const HazardTarget &chosen = hazards[best_idx];
        output.play_hazard_ping(chosen);

        ObjectAudioState &st = states[chosen.id];
        st.has_beeped = true;
        st.last_beep = now;
        st.last_beep_dist = chosen.distance;
        last_any_beep = now;

        if (kDebugAudio) {
          std::cout << "[Audio] Ping ID:" << chosen.id
                    << " | dist=" << chosen.distance << "m"
                    << " (X:" << chosen.x << ", Y:" << chosen.y
                    << ", Z:" << chosen.z << ")\n";
        }
      }
    }

    // 4. Sleep precisely until the next object is due (or a new frame arrives)
    auto earliest_due = now + kPollSlice;
    for (std::size_t i = 0; i < active_count; ++i) {
      const HazardTarget &h = hazards[i];
      const ObjectAudioState &st = states[h.id];

      Clock::time_point due = now;
      if (st.has_beeped &&
          (st.last_beep_dist - h.distance < kApproachTriggerM)) {
        due = st.last_beep + cooldown_for_distance(h.distance);
      }
      earliest_due = std::min(earliest_due, due);
    }

    auto wake_time = std::max(earliest_due, last_any_beep + kMinGlobalGap);
    wake_time = std::min(wake_time, now + kPollSlice);

    std::unique_lock<std::mutex> lock(g_mtx);
    g_cv.wait_until(lock, wake_time,
                    [] { return g_has_new_frame || !g_is_running; });
  }

  std::cout << "[Audio] Stopped.\n";
} catch (const std::exception &e) {
  std::cerr << "[Audio] Error: " << e.what() << std::endl;
  g_is_running = false;
  g_cv.notify_all();
}
