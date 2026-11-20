#include "audio.hpp"
#include "hazard_types.hpp"

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// --- Beep parameters (step 1: fixed beep, independent of the vision data) ---
constexpr int kSampleRate = 48000;
constexpr float kBeepFrequencyHz = 880.0f;
constexpr float kBeepDurationS = 0.15f;
constexpr float kFadeDurationS = 0.005f; // Avoids audible clicks at start/end
constexpr float kBeepAmplitude = 0.8f;
constexpr auto kBeepPeriod = std::chrono::seconds(1);

// Mono 16-bit sine burst. Must stay MONO: OpenAL only spatializes (HRTF) mono
// buffers, stereo buffers are played as-is.
std::vector<int16_t> make_beep() {
  const int n = static_cast<int>(kSampleRate * kBeepDurationS);
  const int fade = static_cast<int>(kSampleRate * kFadeDurationS);
  std::vector<int16_t> samples(n);
  for (int i = 0; i < n; ++i) {
    float envelope = 1.0f;
    if (i < fade) {
      envelope = static_cast<float>(i) / fade;
    } else if (i >= n - fade) {
      envelope = static_cast<float>(n - 1 - i) / fade;
    }
    const float t = static_cast<float>(i) / kSampleRate;
    const float value = std::sin(2.0f * static_cast<float>(M_PI) *
                                 kBeepFrequencyHz * t);
    samples[i] = static_cast<int16_t>(value * envelope * kBeepAmplitude * 32767);
  }
  return samples;
}

void check_al(const char *what) {
  const ALenum err = alGetError();
  if (err != AL_NO_ERROR) {
    throw std::runtime_error(std::string(what) + " failed: " +
                             alGetString(err));
  }
}

// Owns the OpenAL device/context/buffer/source and releases them on scope exit.
// The output goes to the system default sink (PipeWire / PulseAudio), so the
// Bluetooth earphones only need to be the default output (see scripts/bt_audio.sh).
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

    // Listener at the origin, looking towards -Z (OpenAL default).
    // NOTE for later: HazardTarget.z is "forward positive", so a hazard must be
    // placed at (x, y, -z) in OpenAL coordinates.
    alListener3f(AL_POSITION, 0.0f, 0.0f, 0.0f);

    const std::vector<int16_t> beep = make_beep();
    alGenBuffers(1, &buffer_);
    alBufferData(buffer_, AL_FORMAT_MONO16, beep.data(),
                 static_cast<ALsizei>(beep.size() * sizeof(int16_t)),
                 kSampleRate);
    alGenSources(1, &source_);
    alSourcei(source_, AL_BUFFER, static_cast<ALint>(buffer_));
    alSourcei(source_, AL_SOURCE_RELATIVE, AL_TRUE);
    alSource3f(source_, AL_POSITION, 0.0f, 0.0f, -1.0f); // 1 m straight ahead
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

  void play_beep() {
    alSourcePlay(source_); // Restarts from the beginning if still playing
    check_al("alSourcePlay");
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
    std::cout << "[Audio] Output device: " << (name ? name : "unknown")
              << "\n";

    if (alcIsExtensionPresent(device_, "ALC_SOFT_HRTF")) {
      ALCint hrtf = ALC_FALSE;
      alcGetIntegerv(device_, ALC_HRTF_SOFT, 1, &hrtf);
      if (hrtf) {
        std::cout << "[Audio] HRTF enabled: "
                  << alcGetString(device_, ALC_HRTF_SPECIFIER_SOFT) << "\n";
      } else {
        std::cout << "[Audio] HRTF disabled\n";
      }
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
  std::cout << "[Audio] Worker started (1 beep / second)\n";

  // OpenAL keeps the output stream open (mixing silence between beeps), so the
  // Bluetooth earphones never go to sleep and cut the start of a beep.
  auto next_beep = std::chrono::steady_clock::now();
  while (g_is_running) {
    output.play_beep();
    next_beep += kBeepPeriod;

    // ----------------------------------------------------------------------
    // TODO (step 2): read g_hazards here and adapt the beep rate /
    // position (x, y, -z) to the detected hazard.
    // ----------------------------------------------------------------------
    std::unique_lock<std::mutex> lock(g_mtx);
    g_cv.wait_until(lock, next_beep, [] { return !g_is_running; });
  }

  std::cout << "[Audio] Stopped.\n";
} catch (const std::exception &e) {
  std::cerr << "[Audio] Error: " << e.what() << std::endl;
  g_is_running = false;
  g_cv.notify_all();
}
