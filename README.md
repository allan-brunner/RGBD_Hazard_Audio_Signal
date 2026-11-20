# RGBD_Hazard_Recognition
Hazard recognition repo from RGBD camera

## How to use
### Setup

#### Clone
Clone the repo:

Using HTTPS:
```bash
git clone https://github.com/allan-brunner/RGBD_Hazard_Audio_Signal.git
```

Using SSH:
```bash
git clone git@github.com:allan-brunner/RGBD_Hazard_Audio_Signal.git
```

Go to clone directory:
```bash
cd RGBD_Hazard_Audio_Signal
```

#### Install CMake
On Arch:
```bash
sudo pacman -S cmake base-devel
```

On Fedora:
```bash
sudo dnf install cmake gcc-c++ make
```

On Ubuntu:
```bash
sudo apt install cmake
```

### Execute the script trought the container

> If not on ARM64 setup, install QEMU Binfmt emulation:
>
> `docker run --privileged --rm tonistiigi/binfmt --install arm64`

Run:
```bash
docker compose up -d
```

`--mock` (used by the container) disables the camera: only the audio runs.

## Audio over Bluetooth earphones

The program plays its sounds with OpenAL on the **default audio output** of the
system (PipeWire / PulseAudio). Bluetooth is handled by the OS (BlueZ): the
earphones only have to be connected and set as default output.
Current behaviour (step 1): one beep per second, straight ahead.

### 1. Connect the earphones (on the host / the UNO Q, not in Docker)

Required packages:
- Fedora: `bluez pipewire-pulseaudio` (installed by default)
- Debian / UNO Q: `sudo apt install bluez pulseaudio-utils pipewire-pulse wireplumber libspa-0.2-bluetooth`

Check that an audio server is running: `pactl info`.

```bash
# AirPods: in the case, lid open, hold the back button until the LED flashes white
scripts/bt_audio.sh scan                       # find the MAC address
scripts/bt_audio.sh connect XX:XX:XX:XX:XX:XX  # pair + connect + default output
scripts/bt_audio.sh status
```

### 2b. Test in the UNO Q simulation container

```bash
docker compose up --build   # audio goes through the host PipeWire/PulseAudio socket
```

### Troubleshooting

- **AirPods fail to pair / connect** on Linux: in `/etc/bluetooth/main.conf`
  set `ControllerMode = bredr`, then `sudo systemctl restart bluetooth`.
- **No sound but earphones connected**: `pactl get-default-sink` must show a
  `bluez_output...` (PipeWire) or `bluez_sink...` (PulseAudio) sink.
- Bluetooth A2DP adds ~150-300 ms of latency (≈ 30-40 cm at walking speed).
