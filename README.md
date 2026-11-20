# RGBD_Hazard_Recognition
Hazard recognition repo from RGBD camera

## How to use
### Setup

#### Clone
Using HTTPS:
```bash
git clone [https://github.com/allan-brunner/RGBD_Hazard_Audio_Signal.git](https://github.com/allan-brunner/RGBD_Hazard_Audio_Signal.git)
```

Using SSH:

```bash
git clone git@github.com:allan-brunner/RGBD_Hazard_Audio_Signal.git
```

Go to the clone directory:

```bash
cd RGBD_Hazard_Audio_Signal
```

### Run in the UNO Q simulation container

> If not on an ARM64 host, install QEMU binfmt emulation first:
> `docker run --privileged --rm tonistiigi/binfmt --install arm64`

If `kShowLiveWindow` is enabled in `vision.cpp`, allow Docker to open an X11 window on your host:

```bash
xhost +local:docker
```

Build and run:

```bash
docker compose up --build
```

## Audio over Bluetooth earphones

The program plays its sounds with OpenAL on the **default audio output** of the
system (PipeWire / PulseAudio). Bluetooth is handled by the OS (BlueZ): the
earphones only have to be connected and set as default output.

### 1. Connect the earphones (on the host / the UNO Q, not in Docker)

Required packages:

* Fedora: `bluez pipewire-pulseaudio` (installed by default)
* Debian / UNO Q: `sudo apt install bluez pulseaudio-utils pipewire-pulse wireplumber libspa-0.2-bluetooth`

Check that an audio server is running: `pactl info`.

```bash
# Example (AirPods): in the case, lid open, hold the back button until LED flashes white
scripts/bt_audio.sh scan                       # find the MAC address
scripts/bt_audio.sh connect XX:XX:XX:XX:XX:XX  # pair + connect + default output
scripts/bt_audio.sh status
```

### 2. Troubleshooting

* **AirPods fail to pair / connect** on Linux: in `/etc/bluetooth/main.conf`
set `ControllerMode = bredr`, then `sudo systemctl restart bluetooth`.
* **No sound but earphones connected**: `pactl get-default-sink` must show a
`bluez_output...` (PipeWire) or `bluez_sink...` (PulseAudio) sink, using the
A2DP (stereo) profile so OpenAL HRTF 3D audio works.
* Bluetooth A2DP adds ~150-300 ms of latency (≈ 30-40 cm at walking speed).

