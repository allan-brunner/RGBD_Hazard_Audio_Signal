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
