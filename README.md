# RGBD_Hazard_Recognition
Hazard recognition repo from RGBD camera

## How to use
### Setup
#### Install librealsense

On Arch:
```bash
yay -S librealsense
```

On Fedora:
```bash
sudo dnf install librealsense librealsense-devel python3-librealsense
```

On Ubuntu:
```bash
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

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


#### Prepare
Create a build dir:
```bash
mkdir build
```

Specify the build directory:
```bash
cmake -B build
```

### Run
Build the project:
```bash
cmake --build build
```

Run the project:
```bash
./build/bin/head_soft
```
