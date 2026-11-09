# Match the Arduino UNO Q 64-bit ARM Debian environment
FROM --platform=linux/arm64 debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive

# Install build toolchain, RealSense USB deps, OpenCV, OpenAL (3D Audio), and Bluetooth/Audio routing
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    ca-certificates \
    udev \
    libusb-1.0-0-dev \
    libssl-dev \
    libopencv-dev \
    libopenal-dev \
    libalut-dev \
    pulseaudio-utils \
    libpulse0 \
    bluez \
    libbluetooth-dev \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Build librealsense2 from source with RSUSB backend (required for D456 in ARM64 containers)
ARG LIBREALSENSE_VERSION=v2.56.3
RUN git clone --depth 1 --branch ${LIBREALSENSE_VERSION} https://github.com/IntelRealSense/librealsense.git /tmp/librealsense \
    && mkdir -p /tmp/librealsense/build && cd /tmp/librealsense/build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_PYTHON_BINDINGS=OFF \
        -DCMAKE_CXX_FLAGS="-mcpu=cortex-a53" \
    && make -j4 \
    && make install \
    && ldconfig \
    && rm -rf /tmp/librealsense

# Enable OpenAL Head-Related Transfer Function (HRTF) for 3D binaural audio over earphones
RUN mkdir -p /etc/openal && \
    echo "[general]\nhrtf = true\n" > /etc/openal/alsoft.conf

# Replicate the default 'arduino' user and '/home/arduino' workspace of the UNO Q
RUN useradd -m -s /bin/bash -G plugdev,audio,video,sudo arduino && \
    echo "arduino ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER arduino
WORKDIR /home/arduino/RGBD_Hazard_Audio_Signal

# Copy project files
COPY --chown=arduino:arduino CMakeLists.txt main.cpp ./

# Build the C++ project targeting Cortex-A53
RUN mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j4

CMD ["./build/hazard_detector", "--mock"]
