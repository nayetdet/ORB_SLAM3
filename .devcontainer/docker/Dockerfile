FROM mcr.microsoft.com/devcontainers/base:trixie

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ORB_SLAM3_ROOT=/workspaces/ORB_SLAM3 \
    CMAKE_POLICY_VERSION_MINIMUM=3.5

USER root

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        git \
        ca-certificates \
        curl \
        wget \
        unzip \
        zsh \
        python3 \
        python3-numpy \
        python3-matplotlib \
        python3-yaml \
        gdb \
        valgrind \
        clang \
        clangd \
        clang-format \
        lld \
        openssl \
        libssl-dev \
        libboost-all-dev \
        libeigen3-dev \
        libopencv-dev \
        libpcl-dev \
        liboctomap-dev \
        libglew-dev \
        libgl-dev \
        libglu1-mesa-dev \
        libepoxy-dev \
        mesa-common-dev \
        libwayland-dev \
        libxkbcommon-dev \
        libx11-dev \
        libxcursor-dev \
        libxext-dev \
        libxfixes-dev \
        libxi-dev \
        libxinerama-dev \
        libxrandr-dev \
        libjpeg-dev \
        libpng-dev \
        libudev-dev \
        libusb-1.0-0-dev \
        libv4l-dev \
        libgtk-3-dev \
        libglfw3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --branch v0.9.2 --depth 1 \
        https://github.com/stevenlovegrove/Pangolin.git /tmp/Pangolin \
    && cmake -S /tmp/Pangolin -B /tmp/Pangolin/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TOOLS=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_PANGOLIN_PYTHON=OFF \
    && cmake --build /tmp/Pangolin/build --parallel \
    && cmake --install /tmp/Pangolin/build \
    && rm -rf /tmp/Pangolin

RUN git clone --branch v2.56.5 --depth 1 \
        https://github.com/IntelRealSense/librealsense.git /tmp/librealsense \
    && cmake -S /tmp/librealsense -B /tmp/librealsense/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_TOOLS=OFF \
        -DBUILD_PYTHON_BINDINGS=OFF \
        -DBUILD_UNIT_TESTS=OFF \
        -DFORCE_RSUSB_BACKEND=ON \
        -DCMAKE_C_COMPILER=clang \
        -DCMAKE_CXX_COMPILER=clang++ \
        -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=lld \
        -DCMAKE_MODULE_LINKER_FLAGS=-fuse-ld=lld \
    && cmake --build /tmp/librealsense/build --parallel \
    && cmake --install /tmp/librealsense/build \
    && rm -rf /tmp/librealsense

WORKDIR /workspaces/ORB_SLAM3

USER vscode
