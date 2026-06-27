# =============================================================================
# SAAT — Jetson Nano Docker Image
# Base: ROS2 Foxy on Ubuntu 18.04 (arm64)
# =============================================================================
# Build:
#   docker build -t saat:latest .
#
# Run (with GPIO + camera access):
#   docker run --rm -it \
#     --privileged \
#     --device /dev/video0 \
#     --device /dev/i2c-1 \
#     -v /sys/bus/usb:/sys/bus/usb \
#     -v $(pwd)/saat_data:/saat_data \
#     -p 8080:8080 \
#     saat:latest
# =============================================================================

FROM nvcr.io/nvidia/l4t-base:r32.6.1

# ── System packages ────────────────────────────────────────────────────────
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS2 Foxy prerequisites
    curl gnupg2 lsb-release \
    # Build tools
    build-essential cmake git python3-pip python3-dev \
    # OpenCV system libs
    libopencv-dev python3-opencv \
    # I2C tools (for PCA9685 debug)
    i2c-tools libi2c-dev \
    # USB libs (for RealSense)
    libusb-1.0-0-dev udev \
    && rm -rf /var/lib/apt/lists/*

# ── ROS2 Foxy install ──────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) \
          signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
          http://packages.ros.org/ros2/ubuntu \
          $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-foxy-ros-base \
        ros-foxy-cv-bridge \
        ros-foxy-image-transport \
        ros-foxy-launch-ros \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies ────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    # RealSense SDK Python bindings (built from source for arm64)
    pyrealsense2 \
    # Servo driver
    adafruit-circuitpython-servokit \
    adafruit-blinka \
    # GPIO
    Jetson.GPIO \
    # Web dashboard
    flask \
    # Numeric
    numpy

# ── Intel RealSense SDK (librealsense2) ───────────────────────────────────
# Install pre-built arm64 binaries via apt
RUN apt-key adv --keyserver keyserver.ubuntu.com \
      --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCD && \
    add-apt-repository \
      "deb https://librealsense.intel.com/Debian/apt-repo \
       $(lsb_release -cs) main" && \
    apt-get update && apt-get install -y \
      librealsense2-utils \
      librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

# ── Workspace copy and build ───────────────────────────────────────────────
WORKDIR /saat_ws
COPY src/ src/

RUN . /opt/ros/foxy/setup.sh && \
    rosdep init && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --packages-select saat_interfaces saat_core

# ── Data volume ───────────────────────────────────────────────────────────
RUN mkdir -p /saat_data
VOLUME ["/saat_data"]

# ── Entrypoint ────────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "saat_core", "saat_launch.py"]
