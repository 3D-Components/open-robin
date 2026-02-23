################################################################################
# Stage 1: Build open62541 OPC UA library
################################################################################
FROM ubuntu:24.04 AS open62541-builder

RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3 \
    && rm -rf /var/lib/apt/lists/*

# Clone and build open62541 v1.4.14
RUN cd /tmp && \
    git clone --branch v1.4.14 --depth 1 https://github.com/open62541/open62541.git && \
    cd open62541 && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DUA_ENABLE_ENCRYPTION=OFF \
        -DUA_ENABLE_SUBSCRIPTIONS=ON \
        -DUA_ENABLE_METHODCALLS=ON \
        -DCMAKE_INSTALL_PREFIX=/opt/open62541 && \
    make -j$(nproc) && \
    make install

################################################################################
# Stage 2: Main Vulcanexus image
################################################################################
FROM eprosima/vulcanexus:jazzy-desktop-4.3.1

RUN curl -sSL raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

# Install essential packages
RUN apt-get update -q && apt-get install -y \
    gnupg2 \
    iputils-ping \
    usbutils \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-networkx \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-venv \
    python3-pandas \
    python3-numpy \
    python3-matplotlib \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages and other dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-ur \
    ros-jazzy-ur-simulation-gz \
    ros-jazzy-moveit \
    ros-jazzy-moveit-py \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-ros2controlcli \
    && rm -rf /var/lib/apt/lists/*

# Copy open62541 from builder stage
COPY --from=open62541-builder /opt/open62541 /usr/local

# Update library cache for open62541
RUN ldconfig

# Install additional Python dependencies
COPY vulcanexus_ws/requirements.txt /tmp/requirements.txt

# Create virtual environment with access to system packages and install dependencies
RUN python3 -m venv --system-site-packages /opt/venv && \
    /opt/venv/bin/pip install --upgrade pip && \
    /opt/venv/bin/pip install -r /tmp/requirements.txt

# Set the virtual environment path
ENV PATH="/opt/venv/bin:$PATH"
ENV PYTHONPATH="/opt/venv/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages"

COPY vulcanexus_ws/ws_setup.sh /workspace/ros2_packages/ws_setup.sh
RUN chown 1000:1000 /workspace/ros2_packages

WORKDIR /workspace/ros2_packages