################################################################################
# Vulcanexus image for the ROBIN no-hardware welding demo
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
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages and other dependencies.
# ros-jazzy-ur provides ur_description + ur_robot_driver, needed by
# robin_description/urdf/ur.urdf.xacro to render the UR10e in the no-hardware demo.
RUN apt-get update && apt-get install -y \
    ros-jazzy-ur \
    ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python dependencies
COPY vulcanexus_ws/requirements.txt /tmp/requirements.txt

# Create virtual environment with access to system packages and install dependencies
RUN python3 -m venv --system-site-packages /opt/venv && \
    /opt/venv/bin/pip install --upgrade pip && \
    /opt/venv/bin/pip install -r /tmp/requirements.txt

# Set the virtual environment path
ENV PATH="/opt/venv/bin:$PATH"
ENV PYTHONPATH="/opt/venv/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages"

WORKDIR /workspace/ros2_packages

RUN chown 1000:1000 /workspace/ros2_packages