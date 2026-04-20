FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install essential build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    wget \
    curl \
    lsb-release \
    gnupg2 \
    iproute2 \
    autoconf \
    autogen \
    libtool \
    libserial-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and Install IgH EtherCAT Master (Userspace Libraries only)
RUN mkdir -p /tmp/ethercat && cd /tmp/ethercat && \
    git clone --depth 1 https://gitlab.com/etherlab.org/ethercat.git . && \
    ./bootstrap && \
    ./configure --prefix=/usr/local/etherlab --disable-8139too --disable-e1000 --disable-e1000e --disable-r8169 --disable-generic --disable-kernel && \
    make && make install && \
    rm -rf /tmp/ethercat

# Install MoveIt 2 and related ROS 2 control packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ign-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rosbag2-py \
    ros-${ROS_DISTRO}-moveit-ros-perception \
    ros-${ROS_DISTRO}-warehouse-ros-sqlite \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-kdl-parser \
    ros-${ROS_DISTRO}-orocos-kdl-vendor \
    python3-numpy \
    libyaml-cpp-dev \
    libcap2-bin \
    && rm -rf /var/lib/apt/lists/*

# Layer 4: Ignition Gazebo & Integration Packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-interfaces \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Layer 3.5: Python Dependencies for Scan-N-Plan
RUN pip3 install --no-cache-dir \
    open3d \
    scipy \
    scikit-learn \
    matplotlib \
    pandas \
    # pyrealsense2 \
    opencv-python

# Grant network capabilities and configure system linker for privileged ROS 2 binaries
# Requirement: Binaries with capabilities ignore LD_LIBRARY_PATH, so we must add paths to ld.so.conf.d
RUN setcap cap_net_raw,cap_net_admin=eip /opt/ros/humble/lib/controller_manager/ros2_control_node \
    && echo "/opt/ros/humble/lib" > /etc/ld.so.conf.d/ros2.conf \
    && echo "/opt/ros/humble/lib/x86_64-linux-gnu" >> /etc/ld.so.conf.d/ros2.conf \
    && echo "/ros2_ws/install/soem_ros_control/lib" >> /etc/ld.so.conf.d/ros2.conf \
    && echo "/ros2_ws/install/soem_ethercat_driver/lib" >> /etc/ld.so.conf.d/ros2.conf \
    && ldconfig

# Set CycloneDDS as the default RMW
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Create a non-root user (Dynamic UID/GID provided at build time)
ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# Create workspace directory and set permissions
WORKDIR /ros2_ws
RUN chown -R $USERNAME:$USER_GID /ros2_ws

# Setup entrypoint
COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $USERNAME

# Initialize rosdep for the non-root user
RUN rosdep update

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
