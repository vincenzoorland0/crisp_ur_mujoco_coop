ARG ROS_DISTRO=jazzy
ARG CRISP_CONTROLLERS_VERSION=1.1.0

FROM osrf/ros:${ROS_DISTRO}-desktop AS base

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Remove existing user/group if IDs already exist
RUN if getent passwd ${USER_UID}; then \
      userdel -r "$(getent passwd ${USER_UID} | cut -d: -f1)"; \
    fi && \
    if getent group ${USER_GID}; then \
      groupdel "$(getent group ${USER_GID} | cut -d: -f1)"; \
    fi

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    mkdir -p /home/${USERNAME}/.config && \
    chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}/.config

RUN apt-get update && \
    apt-get install -y sudo && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      vim \
      build-essential \
      cmake \
      wget \
      git \
      unzip \
      python3-pip \
      python3-venv \
      python3-flake8 \
      python3-rosdep \
      python3-setuptools \
      python3-vcstool \
      python3-colcon-common-extensions \
      python3-scipy \
      pkg-config \
      libpoco-dev \
      libeigen3-dev \
      libglfw3-dev \
      libgl1-mesa-dev \
      libgl1-mesa-dri \
      libx11-dev \
      libx11-6 \
      libxrandr2 \
      libxi6 \
      libxinerama1 \
      libxcursor1 \
      libxext6 \
      xorg-dev \
      mesa-utils \
      libopencv-dev \
      libpcl-dev \
      ros-${ROS_DISTRO}-urdf \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-controller-manager \
      ros-${ROS_DISTRO}-joint-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher-gui \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-pcl-ros \
      ros-${ROS_DISTRO}-perception-pcl \
      ros-${ROS_DISTRO}-pcl-conversions \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-urdfdom-py \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-ament-index-cpp \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp \
      ros-${ROS_DISTRO}-launch-param-builder && \
    rm -rf /var/lib/apt/lists/*

RUN ln -sf /usr/bin/python3 /usr/bin/python

USER ${USERNAME}
RUN mkdir -p /home/${USERNAME}/ros2_ws/src
WORKDIR /home/${USERNAME}/ros2_ws

# Optional empty build, similar to repo example
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build
RUN echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

FROM base AS ur

# Clone only UR repo so we can build ur_description
RUN git clone --branch ${ROS_DISTRO} https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver

# Optional override for your real-robot ros2_control xacro
COPY --chown=ros:ros config/ur.ros2_control.xacro \
  /home/ros/ros2_ws/src/Universal_Robots_ROS2_Driver/ur_robot_driver/urdf/ur.ros2_control.xacro

# Build only ur_description, then ignore the rest of the UR repo
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /home/ros/ros2_ws && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --packages-select ur_description && \
    find src/Universal_Robots_ROS2_Driver -mindepth 1 -maxdepth 1 -type d ! -name ur_description -exec touch {}/COLCON_IGNORE \;

FROM ur AS ur-overlay

ARG CRISP_CONTROLLERS_VERSION=1.1.0

# MuJoCo backend
COPY --chown=ros:ros config/mujoco_system_initial_positions.patch \
  /tmp/mujoco_system_initial_positions.patch
COPY --chown=ros:ros config/mujoco_system_effort_state.patch \
  /tmp/mujoco_system_effort_state.patch
COPY --chown=ros:ros config/mujoco_ros2_control_step_timing.patch \
  /tmp/mujoco_ros2_control_step_timing.patch
RUN git clone --branch jazzy https://github.com/dfki-ric/mujoco_ros2_control.git src/mujoco_ros2_control && \
    git -C src/mujoco_ros2_control apply /tmp/mujoco_system_initial_positions.patch && \
    git -C src/mujoco_ros2_control apply /tmp/mujoco_system_effort_state.patch && \
    git -C src/mujoco_ros2_control apply /tmp/mujoco_ros2_control_step_timing.patch && \
    if [ -d src/mujoco_ros2_control/franka_mujoco ]; then touch src/mujoco_ros2_control/franka_mujoco/COLCON_IGNORE; fi && \
    if [ -d src/mujoco_ros2_control/unitree_h1_mujoco ]; then touch src/mujoco_ros2_control/unitree_h1_mujoco/COLCON_IGNORE; fi

# Your repository
COPY . src/crisp_ur_demo

# CRISP controllers
RUN git clone --branch ${ROS_DISTRO} --depth 1 https://github.com/utiasDSL/crisp_controllers.git src/crisp_controllers
# Patch CRISP Cartesian controller to avoid parameter-service deadlock in controller_manager
COPY --chown=ros:ros /config/cartesian_controller.cpp \
  /home/ros/ros2_ws/src/crisp_controllers/src/cartesian_controller.cpp

# Install rosdeps only for the packages you actually need
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /home/ros/ros2_ws/install/setup.bash && \
    sudo apt-get update && \
    rosdep update && \
    rosdep install -q \
      --from-paths \
        src/mujoco_ros2_control/mujoco_ros2_control \
        src/crisp_controllers \
        src/crisp_ur_demo/crisp_ur_mujoco \
      --ignore-src --rosdistro ${ROS_DISTRO} -y && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      --packages-up-to mujoco_ros2_control crisp_controllers crisp_ur_mujoco
