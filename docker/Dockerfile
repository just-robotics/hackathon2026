FROM ghcr.io/autowarefoundation/autoware:universe-devel-cuda-humble-20260721

SHELL ["/bin/bash", "-c"]

ENV TZ=Europe/Moscow
ENV DEBIAN_FRONTEND=noninteractive

ENV WORKDIR=/autoware
ENV SOURCE=/opt/autoware/setup.bash

ARG VEHICLE_ID

RUN if [[ -z "${VEHICLE_ID:-}" ]]; then \
        echo "ERROR: VEHICLE_ID is not set"; \
        exit 1; \
    fi

WORKDIR ${WORKDIR}

# RU mirrors
RUN sed -i \
    -e 's|http://archive.ubuntu.com/ubuntu|http://ru.archive.ubuntu.com/ubuntu|g' \
    -e 's|http://security.ubuntu.com/ubuntu|http://ru.archive.ubuntu.com/ubuntu|g' \
    /etc/apt/sources.list && \
    sed -i \
    's|http://azure.archive.ubuntu.com/ubuntu|http://ru.archive.ubuntu.com/ubuntu|g' \
    /etc/apt/ubuntu-mirrors.list

RUN \
    apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
      git \
      sudo \
      ca-certificates \
      curl \
      gnupg \
      lsb-release \
      cmake \
      build-essential \
      sysstat \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-pip \
      python3-numpy \
      python3-pygame \
      python3-serial \
      iputils-ping \
      iproute2 \
      vim \
      nano \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rclpy \
      ros-humble-rosidl-default-generators \
      ros-humble-std-srvs \
      ros-humble-tf2-ros \
      ros-humble-teleop-twist-keyboard \
      ros-humble-teleop-twist-joy \
      ros-humble-usb-cam \
      ros-humble-foxglove-bridge \
      ros-humble-imu-tools \
      ros-humble-topic-tools \
      ros-humble-xacro \
      ros-humble-tf-transformations \
      ros-humble-rosbag2-storage-mcap \
      ros-humble-rviz2 \
      && rm -rf /var/lib/apt/lists/*

# Gazebo Classic 11. Формально EOL с января 2025, но выбран сознательно:
# только под него существуют плагины Livox, воспроизводящие настоящий
# non-repetitive паттерн Mid-360 (с полями tag/line и offset_time для деskew).
# GPU-растеризации, как в RGL, здесь нет -- лучи считает ODE на CPU.
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
      gazebo \
      libgazebo-dev \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-plugins \
      ros-humble-gazebo-ros \
      unzip && \
    rm -rf /var/lib/apt/lists/*

# GTSAM -- зависимость lio_sam (факторный граф). В apt для jammy его нет,
# ставим из PPA borglab. Сам lio_sam сейчас не собирается (локализация идёт из
# симулятора), но GTSAM оставлен, чтобы включение LIO-SAM не требовало
# пересборки половины образа.
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends software-properties-common && \
    add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
    apt-get update -y && \
    apt-get install -y --no-install-recommends \
      libgtsam-dev \
      libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

# sensing dependencies
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/livox_sdk && \
    mkdir /tmp/livox_sdk_build && \
    cmake -S /tmp/livox_sdk -B /tmp/livox_sdk_build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/livox_sdk_build --parallel && \
    cmake --install /tmp/livox_sdk_build && \
    rm -rf /tmp/livox_sdk /tmp/livox_sdk_build

RUN mkdir -p ${WORKDIR}/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git ${WORKDIR}/src/livox_ros_driver2
WORKDIR ${WORKDIR}/src/livox_ros_driver2
RUN mv package_ROS2.xml package.xml && \
    cp -rf launch_ROS2/ launch/
WORKDIR ${WORKDIR}

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
      ros-humble-nav2-mppi-controller \
      ros-humble-nav2-controller && \
    rm -rf /var/lib/apt/lists/*

COPY src ${WORKDIR}/src

RUN source ${SOURCE} && colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release


RUN echo "source ${SOURCE}" >> /root/.bashrc && \
    echo "source ${WORKDIR}/install/setup.bash" >> /root/.bashrc

# autocompletion for git
RUN echo "source /usr/share/bash-completion/completions/git" >> ~/.bashrc

CMD ["bash"]
