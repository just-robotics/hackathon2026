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
      python3-pygame \
      python3-serial \
      iputils-ping \
      iproute2 \
      vim \
      nano \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-teleop-twist-keyboard \
      ros-humble-teleop-twist-joy \
      ros-humble-usb-cam \
      ros-humble-foxglove-bridge \
      ros-humble-imu-tools \
      ros-humble-topic-tools \
      ros-humble-xacro \
      ros-humble-tf-transformations \
      ros-humble-rosbag2-storage-mcap \
      ros-humble-ros-gz-sim \
      && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble officially targets Gazebo Fortress. The OSRF repository provides
# the Humble bridge rebuilt for Gazebo Harmonic as ros-humble-ros-gzharmonic.
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update -y && \
    conflicting_packages="$(dpkg-query -W -f='${binary:Package}\n' 'ros-humble-ros-gz*' 2>/dev/null || true)" && \
    if [[ -n "${conflicting_packages}" ]]; then \
      apt-get remove -y ${conflicting_packages}; \
    fi && \
    apt-get install -y --no-install-recommends \
      gz-harmonic \
      ros-humble-ros-gzharmonic \
      unzip && \
    rm -rf /var/lib/apt/lists/*

# RGL Gazebo Plugin (Harmonic): GPU raycast + Livox Mid-360 non-repetitive pattern.
ARG RGL_VERSION=v0.2.0-harmonic
ARG RGL_UBUNTU=22
RUN curl -fsSL -o /tmp/rgl.zip \
      "https://github.com/RobotecAI/RGLGazeboPlugin/releases/download/${RGL_VERSION}/RGLGazeboPlugin_ubuntu${RGL_UBUNTU}.zip" && \
    unzip -q /tmp/rgl.zip -d /tmp/rgl && \
    mkdir -p /opt/rgl/plugins /opt/rgl/gui && \
    cp /tmp/rgl/RGLServerPlugin/*.so /opt/rgl/plugins/ && \
    cp /tmp/rgl/RGLVisualize/*.so /opt/rgl/gui/ && \
    rm -rf /tmp/rgl /tmp/rgl.zip && \
    git clone --depth 1 --branch ${RGL_VERSION} \
      https://github.com/RobotecAI/RGLGazeboPlugin.git /tmp/rgl_src && \
    cp -a /tmp/rgl_src/lidar_patterns /opt/rgl/lidar_patterns && \
    rm -rf /tmp/rgl_src

COPY src ${WORKDIR}/src

# sensing dependencies
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/livox_sdk && \
    mkdir /tmp/livox_sdk_build && \
    cmake -S /tmp/livox_sdk -B /tmp/livox_sdk_build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/livox_sdk_build --parallel && \
    cmake --install /tmp/livox_sdk_build && \
    rm -rf /tmp/livox_sdk /tmp/livox_sdk_build

RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git ${WORKDIR}/src/livox_ros_driver2
WORKDIR ${WORKDIR}/src/livox_ros_driver2
RUN mv package_ROS2.xml package.xml && \
    cp -rf launch_ROS2/ launch/
WORKDIR ${WORKDIR}

RUN source ${SOURCE} && colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release

RUN echo "source ${SOURCE}" >> /root/.bashrc && \
    echo "source ${WORKDIR}/install/setup.bash" >> /root/.bashrc

# autocompletion for git
RUN echo "source /usr/share/bash-completion/completions/git" >> ~/.bashrc

CMD ["bash"]
