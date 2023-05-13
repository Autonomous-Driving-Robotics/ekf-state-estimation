FROM nvidia/cuda:11.7.0-cudnn8-runtime-ubuntu20.04


# Install basic ubuntu and python packages
RUN /bin/bash -c 'apt update; \
    apt install -y software-properties-common; \
    add-apt-repository ppa:deadsnakes/ppa; \
    apt update; \
    apt install -y python3-pip; \
    python3 -m pip install --upgrade pip; \
    python3 -m pip install pygame numpy carla==0.9.13; \
    useradd -m dev; \
    apt-get -y install libjpeg-turbo8 apt libtiff5'
#-------------------------------------------------

# Install ROS2
RUN /bin/bash -c 'apt update && apt install -y locales; \
    locale-gen en_US en_US.UTF-8; \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8; \
    export LANG=en_US.UTF-8; \
    add-apt-repository universe; \
    apt update && apt install -y curl; \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null; \
    apt update && apt -y upgrade'

RUN /bin/bash -c 'apt install -y ros-foxy-desktop python3-argcomplete ros-dev-tools ros-foxy-derived-object-msgs'
#-------------------------------------------------

USER dev
WORKDIR /home/dev/ros2_ws