FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    python3-colcon-common-extensions \
    python3-argcomplete \
    vim \
    nano \
    tree \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /subscriber_ws

COPY mnist_subscriber ./src/mnist_subscriber

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select mnist_subscriber --symlink-install

SHELL ["/bin/bash", "-c"]

CMD source /opt/ros/humble/setup.bash && \
    source /subscriber_ws/install/setup.bash && \
    ros2 run mnist_subscriber mnist_subscriber