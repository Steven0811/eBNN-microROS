FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV ROS_DOMAIN_ID=0

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    python3-colcon-common-extensions \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /ebnn_ws

COPY ebnn_monitor ./src/ebnn_monitor

COPY tools ./tools

COPY models ./models

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select ebnn_monitor --symlink-install

SHELL ["/bin/bash", "-c"]

CMD \
    source /opt/ros/humble/setup.bash && \
    source /ebnn_ws/install/setup.bash && \
    echo "Checking PCA model..." && \
    if [ ! -f /ebnn_ws/models/pca_mnist.joblib ]; then \
        echo "Training PCA model..." && \
        python3 /ebnn_ws/tools/train_pca.py; \
    else \
        echo "PCA model exists, skip training."; \
    fi && \
    echo "Starting ebnn_monitor node..." && \
    ros2 run ebnn_monitor ebnn_monitor
    
