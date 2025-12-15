#!/bin/bash

docker build -t mnist_publisher .

docker run --rm \
    -it \
    --net host \
    -e ROS_DOMAIN_ID=0 \
    --name mnist_publisher \
    mnist_publisher \