#!/bin/bash

docker build -t mnist_subscriber .

docker run --rm \
    -it \
    --net host \
    -e ROS_DOMAIN_ID=0 \
    --name mnist_subscriber \
    mnist_subscriber \