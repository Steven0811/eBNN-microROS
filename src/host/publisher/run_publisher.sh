#!/bin/bash

docker build -t mnist_publisher .

docker run --rm -it \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ROS_DOMAIN_ID=0 \
  mnist_publisher
