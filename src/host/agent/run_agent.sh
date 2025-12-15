#!/bin/bash

PORTS=(
  /dev/ttyUSB0
)

DOMAIN_IDS=(
  0
)

AGENT_IMAGE="microros/micro-ros-agent:humble"

for i in "${!PORTS[@]}"; do
  port="${PORTS[$i]}"
  domain_id="${DOMAIN_IDS[$i]}"

  echo "Starting agent on $port with ROS_DOMAIN_ID=$domain_id"

  docker run -i --rm \
    --name "micro_ros_agent_$i" \
    --device="$port" \
    -e ROS_DOMAIN_ID="$domain_id" \
    "$AGENT_IMAGE" \
    serial --dev "$port" -v6 &
done