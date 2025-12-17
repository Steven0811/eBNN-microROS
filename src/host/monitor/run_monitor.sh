#!/bin/bash

docker build -t ebnn_monitor .

docker run --rm \
    -it \
    --net host \
    --name ebnn_monitor \
    ebnn_monitor \