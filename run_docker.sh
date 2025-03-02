#!/bin/bash

docker run -p 6080:80 \
    -v "D:/Studies:/shelfino_dir" \
    -v my_ros2_volume:/root \
    --security-opt seccomp=unconfined \
    --platform=linux/amd64 \
    --shm-size=12g \
    --memory=16g \
    --cpus=6 \
    --sysctl kernel.shmmax=2147483648 \
    --sysctl kernel.shmall=524288 \
    my_ros2_container_saved
