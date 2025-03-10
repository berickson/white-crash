#! /bin/bash

while true
do
  docker run -it --rm \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    --privileged \
    --net=host \
    microros/micro-ros-agent:jazzy \
    udp4 --port 8888 -v3

  sleep 5
done