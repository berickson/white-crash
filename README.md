# Running

Launch micro-ros-agent:

```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v3
```

View log messages (note qos-depth):

```bash
ros2 topic echo /tank/log --qos-depth 50| grep -E --color=never -o "'.*\'"
```
