# White-Crash
This is for a 3d printed treaded rover robot built with low cost components. It was originally going to be a bunch of rovers that could link together, that's why the name "train". This personal work probably isn't intended for others to use, but you're free to try.

<iframe width="507" height="901" src="https://www.youtube.com/embed/mtOcJ6Oahmc" title="Robot Indoor Obstacle Test" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

# Features
- RC via ELRS  with telemetry including GPS, battery, compass, and mode feedback. Works well with RadioMaster MT12.
- Micro Ros logging
- GPS Waypoint Following
- Calibrate compass from the remote control
- "Unstick" during auto mode detects stuck wheels and try to unstick them

# Hardware

# Software

# Running

Launch micro-ros-agent:

```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v3
```

View log messages (note qos-depth):

```bash
ros2 topic echo /tank/log --qos-depth 50| grep -E --color=never -o "'.*\'"
```
