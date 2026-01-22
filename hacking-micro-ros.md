# Hacking on micro-ROS

As we have encountered many issues with micro-ROS causing system crashes on the robot, it may be necessesary to make modifications for robustness. Below gives workflow guidance for this.

## Overview

micro-ROS is a multi-layered system. Bugs can exist in different repositories:

| Layer | Repository | Description |
|-------|------------|-------------|
| PlatformIO wrapper | [micro-ROS/micro_ros_platformio](https://github.com/micro-ROS/micro_ros_platformio) | Build scripts, transport code, ESP32 integration |
| micro-ROS Agent | [micro-ROS/micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent) | ROS 2 bridge node (wraps XRCE-DDS Agent) |
| XRCE-DDS Client | [eProsima/Micro-XRCE-DDS-Client](https://github.com/eProsima/Micro-XRCE-DDS-Client) | DDS protocol on MCU |
| XRCE-DDS Agent | [eProsima/Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) | Core agent logic |
| RCL/RCLC | various | Client library |

## Local Library Location

PlatformIO installs the library to:
```
.pio/libdeps/lolin_s3_mini/micro_ros_platformio/
```

The built sources (cloned during build) are in:
```
.pio/libdeps/lolin_s3_mini/micro_ros_platformio/build/mcu/src/
```

## Development Workflow

### Option 1: Quick In-Place Editing (for testing)

The library already has `.git`. You can edit directly:

```bash
cd .pio/libdeps/lolin_s3_mini/micro_ros_platformio
# Make changes, test
git diff  # see what you changed
```

**⚠️ Warning**: `pio lib install` or cleaning may overwrite changes!

### Option 2: Fork and Use Your Own (recommended)

1. Fork on GitHub: https://github.com/micro-ROS/micro_ros_platformio

2. Update `platformio.ini`:
   ```ini
   lib_deps =
       https://github.com/YOUR_USERNAME/micro_ros_platformio
   ```

3. Or for local development:
   ```ini
   lib_deps =
       /home/brian/projects/micro_ros_platformio
   ```

4. Clone your fork separately:
   ```bash
   cd ~/projects
   git clone git@github.com:YOUR_USERNAME/micro_ros_platformio.git
   cd micro_ros_platformio
   git remote add upstream https://github.com/micro-ROS/micro_ros_platformio
   ```

### Rebuilding After Changes

```bash
# Clean the micro-ROS library (forces rebuild)
pio run --target clean_microros

# Or use VS Code task:
# "PlatformIO: Full Clean (lolin_s3_mini)"

# Then rebuild
pio run
```

## Contributing Upstream

Both micro-ROS repos accept PRs via standard GitHub workflow:

1. **File an issue first** for large changes
2. **Fork the repo** on GitHub
3. **Sign your commits** with DCO (required):
   ```bash
   git commit -s -m "Fix: description"
   ```
4. **Open a PR** against the upstream repo

License: Apache-2.0

## Debugging Tips

- **ESP32 crashes/reboots**: likely in `micro_ros_platformio` or `Micro-XRCE-DDS-Client`
- **Agent-side issues**: check `micro-ROS-Agent` or `Micro-XRCE-DDS-Agent`
- **Transport bugs (WiFi/serial)**: look in `micro_ros_platformio/platform_code/arduino/`

## Current Version Check

```bash
cd .pio/libdeps/lolin_s3_mini/micro_ros_platformio
git log -1 --oneline
git ls-remote origin main | cut -f1  # compare to upstream
```
