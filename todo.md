Todo
- create routes in the field
- wifi on a switch
- configuration web page. Could it be on the real web?
- platformio libraries for custom device code, etc.

o get ready for maker fair
- Test with phone network
- Obstacle avoidance on a switch
- Speed limit / battery limit

o Bug: upload seems to depend on wheel position
- fix with new board

- set waypoints from radio control
- test with remote network and laptop (no home network)
- Auto add intermediate waypoints if they are spaced too far apart
- add a camera
- Custom PCB
- 3d hardware mount points for all hardware
- Solder motor controller
- Add mpu6050 and try to avoid toppling over when you hit an obstacle

Next
o L (large)
- big enough to go over side-walk in front of house
- high clearance
- center track?
- nice camera
- mpu
- pi
- circuit board
- more power
- runs upside-down?

o Tank train

On Hold
- Electronic on switch
- Battery Compartment:  You shouldn't have to remove the body to change the battery
- Camera (waiting for ESP32Cam results from Seth)
- XShut to enable setting one at a time
- Try 600rpm motors (currently 200rpm)
- Drive wheel on top and passive on bottom to make more clearance
- Estimate current by voltage and RPM, use this fur current limiting - see https://electronics.stackexchange.com/a/347027
- faster gps library



Done
- Power switch on Chassis instead of lid
- Robust network connection - use state machine to reconnect
- Battery mounts for LiPo and/or hold in the LiIon better so they don't fall out when you go upside down
- micro-ros-agent errors, eprosima::fastcdr::exception::BadParamException, etc.
- enable running off-line, currently set_microros_wifi_transports blocks until connected
-Try out pro GPS with new library (it works and gets about 27 satellites but less reliable position)
- Turn car around so motors are in back and battery is in front
- Add VL53L1X ToF sensors
-- Individually set addresses Power with GPIO or use 

-VL53L1X ToF sensors slowing / hanging (needed pullups)
- Bigger holes for motor fastening boltes, maybe 0.3mm bigger
- Sloped bottom instead of round to 3d print better
- Wobbly wheels; thicker mating area? Axle Tube? Fixed axles in back?
- ESP Mount tabs in wrong place
- Wheel holes not printing properly
- add a body
- ROS on own thread, doesn't block main thread
- "dead man switch" - moving range of throttle or steering stops auto
o Upgrade 3d chassis
- Upgrade to dual core ESP-32 D mini (test first)
- Bearing rear wheels w/ external adjustable axles
- Bigger wheels
- Aim tof sensors upward
- Switch should go on chassis
