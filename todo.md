Auto add intermediate waypoints if they are spaced too far apart
ROS on own thread, doesn't block main thread
Estimate current by voltage and RPM, use this fur current limiting - see https://electronics.stackexchange.com/a/347027

White-Crash M 3d model
- Bigger holes for motor fastening boltes, maybe 0.3mm bigger
- Sloped bottom instead of round to 3d print better
- Wobbly wheels; thicker mating area? Axle Tube? Fixed axles in back?
- ESP Mount tabs in wrong place
- Wheel holes not printing properly


3d add a body
3d add bash-guard to chassis
3d hardware mount points for all hardware

Clean up wiring. Solder motor controller

Add mpu6050 and try to avoid toppling over when you hit an obstacle

On Hold
Waiting for hardware - Add VL53L1X ToF sensors
- Individually set addresses Power with GPIO or use XShut to enable setting one at a time
Try 600rpm motors (currently 200rpm)
Drive wheel on top and passive on bottom to make more clearance



Done
Try out pro GPS with new library (it works and gets about 27 satellites)
Turn car around so motors are in back and battery is in front