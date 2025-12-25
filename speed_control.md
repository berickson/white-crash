# Speed Control
For high performance, precision and safe operation, you need a good speed control system.

The basic robot is small and tank-tread driven. We want to be able to go slow, fast, accelerate at reliable rates, and detect things like slippage.

The system should support online calibration and automatic tuning.

We can use the built in compass for rough validation of turning, and we can use the distance sensors and wheel sensors to help calibrate things like wheel slip.

The user shouldn't have to tune PIDs

Plan

1. Get simple PID running on each side. We can test this with the robot on racks. Implement in FSM and intiate with a button / switch combination. The goal will be to keep to a velocity curve that includes starting, ramping, and slowing to a stop at an exact distance.
2. Tune and test on the ground. Here we can have a "go to wall" function that goes to a fixed distance from the wall as fast as possible.
3. Implement differential drive.
4. Test on different surfaces
5. Add force feedback and load detection

