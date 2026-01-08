# Motor Control

Precise and responsive motor control is paramount for high performance and accurate robots that rely on acceleration curves, need precise placement, etc.

Current focus: optimize for autonomous go-to-can contest task requiring sub-centimeter positioning.

## Problem Space

H-bridge motor control of a physical is more complex and non-linear the idealized force input function of theoritical PID system.

PWM motor control over h-bride has the following characteristics:

- There are 4 possible combinations of a / b inputs. One drives it forward, one in reverse, on coasts, and the other brakes by sinking power back into the system
- These input combinations are pulsed using PWM at frequencies typically in 10s of KHz
- Back-EMF means that the same input will have different force outputs depending on the currrent speed
- There is a start friction that must be overcome to start motor movement, and a dynamic friction to maintain movement
- The robot itself adds dynamical resistance depending on environment. For example, the motor has to work harder on a grade, or on carpet versus on a hard surface. 
- Speeding up and slowing down aren't symmetrical.
- Battery voltage directly affects motor output and battery voltage varies as the batteries drain


PIDs themselves have the following problems
- Motor doesn't act like an idealized force input function
- PID tends to try for steady state, but the real world needs dynamical behavior, like acceleration

Physical robot contraints
- Tanks treads are bumpy and don't give smooth response
- Tread slipping means that turning doesn't match what a differential drive model would suggest

# Implemented
- Feed forward
- Acceleration input to PID
- Basic PWM to Velocity characteriztion
- Simple switching to fast decay based on whether speed needs to increase or decrease
- Instrumentation includes wheel encoders, front facing TOF distance sensors, BNO055 9 axis IMU.  These all log to ROS2.

# Issues
- Current issue most critical for contest: go-to-can task requires precise position control (within 5mm)
- Deceleration overshoot affects final positioning accuracy
- Difficulties following deceleration curves - robot overshoots and undershoots
- Robot doesn't always react quickly to changing environments, e.g. uphill to downhill

# Approach

## Motor Control Strategy
- Implement continuous control function across all regimes: acceleration, speed maintenance, and deceleration
- Input: (v_current, v_target, error, acceleration_current, acceleration_target) → optimal PWM/braking strategy
- Not mode-based, but a unified function that smoothly transitions between:
  - Accelerating: forward PWM
  - Maintaining: balanced forward/coast
  - Decelerating: reduced forward, proportional braking, or reverse (depending on error magnitude)
- Question: For a given deceleration need, what's the optimal mix of reduced forward PWM vs. active braking vs. reverse torque?

## Slip Compensation for Turning
- Tank treads exhibit dynamic slip depending on turn radius and surface
- Point turns: 2-3x slip factor (effective track width 3x geometric)
- Large radius turns: ~1.09x slip (minimal)
- Model: `slip_factor = 1.0 + (max_slip - 1.0) × e^(-r / L)` (from prior research)
- Needs adaptive implementation using IMU feedback

## Testing & Optimization
- Automated physical tests that execute and log to ROS
- Methodical optimization: gradient descent or predictive methods over real-world metrics
- Phase 1 priority: Wall approach test (maps to go-to-can contest task)

## Test Scenarios (priority order)
1. **Wall positioning** - drive to exact distance from wall using TOF sensors
   - Metrics: overshoot/undershoot (target: 5mm), time, deceleration profile
2. **Speed trajectory following** - track prescribed velocity/acceleration profile
   - Metrics: speed deviation (target: 3%), acceleration tracking
3. **Low speed reliability** - smooth operation at minimal speeds
   - Metrics: minimum reliable speed (target: 0.1 m/s), smoothness
4. **Precision turning** - execute 90° or 360° turns
   - Metrics: angle accuracy (target: 2°), time, overshoot
5. Smoothly execute different turning at different rates. For exmaple, 2 radians per second, 8 radians per second. minimize angular velocity error
