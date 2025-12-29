# Speed Control
For high performance, precision and safe operation, you need a good speed control system.

The basic robot is small and tank-tread driven. We want to be able to go slow, fast, accelerate at reliable rates, and detect things like slippage.

The system should support online calibration and automatic tuning.

We can use the built in compass for rough validation of turning, and we can use the distance sensors and wheel sensors to help calibrate things like wheel slip.

The user shouldn't have to tune PIDs manually.

## Implementation Plan

### Step 1: Open-Loop Characterization ✅ COMPLETE
**Goal:** Build feedforward model and get initial PI gains

**Implementation Notes:**
- Created `OpenLoopCharacterizationMode` task class in [`src/main.cpp`](src/main.cpp#L979-L1107)
- Implemented voltage-to-PWM conversion: `pwm = voltage / v_bat`
- Commands sequence: 2V, 4V, 6V, 8V, 10V (2s settle, 1s measure each)
- Logs to existing `/white_crash/update` topic (uses PWM and battery voltage)
- Added to FSM with `"open-loop"` event trigger
- Created [`scripts/characterize_motors.py`](scripts/characterize_motors.py) for data collection and analysis

**Results (from actual test run):**
- **Left Motor:** `V = 0.295 + 2.759 * v` (V/(m/s))
- **Right Motor:** `V = 0.247 + 2.841 * v` (V/(m/s))
- Motors well-matched (2.76 vs 2.84 gain ratio)
- ~0.3V offset indicates friction/dead-zone
- **Recommended Initial Gains:**
  - `P = 0.56 V/(m/s)` (conservative, 20% of feedforward)
  - `I = 0.28 V/(m/s²)` (eliminate steady-state error)
  - `D = 0.0` (not needed for velocity control)

**Files Modified:**
- [`src/main.cpp`](src/main.cpp): Added OpenLoopCharacterizationMode class
- [`scripts/characterize_motors.py`](scripts/characterize_motors.py): Analysis script
- [`scripts/motor_characterization_results.txt`](scripts/motor_characterization_results.txt): Results
- [`scripts/motor_characterization_plot.png`](scripts/motor_characterization_plot.png): Plots

**✅ APPROVED:** Open-loop model shows excellent fit, motors well-matched, ready for Step 2

---

### Step 2: Feedforward + PI Framework + Differential Drive ✅ COMPLETE
**Goal:** Add infrastructure with voltage-based control

**Implementation Summary:**
- **PIController class** ([src/main.cpp](src/main.cpp#L1045-L1147)):
  - Feedforward model: `V_ff = 0.271 + 2.800 * v`
  - Gains: `P=0.56, I=20.0` V/(m/s²) (I increased for dead zone breakthrough)
  - Direction-aware integral reset on forward↔reverse transitions
  - Anti-windup with back-calculation when output saturates
  - Voltage-to-PWM conversion: `pwm = voltage / v_bat`

- **Differential drive with closed-loop angular velocity control** ([src/main.cpp](src/main.cpp#L1178-L1204)):
  - Converts `(v_linear, ω_angular)` to `(v_left, v_right)`
  - Uses BNO055 gyroscope Z-axis for measured angular velocity
  - Separate PI controller for omega: `k_p=0.5, k_i=2.0`
  - Track width: 0.20m (approximate, to be calibrated)

- **Twist control API** ([src/main.cpp](src/main.cpp#L1221-L1270)):
  - `set_twist_target()` - set desired velocities
  - `enable_twist_control()` - initialize controllers
  - `disable_twist_control()` - stop and reset
  - `update_twist_control()` - main control loop (10ms intervals)
  - Does NOT affect RC control (HandMode) - that remains direct PWM

- **PIControlTestMode task** ([src/main.cpp](src/main.cpp#L1602-L1686)):
  - 8-step test sequence: stopped, 0.5 m/s, 1.0 m/s, decelerate, stop, reverse, rotate left/right (±3 rad/s)
  - Triggered by SC button press (hand → pi-test mode)
  - Logs via existing `/white_crash/update` topic

- **Monitoring script** ([scripts/monitor_pi_control.py](scripts/monitor_pi_control.py)):
  - Real-time velocity tracking display
  - Error statistics and performance metrics
  

**Next Steps:**
- Test on racks to verify basic tracking
- Adjust gains if needed (P and I)
- Calibrate track width using rotation tests
- Proceed to Step 3 for acceleration feedforward and auto-tuning

**✅ READY FOR TESTING**

---

### Step 3: Add Commanded Velocity Logging + Acceleration Feedforward
**Goal:** Enable tracking error analysis and add acceleration compensation

**Phase 3A: Add Logging (REQUIRED FIRST)**

Update [Update.msg](extra_packages/white_crash_msgs/msg/Update.msg) to add:
```
float32 twist_target_linear         # Commanded linear velocity (m/s)
float32 twist_target_angular        # Commanded angular velocity (rad/s)
float32 twist_target_accel_linear   # Commanded linear acceleration (m/s²)
float32 twist_target_accel_angular  # Commanded angular acceleration (rad/s²)
float32 v_left_target               # Commanded left wheel velocity (m/s)
float32 v_right_target              # Commanded right wheel velocity (m/s)
```

Tasks:
- Run `PlatformIO: Full Clean` to rebuild ROS messages
- Update main loop publishing code to populate new fields:
  - When `twist_control_enabled == true`: set to actual target values
  - When `twist_control_enabled == false`: set to `NAN` (not in twist control mode)
- Update [scripts/monitor_pi_control.py](scripts/monitor_pi_control.py) to display tracking error and acceleration
- Run PIControlTestMode on racks to validate current performance

**Phase 3B: Acceleration Feedforward** ✅ COMPLETE

**Implementation Summary:**
- Added `ff_accel` parameter to PIController class (initial value 0.7 V/(m/s²))
- Added global acceleration limits: `max_linear_accel = 1.0 m/s²`, `max_angular_accel = 2.0 rad/s²`
- Updated `set_twist_target()` signature to accept optional acceleration parameters:
  ```cpp
  void set_twist_target(float v_linear, float omega_angular, 
                        float accel_linear = 0.0, 
                        float accel_angular = 0.0)
  ```
- Implemented ramping logic in `update_twist_control()`:
  - When `accel == 0.0`: applies internal ramping at configured max acceleration
  - When `accel != 0.0`: uses explicit acceleration for feedforward
  - Ramping applied separately for linear and angular velocities
- Added acceleration feedforward to voltage calculation:
  ```cpp
  v_total = v_ff + ff_accel*accel + v_pi
  ```
- Added global variables for logging actual ramped/effective values:
  - `twist_effective_linear/angular` - post-ramping velocities
  - `twist_effective_accel_linear/angular` - actual accelerations applied
  - `v_left_target_effective/v_right_target_effective` - wheel targets after kinematics
- Updated ROS message publishing to log effective values

**Files Modified:**
- [src/main.cpp](src/main.cpp): PIController, twist control API, update_twist_control(), logging
- All changes maintain backward compatibility - existing code works with default `accel=0.0`

**Use Cases Now Supported:**
- **External commands (ROS `cmd_vel`):** Automatic smooth ramping at 1.0 m/s²
- **Smart position control:** Can specify explicit acceleration for better tracking
- **Existing code:** Works unchanged with automatic smoothing

**Next Steps:**
- Build and test basic operation with new acceleration feedforward
- Proceed to Phase 3C for acceleration tuning tests

**✅ READY FOR TESTING**

---

**Phase 3B: Acceleration Feedforward (OLD - REPLACED)**

**API Design:**
```cpp
/**
 * Set twist velocity targets with optional acceleration specification.
 * 
 * @param v_linear Linear velocity (m/s), positive = forward
 * @param omega_angular Angular velocity (rad/s), positive = counter-clockwise
 * @param accel_linear Expected linear acceleration (m/s²):
 *                     - 0.0 = no explicit intent, apply internal ramping (default)
 *                     - Non-zero = explicit acceleration for feedforward
 * @param accel_angular Expected angular acceleration (rad/s²), similar behavior
 */
void set_twist_target(float v_linear, float omega_angular, 
                      float accel_linear = 0.0, 
                      float accel_angular = 0.0);
```

**Implementation Tasks:**
- Add `ff_accel` parameter to PIController (~0.5-1.0 V/(m/s²))
- Add global acceleration limits: `max_linear_accel = 1.0 m/s²`, `max_angular_accel = 2.0 rad/s²`
- In `update_twist_control()`:
  - If `accel_linear == 0.0`: Apply internal ramping with `max_linear_accel` limit
  - If `accel_linear != 0.0`: Use explicit acceleration for feedforward
  - Same logic for angular acceleration
- Add to PIController voltage: `v_total = v_ff + a_ff*a_cmd + v_pi`
- Update `set_twist_target()` signature to accept acceleration parameters

**Use Cases:**
- **External commands (ROS `cmd_vel`, joystick):** Call `set_twist_target(1.0, 0.0)` → internal ramping applied
- **Smart position control (`GoToCanMode`):** Compute acceleration, call `set_twist_target(v, ω, a_computed, 0.0)` → uses explicit feedforward
- **Existing code:** Works unchanged with default `accel=0.0` → gets automatic smoothing

**Verification:**
- External velocity steps ramp smoothly at configured rate
- `GoToCanMode` deceleration tracking improves (no PI windup during braking)

**Phase 3C: Tuning Tests**

Tasks:
- Create `AccelTuningTestMode` task
- Run ramped velocity tests: 0→1→2→3 m/s with various ramp rates (0.5, 1.0, 2.0 m/s²)
- Python script analyzes tracking error and tunes: `ff_accel`, and possibly `P`/`I`
- Test on ground in straight line

**Verification:** 
- Track ramped velocity profiles with < 5% error
- Minimal overshoot during velocity changes
- Good response across acceleration rates

**✋ APPROVAL POINT:** Review tuned gains and acceleration tracking before rotational tests

---

### Step 4: Tune Rotational Motion  
**Goal:** Optimize for pure rotation (v = 0, ω ≠ 0)

**Tasks:**
- Run step response for angular velocity
- May need separate gains or gain scheduling
- Use compass to validate turn angles

**Test:** Rotate in place 90°, 180°, 360°
**Verification:**
- Compass heading matches commanded rotation ±5°
- No significant drift

**✋ APPROVAL POINT:** Review rotational performance

---

### Step 5: Combined Motion & Surfaces
**Goal:** Test realistic scenarios

**Tasks:**
- Waypoint following (curves)
- Test on: concrete, grass, carpet
- Adjust gains if needed per surface

**Test:** Auto mode waypoint navigation
**Verification:**
- Successfully completes route
- Stays within trajectory bounds

**✋ APPROVAL POINT:** Review multi-surface performance

---

### Step 6: Self-Tuning & Advanced Features
**Goal:** Make the system continuously adapt and optimize itself

**Tasks:**
- **Continuous model adaptation:**
  - Online feedforward model updates (voltage-to-speed curves adapt to battery level, wear, terrain)
  - Rotation offset calibration (compensate for motor differences)
  - Automatic gain adaptation based on performance metrics
- **Acceleration profiling:**
  - Trapezoidal or S-curve velocity profiles for smooth motion
  - Jerk limiting for passenger comfort
- **Load and terrain sensing:**
  - Integrate with StuckChecker for load detection
  - Detect surface changes (grass vs concrete) and adapt gains/feedforward
  - Wheel slip detection and compensation
- **Performance monitoring:**
  - Track tracking error statistics
  - Auto-trigger re-tuning if performance degrades

**Test:** Long-term operation across varied conditions, battery states, surfaces
**Verification:** 
- System maintains performance as battery drains
- Adapts to different surfaces automatically
- Rotation accuracy improves over time
- No manual re-calibration needed

---

## Design Notes

- **Voltage-based control:** Controller outputs voltage commands, more stable across battery levels
- **Auto-tuning:** Ramped velocity tests with offline analysis (Python)
- **Max speed:** < 5 m/s unloaded
- **Control architecture: Velocity FF + Acceleration FF + PI**
  ```
  voltage = velocity_feedforward(v_cmd) + accel_feedforward(a_cmd) + P*error + I*integral
  ```
  - Velocity feedforward handles steady-state (~80% of work)
  - Acceleration feedforward handles transients (~15% of work)
  - PI corrects for disturbances, model errors, terrain changes (~5%)
  - No D term needed (friction provides velocity damping)
  - I term eliminates steady-state error
- **Initial gains:** P ≈ 1-2 V/(m/s), I ≈ 0.5-1.0 V/(m/s·s), accel_ff ≈ 0.5-1.0 V/(m/s²), D=0
  - Lower P/I than pure PI since feedforward handles most work
- **Differential drive from start:** Avoids re-tuning later

