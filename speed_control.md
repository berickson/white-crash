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
- Created `PIController` class in [`src/main.cpp`](src/main.cpp#L890-L964)
  - Feedforward model: `V_ff = 0.271 + 2.800 * v`
  - Initial gains: `P=0.56, I=0.28` (20% of feedforward slope)
  - Anti-windup: stops integration when output saturates
  - Voltage-to-PWM conversion: `pwm = voltage / v_bat`
- Implemented differential drive kinematics: `diff_drive_kinematics()`
  - Converts `(v_linear, ω_angular)` to `(v_left, v_right)`
  - Track width: 0.20m (approximate, to be calibrated in Step 4)
- Created `set_twist()` API for programmatic control
  - Main interface for autonomous modes
  - Reads speedometer feedback, runs PI controllers, applies motor commands
  - Does NOT affect RC control (HandMode) - that remains direct PWM
- Created `PIControlTestMode` task in [`src/main.cpp`](src/main.cpp#L1256-L1334)
  - Test sequence: stopped, 0.5 m/s, 1.0 m/s, decelerate, stop, reverse, rotate
  - Triggered by SC button press (hand → pi-test mode)
  - Logs all data via existing `/white_crash/update` topic
- Created [`scripts/monitor_pi_control.py`](scripts/monitor_pi_control.py)
  - Real-time monitoring of commanded vs actual velocities
  - Terminal output and optional matplotlib plots
  - Error statistics and tracking performance metrics

**Button Mapping:**
- **SC button** (from hand mode): Enter PI control test mode
- **SD button** (from hand mode): Enter open-loop characterization mode
- **SC button** (from pi-test mode): Enter compass calibration mode
- **RC stick movement**: Return to hand mode (from any test mode)

**Test Instructions:**
1. Build and upload firmware to robot
2. Start micro-ROS agent: `./start_microros_agent.sh`
3. Run monitor script: `./scripts/monitor_pi_control.py --plot`
4. Put robot on racks (wheels off ground) or in safe testing area
5. Press SC button on RC transmitter to start test
6. Monitor real-time velocity tracking
7. Test runs for ~35 seconds through full sequence
8. Review error statistics and plots

**Files Modified:**
- [`src/main.cpp`](src/main.cpp): Added PIController class, diff drive, set_twist(), PIControlTestMode
- [`scripts/monitor_pi_control.py`](scripts/monitor_pi_control.py): Monitoring and analysis script

**Next Steps:**
- Test on racks to verify basic tracking
- Adjust gains if needed (P and I)
- Calibrate track width using rotation tests
- Proceed to Step 3 for acceleration feedforward and auto-tuning

**✅ READY FOR TESTING**

---

### Step 3: Auto-Tune Straight Line Motion with Acceleration Feedforward
**Goal:** Optimize feedforward+PI for linear velocity (ω = 0) with near-perfect tracking

**Tasks:**
- Add acceleration feedforward: `voltage = v_ff(v) + a_ff(a) + P*error + I*integral`
- Implement smooth velocity profiles (not step changes) to generate clean commanded acceleration
- Run ramped velocity tests: 0→1→2→3 m/s with various ramp rates
- Log commanded velocity, commanded acceleration, actual velocity to ROS
- Python script analyzes response and tunes: velocity_ff, accel_ff gain, P, I
- Flash new gains and verify

**Test:** On ground, straight line driving with acceleration/deceleration
**Verification:** 
- Track ramped velocity profiles with < 5% error
- Go to wall test: measure actual vs commanded distance
- Minimal overshoot during velocity changes

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

