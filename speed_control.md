# Speed Control
For high performance, precision and safe operation, you need a good speed control system.

The basic robot is small and tank-tread driven. We want to be able to go slow, fast, accelerate at reliable rates, and detect things like slippage.

The system should support online calibration and automatic tuning.

We can use the built in compass for rough validation of turning, and we can use the distance sensors and wheel sensors to help calibrate things like wheel slip.

The user shouldn't have to tune PIDs manually.

## Implementation Plan

### Step 1: Open-Loop Characterization
**Goal:** Build feedforward model and get initial PI gains

**Tasks:**
- Create open-loop test mode (FSM state)
- Command fixed voltages (2V, 4V, 6V, 8V, 10V)
- Log commanded voltage and steady-state velocity to ROS
- Fit feedforward model: `voltage = f(velocity)` (linear or polynomial)
- Calculate initial P gain: `P ≈ 1-2 V/(m/s)` (conservative, since feedforward does most work)

**Test:** Robot on racks, verify both wheels respond to voltage commands
**Output:** 
- Feedforward function/lookup table
- Initial PI gains (P ≈ 1-2, I ≈ 0.5)

**✋ APPROVAL POINT:** Review open-loop model fit quality before proceeding

---

### Step 2: Feedforward + PI Framework + Differential Drive
**Goal:** Add infrastructure with voltage-based control

**Tasks:**
- Implement PI controller class (voltage output, not PWM)
- Implement feedforward model from step 1: `voltage = feedforward(v_target) + P*error + I*integral`
- Add differential drive kinematics: `(v_linear, ω_angular) → (v_left, v_right)`
- Total voltage converts to PWM: `pwm = voltage_command / v_bat`
- Start with conservative PI gains (D=0, not needed for velocity control)
- Add step response test mode for auto-tuning

**Test:** On racks, verify each wheel can track a setpoint well
**Verification:** Can command `v=1.0 m/s, ω=0` with feedforward doing most work, PI correcting errors

**✋ APPROVAL POINT:** Review feedforward+PI response before tuning

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

