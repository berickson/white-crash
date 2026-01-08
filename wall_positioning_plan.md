# Wall Positioning Test - Implementation Plan

**Goal:** Create automated test infrastructure for precise wall approach to optimize motor control for sub-centimeter (5mm target) positioning accuracy.

**Maps to:** Go-to-can contest task

**IMPORTANT IMPLEMENTATION PROTOCOL:**
- ⚠️ **ONE STEP AT A TIME** - Do not proceed to next step without explicit authorization
- ⚠️ **STOP POINTS** marked throughout - wait for user approval before continuing
- ⚠️ **No coding ahead** - Only implement what is explicitly approved
- ⚠️ **This is dangerous hardware** - mistakes can damage motors or robot

---

## Phase 0: Motor Force Characterization (PREREQUISITE)

**Rationale:** Before we can optimize wall approach, we need to understand the motor's force/deceleration characteristics across different control modes. Current characterization only covers steady-state velocity. We need dynamic force profiles for acceleration, coasting, braking, and reverse.

**Testing Stages:**
1. **Initial tests on racks** (wheels off ground) - safer, repeatable
2. **Validation on ground** - real-world friction, surface interaction

### 0.1 Design Force Characterization Tests

**Deceleration Regimes (from gentlest to most aggressive):**
1. **Gentle slow:** Reduced forward PWM (100% → 0%)
2. **Moderate slow:** Coast (0% PWM, both pins LOW) 
3. **Aggressive slow:** Proportional braking (coast + brake mix, 0-100% brake duty)
4. **Emergency stop:** Reverse PWM + brake (danger - mechanical stress!)
5. **Hold mode:** 100% brake (only when stopped or near-stopped)

**Test 1: Reduced Forward PWM (Gentle Deceleration)**
- Get motor to steady velocity (e.g., 1.0 m/s at 100% PWM)
- Reduce forward PWM incrementally: 80%, 60%, 40%, 20%, 0%
- Measure resulting deceleration for each
- Extract: deceleration rate as function of (velocity, pwm_reduction)
- **This is the primary gentle deceleration mode**
- **Trials:** 5 PWM levels × 4 velocities (0.5, 1.0, 2.0, 3.0 m/s) = 20 runs

**Test 2: Coast Deceleration (0% PWM)**
- Get motor up to known velocity
- Command PWM = 0 (coast mode: both pins LOW)
- Measure velocity decay over time
- Extract: coast friction force as function of velocity
- **Baseline for moderate deceleration**
- **Trials:** 6 velocities (0.5, 1.0, 1.5, 2.0, 2.5, 3.0 m/s)

**Test 3: Proportional Braking (Coast + Brake Mix)**
- Get motor up to known velocity
- Apply brake at various duty cycles: 20%, 40%, 60%, 80%, 100%
- Measure deceleration rate for each
- Extract: braking force as function of (velocity, brake_duty)
- **Note:** Requires enhancement to DRV8833 - support proportional brake intensity
- **Trials:** 5 brake levels × 4 velocities (0.5, 1.0, 2.0, 3.0 m/s) = 20 runs

**Test 4: Reverse Torque (Emergency Stop - USE CAREFULLY)**
- At low-medium forward velocities (0.2, 0.4 m/s) - don't go too fast!
- Apply reverse PWM at LOW levels only: 10%, 20%, 30%
- Measure deceleration rate
- Extract: reverse force as function of (velocity, reverse_PWM)
- **Danger:** High mechanical stress - test conservatively at LOW speeds only
- **Trials:** 2 velocities × 3 PWM levels = 6 runs

**Test 5: Pure Brake (Maximum Deceleration)**
- Get motor up to known velocity
- Command full brake mode (both pins HIGH at 100%)
- Measure velocity decay
- Extract: maximum braking force as function of velocity
- **Upper bound for stopping force**
- **Trials:** 6 velocities (0.5, 1.0, 1.5, 2.0, 2.5, 3.0 m/s)

### 0.2 Enhance DRV8833 for Full Control Spectrum

**Location:** `include/drv8833.h`

**Current capabilities:**
- Forward drive: `go(rate > 0, fast_decay=false/true)`
- Coast: `go(0, fast_decay=false)`
- Full brake: `go(0, fast_decay=true)` or explicit brake()

**New requirements for complete deceleration control:**

**1. Add proportional braking:**
```cpp
void brake(float intensity) {
    // intensity ∈ [0, 1]
    // 0 = full coast (both pins LOW)
    // 1 = full brake (both pins HIGH)
    // 0 < intensity < 1 = PWM brake duty cycle
    
    if (intensity <= 0.0) {
        coast();
    } else if (intensity >= 1.0) {
        intensity = 1.0;
    }
    
    // Apply brake with PWM duty cycle
    int range = 1 << resolution;
    analogWrite(pin_fwd, intensity * range);
    analogWrite(pin_rev, intensity * range);
}
```

**2. Clarify existing reduced forward PWM:**
```cpp
// Existing go() method already supports reduced forward PWM:
// go(0.5, false) = 50% forward PWM = gentle deceleration from higher speed
// This is already the gentlest deceleration mode
```

**3. Hold mode (position holding when stopped):**
```cpp
// Just use brake(1.0) for full braking/holding

```

**This enables all test regimes**

---

## 🛑 STOP POINT 0.2

**Before proceeding to 0.3:**
- Review DRV8833 enhancement design
- User must explicitly approve implementation
- Verify this won't damage hardware

---

### 0.3 Create ForceCharacterizationMode

**Location:** `src/main.cpp`

**Functionality:**
```cpp
class ForceCharacterizationMode : public Task {
  // Test sequence (configurable via ROS params):
  // Stage 1: RACK TESTS (wheels off ground)
  //   1. Reduced forward PWM test (15 trials) - GENTLEST
  //   2. Coast test (5 trials) - MODERATE
  //   3. Proportional brake grid (15 trials) - AGGRESSIVE
  //   4. Pure brake test (5 trials) - VERY AGGRESSIVE
  //   5. Reverse torque (6 trials - CAREFUL!) - EMERGENCY ONLY
  // Stage 2: GROUND TESTS (validate on actual surface)
  //   - Repeat subset of critical tests (tests 1-3)
  //   - Compare to rack results (reveals ground friction, tread effects)
  
  // Each trial:
  //   - Accelerate to target velocity using existing twist control
  //   - Wait for steady state (1 sec)
  //   - Disable twist control, apply test condition directly to motors
  //   - Record velocity profile until stopped or timeout (3 sec)
  //   - Re-enable twist control
  //   - Rest 2 seconds between trials
  
  // Test parameters published to ROS for analysis
};
```

**Trigger:** New FSM event "force-char-rack" or "force-char-ground"

---

## 🛑 STOP POINT 0.3

**Before proceeding to 0.4:**
- Review ForceCharacterizationMode implementation
- User must explicitly approve code
- Discuss test sequence and safety limits
- No automatic execution without approval

---

### 0.4 Add Force Characterization Data to ROS

**Option A:** Use existing Update message, add fields:
```
# Force characterization
string force_test_mode      # "reduced_fwd", "coast", "prop_brake", "reverse", "pure_brake", "idle"
float32 force_test_param    # Test parameter (e.g., PWM level, brake duty)
uint32 force_test_trial     # Current trial number
float32 force_test_target_v # Target velocity for this trial
```

**Option B:** Create dedicated ForceCharacterization message (cleaner but more work)

---

## 🛑 STOP POINT 0.4

**Before proceeding to 0.5:**
- Review ROS message design
- User approval to modify messages (requires full rebuild)
- Decision on Option A vs Option B

---

### 0.5 Create Fully Automated Test & Analysis Pipeline

**File:** `scripts/run_force_characterization.py`

**Full automation - no manual intervention required:**

**Phase 1: Trigger and collect data**
- ROS node that orchestrates entire test
- Publishes FSM event to start force characterization mode
- Subscribes to `/white_crash/update` topic
- Automatically records ALL data during test runs
- Saves raw data to timestamped HDF5/pickle file
- Monitors test progress and provides status updates
- Automatically stops when all trials complete

**Phase 2: Automated analysis**
- Loads collected data
- Segments data by trial (detects start/end of each test)
- For each trial:
  - Identifies steady-state phase
  - Extracts deceleration phase
  - Computes instantaneous deceleration (dv/dt)
  - Filters noise (Savitzky-Golay or similar)
- Fits physics models to data:
  - **Reduced forward PWM:** `F_decel(v, pwm) = feedforward_model(pwm, v) - F_friction(v)`
  - **Coast:** `F_friction(v) = c0 + c1*v + c2*v²`
  - **Proportional Brake:** `F_brake(v, duty) = duty * (b0 + b1*v)`
  - **Pure Brake:** `F_brake_max(v) = b0 + b1*v`
  - **Reverse:** `F_reverse(v, pwm) = r0*pwm + r1*pwm*v`
- Statistical analysis:
  - R² values for model fits
  - Confidence intervals on parameters
  - Outlier detection and removal
  - Comparison of left vs right motor

**Phase 3: Automated visualization**
- Generates comprehensive report with plots:
  - Raw velocity vs time for all trials
  - Deceleration vs velocity (with fitted curves)
  - Force profiles for each mode
  - Residuals and fit quality metrics
  - Comparison: rack vs ground (if both run)
- Saves plots to timestamped directory

**Phase 4: Output for controller**
- Exports fitted model parameters to YAML/JSON
- Format ready for direct import into C++ controller
- Includes metadata: test date, conditions, fit quality

**Usage:**
```bash
# Run full automated pipeline (rack tests)
python3 run_force_characterization.py --stage rack

# Run ground tests
python3 run_force_characterization.py --stage ground

# Analyze existing data file
python3 run_force_characterization.py --analyze data_2026_01_08.pkl
```

**Output:**
- `force_char_data_YYYY_MM_DD_HH_MM.pkl` - Raw data
- `force_char_report_YYYY_MM_DD_HH_MM.pdf` - Automated report with all plots
- `force_models_YYYY_MM_DD.yaml` - Parameters for controller
- Terminal output with summary statistics

**No manual log inspection required - everything automated!**

---

## 🛑 STOP POINT 0.5

**Before proceeding to 0.6:**
- Review automated analysis script
- User approval to run on actual robot
- Verify safety limits in test sequences
- Test on racks first before ground

---

## 🛑 MAJOR STOP POINT - END OF PHASE 0

**Before ANY force characterization testing:**
- Complete code review of all Phase 0 components
- User must approve running tests on robot
- Start with rack tests only
- User observes first few trials manually
- Only proceed to ground tests after rack results reviewed

**Deliverables from Phase 0:**
- Force model parameters in YAML
- Automated analysis report with plots
- Confidence in characterized models

**Do NOT proceed to Phase 1 or Phase 2 without explicit authorization**

---

### 0.6 Build Force-to-Control Model

**Goal:** Create function that predicts acceleration for any control input

```cpp
float predict_deceleration(
  float v_current,
  MotorMode mode,  // COAST, BRAKE, REVERSE, FORWARD_REDUCED
  float param      // e.g., reverse_pwm or forward_pwm_reduction
) {
  // Use fitted models from characterization
  // Return predicted deceleration in m/s²
}
```

**Use in controller:** Given desired deceleration, compute optimal control mix

---

## Phase 1: Test Infrastructure & Baseline

### 1.1 Create WallApproachTestMode Class
**Location:** `src/main.cpp`

**Functionality:**
- Inherits from Task pattern (like PIControlTestMode)
- Test parameters (configurable via ROS parameters):
  - `start_distance`: Initial distance from wall (default: 1.0m)
  - `target_distance`: Goal distance from wall (default: 0.20m)
  - `approach_speed`: Target velocity during approach (default: 0.3 m/s)
  - `decel_distance`: Distance at which to begin deceleration (default: 0.3m)
  - `num_trials`: Number of consecutive runs per test (default: 10)
  
**Test Sequence:**
1. Read initial TOF distance (center sensor)
2. Command twist_control to approach wall at `approach_speed`
3. At `target_distance + decel_distance`, begin deceleration profile
4. Stop when TOF reads `target_distance`
5. Wait 1 second to settle
6. Record final distance, overshoot/undershoot, time to target
7. Back up to start_distance (slow, careful)
8. Repeat for num_trials
9. Publish summary statistics

**Trigger:** SC button press in hand mode → "wall-test" FSM event

---

## 🛑 STOP POINT 1.1

**Before proceeding to 1.2:**
- Review WallApproachTestMode implementation
- User approval for code changes
- Discuss FSM integration

---

### 1.2 Add Test Metrics to ROS Messages
**File:** `extra_packages/white_crash_msgs/msg/Update.msg`

Add fields:
```
# Wall positioning test metrics
bool wall_test_active           # Whether test is running
uint32 wall_test_trial_number   # Current trial (1-N)
float32 wall_test_start_dist    # Starting distance (m)
float32 wall_test_target_dist   # Target distance (m)
float32 wall_test_current_dist  # Current TOF reading (m)
float32 wall_test_final_dist    # Final distance after settling (m)
float32 wall_test_error         # Signed error: final - target (m)
float32 wall_test_time          # Time from start to stop (s)
string wall_test_status         # "idle", "approaching", "settling", "complete"
```

**Actions:**
- Run `PlatformIO: Full Clean (lolin_s3_mini)` after message changes
- Rebuild

---

## 🛑 STOP POINT 1.2

**Before proceeding to 1.3:**
- User approval for ROS message changes
- Confirm rebuild completed successfully
- Verify messages working in ROS

---

### 1.3 Create Fully Automated Wall Test Pipeline

**File:** `scripts/run_wall_test.py`

**Full automation - no manual intervention required:**

**Phase 1: Trigger and collect data**
- ROS node that orchestrates entire wall test
- Publishes FSM event to start wall approach test mode
- Subscribes to `/white_crash/update` topic
- Automatically records ALL data during test runs
- Saves raw data to timestamped file
- Monitors test progress (trial N of M)
- Automatically stops when all trials complete

**Phase 2: Automated analysis**
- Loads collected data
- Segments data by trial
- Per-trial metrics:
  - Position error (final_distance - target_distance) in mm
  - Overshoot magnitude (max distance past target)
  - Undershoot (stopped short)
  - Approach time (start to stop)
  - Velocity profile during deceleration
  - Distance at which deceleration began
- Aggregate statistics:
  - Mean error, std deviation, median
  - Max overshoot/undershoot across all trials
  - Success rate (within 5mm tolerance)
  - Consistency metrics

**Phase 3: Automated visualization**
- Generates comprehensive report:
  - Distance vs time for each trial (overlaid)
  - Velocity vs distance
  - Error distribution histogram
  - Box plots of key metrics
  - Comparison to baseline (if available)
- Saves to timestamped PDF report

**Phase 4: Summary output**
- Terminal summary with key results
- Comparison to success criteria
- Recommendations for parameter adjustments

**Usage:**
```bash
# Run full automated wall test (10 trials default)
python3 run_wall_test.py --trials 10

# With specific parameters
python3 run_wall_test.py --trials 20 --target-distance 0.15 --start-distance 1.5

# Analyze existing data
python3 run_wall_test.py --analyze wall_test_2026_01_08.pkl
```

**Output:**
- `wall_test_data_YYYY_MM_DD_HH_MM.pkl` - Raw data
- `wall_test_report_YYYY_MM_DD_HH_MM.pdf` - Automated report
- Terminal summary with pass/fail vs criteria

**No manual log inspection required!**

---

## 🛑 STOP POINT 1.3

**Before proceeding to 1.4:**
- Review automated test script
- User approval to run test
- Verify safety (proper start distance, speed limits)

---

### 1.4 Baseline Testing

**Action:**
- Run automated pipeline: `python3 run_wall_test.py --trials 10 --label baseline`
- Script automatically:
  - Triggers test on robot
  - Collects all data
  - Analyzes performance
  - Generates report
- Review automated report for baseline metrics
- No manual log inspection needed!

---

## 🛑 MAJOR STOP POINT - END OF PHASE 1

**Before proceeding to Phase 2:**
- Review baseline test results
- User must approve continuing with motor control improvements
- Baseline establishes what needs improvement
- Decision: Is Phase 0 force characterization needed first?

**Do NOT start Phase 2 implementation without explicit authorization**

---

## Phase 2: Design Continuous Motor Control Function

**Based on Phase 0 characterization data**

### 2.1 Implement Low-Level Motor Command Interface

**Location:** `include/drv8833.h`

**Current:** `void go(float rate, bool fast_decay)`

**New:** Keep low-level, expose direct control:
```cpp
void set_drive_mode(
  float forward_duty,   // [0, 1]
  float reverse_duty    // [0, 1]
);
// Combinations:
// (fwd>0, rev=0) = forward drive
// (fwd=0, rev>0) = reverse drive  
// (fwd>0, rev>0) = brake (proportional to min(fwd,rev))
// (fwd=0, rev=0) = coast
```

**Answer from Open Questions:** Implement at DRV8833 level

---

## 🛑 STOP POINT 2.1

**Before proceeding to 2.2:**
- Review low-level motor interface design
- User approval for DRV8833 modifications
- Discuss safety implications

---

### 2.2 Design Continuous Control Strategy

**Location:** PIController in `src/main.cpp`

**Input:** `(v_current, v_target, error, accel_current, accel_target)`

**Output:** `MotorCommand(forward_duty, reverse_duty)`

**Strategy using characterized force models:**

```cpp
// Compute desired force based on tracking error and feedforward
float F_desired = compute_desired_force(v_current, v_target, accel_target, error);

// Convert desired force to motor commands using characterized models
MotorCommand cmd;

if (F_desired > 0) {
  // Accelerating: use forward PWM
  cmd = force_to_forward_pwm(F_desired, v_current);
  
} else {
  // Decelerating: choose optimal braking strategy
  float F_needed = -F_desired;
  
  // Option 1: Can we achieve it by reducing forward PWM?
  float F_friction = get_coast_friction_force(v_current);
  if (F_needed <= F_friction * safety_margin) {
    cmd = compute_reduced_forward_pwm(F_needed, v_current);
  }
  
  // Option 2: Need active braking
  else if (F_needed <= get_max_brake_force(v_current)) {
    cmd = compute_brake_command(F_needed, v_current);
  }
  
  // Option 3: Need reverse torque
  else {
    cmd = compute_reverse_command(F_needed, v_current);
  }
}

return cmd;
```

**Key:** All force conversions based on empirical models from Phase 0, not arbitrary thresholds

---

## 🛑 STOP POINT 2.2

**Before proceeding to 2.3:**
- Review continuous control strategy design
- User approval for force-based controller
- Verify Phase 0 models are available and validated

---

### 2.3 Add Position Controller

**Answer from Open Questions:** Two-layer control architecture

**Outer Loop - Position Controller:**
```cpp
// Runs at lower frequency (e.g., 20 Hz)
void update_position_control() {
  float distance_error = target_distance - current_distance;
  
  // Compute desired velocity from position error
  float v_desired = k_p_position * distance_error;
  v_desired = clamp(v_desired, -max_velocity, max_velocity);
  
  // Compute desired acceleration (derivative of velocity setpoint)
  float accel_desired = (v_desired - v_desired_prev) / dt;
  accel_desired = clamp(accel_desired, -max_accel, max_accel);
  
  // Feed to velocity controller
  set_twist_target(v_desired, 0.0, accel_desired, 0.0);
}
```

**Inner Loop - Velocity Controller (existing):**
- Uses force-based control from Phase 2.2
- Runs at 10ms (100 Hz)

---

## 🛑 MAJOR STOP POINT - END OF PHASE 2

**Before proceeding to Phase 3:**
- Complete code review of all Phase 2 components
- User must approve testing new controller
- Test carefully with low speeds first
- Monitor for unexpected behavior
- Compare to baseline performance

**Do NOT proceed to optimization without explicit authorization**

---

## Phase 3: Parameter Optimization

### 3.1 Define Parameter Space

**From Phase 0 (fixed, characterized):**
- Force models: friction coefficients, brake characteristics, reverse torque model
- These are NOT tuned, they're measured physics

**Control parameters to optimize:**
- Position controller: `k_p_position`, `k_d_position` (if added), `max_velocity`, `max_accel`
- Velocity controller: Existing PI gains may need adjustment
- Strategy selection: `safety_margin` for when to use coast vs brake vs reverse
- Battery compensation: May need enhancement based on characterization

**Optimization targets:**
- Minimize: mean absolute error, max overshoot
- Minimize: approach time (secondary)
- Constraint: No undershoot (must not stop short)

### 3.2 Optimization Approach

**Phase 3A: Validate Force Models**
- Run wall approach tests with force-based controller
- Verify predicted vs actual deceleration matches
- If mismatch: iterate on force characterization

**Phase 3B: Optimize High-Level Parameters**
- Grid search on position controller gains (`k_p_position`, `max_accel`)
- These should be small search space since force models are physics-based
- Cost function: `J = w1*|mean_error| + w2*max_overshoot + w3*time`

**Phase 3C: Adaptive/Online Tuning (future)**
- Once basic system works, add online adaptation for:
  - Surface changes (carpet vs hard floor)
  - Battery degradation
  - Load variations

---

## 🛑 STOP POINT 3.2

**Before proceeding to 3.3:**
- Review optimization approach
- User approval for automated parameter sweep
- Set safe parameter ranges

---

### 3.3 Create Automated Optimization Harness

**File:** `scripts/optimize_wall_approach.py`

**Full automation - parameter sweep and optimization:**

**Functionality:**
- Load parameter configurations from YAML file
- For each configuration:
  - Set ROS parameters on robot
  - Trigger wall test: `run_wall_test.py` programmatically
  - Wait for completion
  - Collect results from automated analysis
- Compare all configurations using cost function
- Generate comparison plots and tables
- Report winner with confidence intervals
- Export optimal parameters for deployment

**Usage:**
```bash
# Run grid search
python3 optimize_wall_approach.py --config grid_search.yaml

# Run Bayesian optimization
python3 optimize_wall_approach.py --method bayesian --iterations 50
```

**Output:**
- `optimization_results_YYYY_MM_DD.pdf` - Full comparison report
- `optimal_parameters.yaml` - Best configuration
- Terminal summary with recommendations

**No manual parameter tuning or result comparison!**

---

## 🛑 MAJOR STOP POINT - END OF PHASE 3

**Before proceeding to Phase 4:**
- Review optimization results
- User approval for selected parameters
- Verify improvement over baseline
- Check for any anomalies in test runs

**Do NOT deploy to production without explicit authorization**

---

## Phase 4: Validation & Documentation

### 4.1 Final Validation
- Run 50 trials with optimized parameters
- Test on multiple surfaces (if relevant)
- Verify robustness to battery voltage variation
- Document final performance vs. baseline

### 4.2 Success Criteria
- **Primary:** Mean absolute error ≤ 5mm
- **Secondary:** 
  - Max overshoot ≤ 10mm
  - Std deviation ≤ 3mm
  - Approach time ≤ 3 seconds (from 1m)
  - Success rate ≥ 95% (within 5mm)

### 4.3 Documentation
- Update motor_control.md with results
- Document optimal parameters and reasoning
- Create tuning guide for future adjustments

---

## 🛑 FINAL STOP POINT - END OF PHASE 4

**Project complete - ready for deployment**
- All validation passed
- Documentation complete
- User final approval for production use

---

## Implementation Order

**UPDATED - Physics-First Approach:**

**⚠️ CRITICAL: Each step requires explicit user authorization before proceeding**

1. ⏳ **Phase 0.2:** Enhance DRV8833 → 🛑 STOP - Get approval
2. ⏳ **Phase 0.3:** Create ForceCharacterizationMode → 🛑 STOP - Get approval  
3. ⏳ **Phase 0.4:** Add ROS messages → 🛑 STOP - Get approval
4. ⏳ **Phase 0.5:** Create analysis script → 🛑 STOP - Get approval
5. ⏳ **Phase 0.5:** Run force tests (rack) → 🛑 STOP - Review results
6. ⏳ **Phase 0.5:** Run force tests (ground) → 🛑 STOP - Review results
7. ⏳ **Phase 0.6:** Build force-to-control models → 🛑 MAJOR STOP - End Phase 0
8. ⏳ **Phase 2.1:** Implement motor command interface → 🛑 STOP - Get approval
9. ⏳ **Phase 2.2:** Design force-based controller → 🛑 STOP - Get approval
10. ⏳ **Phase 2.3:** Add position controller → 🛑 MAJOR STOP - End Phase 2
11. ⏳ **Phase 1.1-1.2:** Wall test infrastructure → 🛑 STOP - Get approval
12. ⏳ **Phase 1.3-1.4:** Run baseline wall tests → 🛑 MAJOR STOP - End Phase 1
13. ⏳ **Phase 3.1-3.2:** Validate and optimize → 🛑 STOP - Get approval
14. ⏳ **Phase 3.3:** Run optimization → 🛑 MAJOR STOP - End Phase 3
15. ⏳ **Phase 4:** Final validation → 🛑 FINAL STOP - Project complete

**Key Change:** Force characterization (Phase 0) comes FIRST, before wall testing

**Implementation Protocol:**
- One step at a time
- Stop at every 🛑 marker
- Wait for explicit "proceed" authorization
- No coding ahead
- Review at every major phase boundary

---

## Open Questions (ANSWERED)

1. **Braking implementation:** Option A (DRV8833) vs Option B (PIController)?  
   **A:** DRV8833 level - expose low-level control

2. **Deceleration thresholds:** What initial values for threshold1/threshold2?  
   **A:** Don't use thresholds - characterize motors and use physics-based force models

3. **Position vs velocity control:** Hard switch or blended transition?  
   **A:** Two controllers - Position controller sets V and A, Velocity/Motor controller tracks them

4. **TOF sensor:** Which sensor to use? Center only, or average of all three?  
   **A:** Center only

5. **Battery compensation:** Current approach sufficient or needs enhancement?  
   **A:** Unknown - currently use "virtual voltage" and PWM scales with battery voltage. Need to understand behavior during deceleration. Phase 0 characterization should reveal this.

---

## Risk Mitigation
- **TOF noise:** Add filtering (moving average, Kalman) if readings unstable. 
- **Tread compliance:** Tank treads compress on contact - may need "contact detection" logic
- **Surface variation:** May need surface-specific tuning or adaptive gains
- **Sensor lag:** TOF sensors have ~50ms update rate - account for reaction time in deceleration planning
