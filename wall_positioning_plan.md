# Wall Positioning Test - Implementation Plan

**Goal:** Create automated test infrastructure for precise wall approach to optimize motor control for sub-centimeter (5mm target) positioning accuracy.

**Maps to:** Go-to-can contest task

**IMPORTANT IMPLEMENTATION PROTOCOL:**
- ⚠️ **ONE STEP AT A TIME** - Do not proceed to next step without explicit authorization
- ⚠️ **STOP POINTS** marked throughout - wait for user approval before continuing
- ⚠️ **No coding ahead** - Only implement what is explicitly approved
- ⚠️ **This is dangerous hardware** - mistakes can damage motors or robot

---

## Phase 0 Results: Characterized Force Models ✅ COMPLETE

**Test Date:** January 22, 2026  
**Test Conditions:** Ground tests, hardwood floor  
**Data File:** `test_outputs/run_2026_01_22_17_44_16/force_models_2026_01_22_17_44_16.yaml`

### Fitted Deceleration Models

All models follow: `decel = c0 + c1 * v` (m/s²)

| Mode | c0 (m/s²) | c1 (1/s) | Points | Velocity Range |
|------|-----------|----------|--------|----------------|
| **Coast** | 0.951 | 0.794 | 26 | 0.05 - 1.63 m/s |
| **Pure Brake** | 1.008 | 2.950 | 14 | 0.06 - 1.46 m/s |
| **Prop Brake 25%** | 1.478 | -0.078 | 4 | limited data |
| **Prop Brake 50%** | 1.140 | 0.280 | 17 | good coverage |

### Key Findings

**At 0.5 m/s:**
- Coast: 0.95 + 0.79×0.5 = **1.35 m/s²**
- Pure brake: 1.01 + 2.95×0.5 = **2.49 m/s²**
- Brake ratio: pure_brake/coast ≈ **1.8x stronger**

**At 1.0 m/s:**
- Coast: 0.95 + 0.79×1.0 = **1.74 m/s²**
- Pure brake: 1.01 + 2.95×1.0 = **3.96 m/s²**
- Brake ratio: pure_brake/coast ≈ **2.3x stronger**

**Stopping Distance (from v to 0):**
- Using constant decel approximation: `d = v²/(2*decel)`
- At 0.5 m/s with coast: ~9 cm
- At 0.5 m/s with pure brake: ~5 cm
- At 1.0 m/s with coast: ~29 cm
- At 1.0 m/s with pure brake: ~13 cm

### Implications for Control

1. **Proportional braking gives fine-grained deceleration control**
   - Can interpolate between coast (intensity=0) and pure_brake (intensity=1)
   - At v=0.5: decel range is 1.35 to 2.49 m/s² (1.14 m/s² of control authority)

2. **Velocity-dependent braking is significant**
   - Pure brake c1=2.95 means higher speeds brake much harder
   - This is good for safety but requires model-based control

3. **Current twist control doesn't use proportional braking**
   - Uses fast_decay (reverse PWM) which is different physics
   - Need to enhance twist control to use characterized brake models

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

## Phase 2: Enhance Twist Control with Model-Based Braking

**Architecture:** Two-layer control system using twist control as the motor-level interface.

### Control System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    POSITION CONTROL LAYER                       │
│  (WallApproachMode, GoToCanMode, etc.)                         │
│                                                                 │
│  Input: current_position, target_position, current_velocity     │
│  Output: v_target, accel_target (constant decel to goal)        │
│                                                                 │
│  Trajectory: v(d) = sqrt(2 * decel * distance_to_goal)         │
│              Arrive at target with v=0                          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    TWIST CONTROL LAYER                          │
│  (update_twist_control in main.cpp)                            │
│                                                                 │
│  Input: v_target, omega_target, accel_target, alpha_target      │
│                                                                 │
│  Logic:                                                         │
│    if (v_target > v_current + threshold):                      │
│      → ACCELERATE: use forward PWM with PI control              │
│    elif (v_target ≈ v_current):                                │
│      → MAINTAIN: use forward PWM with PI control                │
│    else:                                                        │
│      → DECELERATE: use proportional braking from force models   │
│        brake_intensity = compute_brake_for_decel(desired_decel) │
│                                                                 │
│  Output: motor.go() or motor.brake()                           │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    DRV8833 MOTOR DRIVER                         │
│                                                                 │
│  go(rate, fast_decay=false) → forward/reverse PWM               │
│  brake(intensity) → proportional braking (both pins HIGH)       │
└─────────────────────────────────────────────────────────────────┘
```

### 2.1 Add Brake Model to Twist Control

**Location:** `src/main.cpp`

**Add characterized brake coefficients:**
```cpp
// Characterized brake models from Phase 0 (ground tests, 2026-01-22)
// Model: decel = c0 + c1 * v (m/s²)
struct BrakeModel {
  float c0;  // constant term (m/s²)
  float c1;  // velocity coefficient (1/s)
};

const BrakeModel coast_model = {0.951, 0.794};       // intensity = 0
const BrakeModel pure_brake_model = {1.008, 2.950};  // intensity = 1
```

**Add brake intensity computation:**
```cpp
// Compute brake intensity to achieve desired deceleration at current velocity
// Returns intensity in [0, 1] where 0=coast, 1=pure_brake
// Returns -1 if desired decel is below coast (need forward PWM to slow down)
// Returns >1 if desired decel exceeds pure_brake capability
float compute_brake_intensity(float desired_decel, float v_current) {
  float decel_coast = coast_model.c0 + coast_model.c1 * v_current;
  float decel_brake = pure_brake_model.c0 + pure_brake_model.c1 * v_current;
  
  if (desired_decel < decel_coast) {
    return -1.0;  // Can't decelerate this slowly with braking alone
  }
  if (desired_decel > decel_brake) {
    return 1.0;   // Clamp to max braking (could add reverse torque later)
  }
  
  // Linear interpolation between coast and pure_brake
  return (desired_decel - decel_coast) / (decel_brake - decel_coast);
}
```

---

## 🛑 STOP POINT 2.1

**Before proceeding to 2.2:**
- Review brake model integration design
- User approval for modifying twist control
- Verify model coefficients match latest characterization

---

### 2.2 Modify update_twist_control() for Braking

**Changes to existing twist control:**

```cpp
void update_twist_control() {
  // ... existing ramping and kinematics code ...
  
  // Get actual velocities
  float v_left_actual = left_speedometer.get_velocity();
  float v_right_actual = right_speedometer.get_velocity();
  
  // Determine if we're decelerating
  bool decelerating_left = v_left_target < v_left_actual - velocity_threshold;
  bool decelerating_right = v_right_target < v_right_actual - velocity_threshold;
  
  if (decelerating_left || decelerating_right) {
    // DECELERATION MODE: Use proportional braking
    
    // Compute desired deceleration from target and current velocity
    // If accel_target is specified, use it; otherwise compute from velocity error
    float desired_decel_left = (accel_left < 0) ? -accel_left : 
                               (v_left_actual - v_left_target) / expected_stop_time;
    float desired_decel_right = (accel_right < 0) ? -accel_right :
                                (v_right_actual - v_right_target) / expected_stop_time;
    
    // Compute brake intensities
    float brake_left = compute_brake_intensity(desired_decel_left, v_left_actual);
    float brake_right = compute_brake_intensity(desired_decel_right, v_right_actual);
    
    // Apply braking (clamp to valid range)
    left_motor.brake(clamp(brake_left, 0.0, 1.0));
    right_motor.brake(clamp(brake_right, 0.0, 1.0));
    
  } else {
    // ACCELERATION/MAINTAIN MODE: Use existing PI control with forward PWM
    // ... existing PI controller code ...
    left_motor.go(pwm_left, false);
    right_motor.go(pwm_right, false);
  }
}
```

**Key insight:** When decelerating, we bypass the PI controller entirely and use direct brake commands based on the characterized physics model. The PI controller is only used for acceleration and speed maintenance.

---

## 🛑 STOP POINT 2.2

**Before proceeding to 2.3:**
- Review modified twist control design
- User approval for control logic changes
- Discuss hybrid PI/direct-brake approach
- Plan for testing (low speeds first)

---

### 2.3 Add Position-to-Velocity Trajectory Generator

**For go-to-position tasks (wall approach, go-to-can):**

The position controller computes a velocity profile that arrives exactly at the target with v=0.

**Constant deceleration profile:**
```cpp
// Given: current distance to target, current velocity, desired deceleration
// Compute: target velocity for this instant

float compute_approach_velocity(float distance_to_target, float v_current, float target_decel) {
  // If we started braking now at target_decel, where would we stop?
  // stopping_distance = v² / (2 * decel)
  
  // Invert: what velocity should we be at to stop exactly at distance_to_target?
  // v_target = sqrt(2 * decel * distance_to_target)
  
  float v_target = sqrt(2.0 * target_decel * distance_to_target);
  
  // Clamp to max approach speed
  v_target = min(v_target, max_approach_velocity);
  
  return v_target;
}

// In position control loop:
float distance_to_target = current_tof_distance - target_distance;
float v_target = compute_approach_velocity(distance_to_target, v_current, chosen_decel);
float accel_target = (v_current > v_target) ? -chosen_decel : max_accel;

set_twist_target(v_target, 0.0, accel_target, 0.0);
```

**Choosing deceleration rate:**
- Conservative: Use coast model (~1.5 m/s² at typical speeds)
- Moderate: Use 50% brake (~2.0 m/s²)
- Aggressive: Use pure brake (~3.5 m/s² at high speeds)

Start conservative, can tune up based on test results.

---

## 🛑 MAJOR STOP POINT - END OF PHASE 2

**Before proceeding to Phase 3:**
- Complete code review of all Phase 2 components
- User must approve testing new controller
- Test carefully with low speeds first (0.3 m/s)
- Monitor for unexpected behavior
- Verify deceleration matches model predictions

**Do NOT proceed to wall testing without explicit authorization**

---

## Phase 3: Wall Approach Testing & Optimization

### 3.1 Test Protocol

**Conservative first approach:**
1. Start distance: 1.5 m from wall
2. Target distance: 0.25 m from wall (safe margin)
3. Max approach speed: 0.3 m/s
4. Deceleration: Use coast model only (gentlest)
5. Safety stop: 0.15 m (emergency brake if closer)

**Metrics to collect:**
- Final distance error (target - actual)
- Overshoot (did we go past target?)
- Time to stop
- Velocity profile during deceleration
- Actual vs predicted deceleration

### 3.2 Tuning Strategy

**If stopping too early (undershoot):**
- Deceleration model over-estimates braking force
- Options: Increase chosen_decel, or add small feedforward velocity

**If stopping too late (overshoot):**
- Deceleration model under-estimates braking force
- Options: Decrease chosen_decel, or start braking earlier

**If oscillating:**
- Check for discontinuity between brake and forward modes
- May need hysteresis in mode switching

---

## 🛑 STOP POINT 3.2

**Before proceeding to 3.3:**
- Review test protocol
- User approval for wall approach tests
- Verify safety margins are sufficient

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

**UPDATED - January 22, 2026:**

**⚠️ CRITICAL: Each step requires explicit user authorization before proceeding**

### Completed ✅
1. ✅ **Phase 0.2:** Enhance DRV8833 with proportional brake()
2. ✅ **Phase 0.3:** Create ForceCharacterizationMode
3. ✅ **Phase 0.4:** Add ROS messages for force characterization
4. ✅ **Phase 0.5:** Create analysis script (run_force_characterization.py)
5. ✅ **Phase 0.5:** Run force tests (ground) - Results recorded above
6. ✅ **Phase 0.6:** Build force models (coast, pure_brake, prop_brake)

### Next Steps
7. ⏳ **Phase 2.1:** Add brake models to twist control → 🛑 STOP - Get approval
8. ⏳ **Phase 2.2:** Modify update_twist_control() for braking → 🛑 STOP - Get approval
9. ⏳ **Phase 2.3:** Add position-to-velocity trajectory generator → 🛑 MAJOR STOP - End Phase 2
10. ⏳ **Phase 3.1:** Wall approach testing (conservative) → 🛑 STOP - Review results
11. ⏳ **Phase 3.2:** Tune deceleration parameters → 🛑 STOP - Get approval
12. ⏳ **Phase 4:** Final validation → 🛑 FINAL STOP - Project complete

**Implementation Protocol:**
- One step at a time
- Stop at every 🛑 marker
- Wait for explicit "proceed" authorization
- No coding ahead
- Review at every major phase boundary

---

## Open Questions (ANSWERED)

1. **Braking implementation:** Option A (DRV8833) vs Option B (PIController)?  
   **A:** Use existing DRV8833.brake() function. Twist control decides when to use it.

2. **Deceleration thresholds:** What initial values for threshold1/threshold2?  
   **A:** Don't use thresholds - use characterized physics models to compute brake intensity

3. **Position vs velocity control:** Hard switch or blended transition?  
   **A:** Two-layer: Position controller generates velocity trajectory → Twist control tracks it with model-based braking

4. **TOF sensor:** Which sensor to use? Center only, or average of all three?  
   **A:** Center only

5. **Battery compensation:** Current approach sufficient or needs enhancement?  
   **A:** Brake models are independent of battery (shorting motor windings). Forward drive still uses battery compensation.

6. **How to achieve precise deceleration?** (NEW)  
   **A:** Twist control interpolates between coast (intensity=0) and pure_brake (intensity=1) based on characterized models. Position controller commands constant deceleration trajectory that arrives at target with v=0.

---

## Known Limitations / TODOs

1. **External cmd_vel=0 while moving:** Current twist control only brakes when the position controller explicitly commands negative acceleration. If an external source sends `cmd_vel=0` while the robot is moving fast, the PI controller will try to achieve v=0 via forward PWM reduction only (which may be inadequate at high speeds). 
   - **Workaround:** Position controllers must always compute and command appropriate deceleration
   - **Future fix:** Twist control could infer required deceleration when v_target=0 and v_actual >> 0, even if accel_target not explicitly negative

---

## Risk Mitigation
- **TOF noise:** Add filtering (moving average, Kalman) if readings unstable. 
- **Tread compliance:** Tank treads compress on contact - may need "contact detection" logic
- **Surface variation:** May need surface-specific tuning or adaptive gains
- **Sensor lag:** TOF sensors have ~50ms update rate - account for reaction time in deceleration planning
