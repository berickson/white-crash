# Revised Control Plan

## Background

After much trial and error with different control strategies for the wheels / tracks, this is a revised plan.

The basic issues it to create PID control to track velocity curves for optimal agility. An example is the "go to can" where the robot must go to an exact distance to the can and stop there before initiating the "pick up can" behavior.

Several "regime based" policies were evaluated, and none behaved per expectations. The regimes included "If you are in slowing phase, do this", etc. These caused non-linearities and tuning difficulties.

In a classical PID control problem, you have a mass that you can apply a force to, and you are attempting to stabilize it.

In our motors with h-bridge controllers, you can turn on and off A and B terminals at high speed PWM to drive with fast decay, slow decay, braking, or coasting. There is no direct "Force" input. The idea here is to use our characterized motor control to simulate a force input to a traditional PID system. This force is velocity dependent, so depending on the current wheel velocity, the same input will give different force outputs.  A 50% PWM might lead to a steady state 2 m/s, but if you are currently going 0 m/s, it would be an accelerating force, and if you are going 4 m/s, it would be a decelerating force. There is also braking that has its own curves.  Instead of using force directly, we will use "expected acceleration" as a stand in, where A=f(V, Control Input), we obviously need the inverse of this which is Control Input = f(V,A).

## Strategy

There are two modes. Moving, and stopping as below. Stopping is a very limited mode at the very end of a stop, and while holding a stop.

## Moving
For standard cases, the robot is considered "moving" the setpoint is non-zero or the present value pv is not very close to zero. 

While moving, we have the desired velocity velocity and acceleration SP_V and SP_A.  The desired acceleration is SP_A + K_V * E_V, we can also include an integral term K_I * E_I.

## Stopping
When you are very close to a commanded stop, there should only be braking and coasting commands. You should only be considered as stopping when you are close enough to your goal and already going a very slow speed.

## Constraints

The control will be constrained by maximum acceleration, and maximum deceleration. For a high performance system, these will be tuned around safety and physical limits like wheel slippage limits.

## Unified Control Function (Pseudocode)

This function captures the core idea: take current velocity and desired acceleration, and return a recommended control output based on the learned response models.

```
function control_from_velocity_and_accel(v_current, a_desired):
	# 1) Predict passive decel from coast/friction model
	a_coast = model_coast_decel(v_current)

	# 2) Compute required net actuation to achieve desired accel
	a_needed = a_desired - a_coast

	# 3) If we need more decel than coast provides, use braking
	if a_needed < 0:
		# convert desired decel magnitude to brake intensity
		brake_intensity = model_brake_intensity(v_current, abs(a_needed))
		brake_intensity = clamp(brake_intensity, 0, 1)
		return Control(brake = brake_intensity, pwm = 0)

	# 4) Otherwise, use forward PWM to provide positive accel
	# map required accel to voltage, then to PWM
	voltage_cmd = model_accel_to_voltage(v_current, a_needed)
	pwm_cmd = voltage_cmd / v_bat
	pwm_cmd = clamp(pwm_cmd, -1, 1)
	return Control(brake = 0, pwm = pwm_cmd)
```

## Data Collection Plan for Accel→Voltage Model (Slip-Aware)

Goal: build a learned model $V = f(v, a)$ so `model_accel_to_voltage()` can predict the voltage/PWM needed for a desired acceleration at a given velocity.

**Test types**
1. **Ramp tests** (preferred): command smooth ramps in velocity (e.g., 0→1→2→3 m/s) at multiple ramp rates (0.5, 1.0, 2.0 m/s²).
2. **Step tests**: small velocity steps (e.g., 1.0→1.5→2.0 m/s) to capture transient accel response.

**Data to log**
- Commanded voltage/PWM
- Measured wheel velocity (encoders)
- Derived acceleration (filtered velocity derivative)
- Battery voltage
- IMU accel (filtered) and gyro yaw rate

**Slip detection (gating)**
- Primary: compare encoder-derived accel vs IMU accel over 50–100ms windows (low-pass/median filtered).
- Secondary: compare differential-drive predicted yaw vs gyro yaw.
- If disagreement exceeds a threshold, tag the sample as **slip** and exclude it from model fitting.

**Fitting**
- Fit a simple model first: `V = a0 + a1*v + a2*a`.
- If error remains, expand to include cross terms: `V = a0 + a1*v + a2*a + a3*v*a`.

**Validation**
- Re-run ramps at intermediate rates and verify accel tracking within 5–10% without slip.

## Archive References (Prior Plans)

These plans are archived to avoid confusion but remain useful for historical context and test procedures:
- [archive/speed_control.md](archive/speed_control.md)
- [archive/motor_control.md](archive/motor_control.md)
