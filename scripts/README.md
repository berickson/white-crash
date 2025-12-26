# Motor Characterization Scripts

## characterize_motors.py

Automates Step 1 of the speed control implementation: open-loop characterization.

### What it does:
1. Triggers the robot's open-loop characterization mode
2. Collects voltage and velocity data from both motors
3. Fits a feedforward model: `voltage = offset + gain * velocity`
4. Calculates recommended initial PI gains
5. Generates plots and saves results

### Prerequisites:
- ROS 2 Jazzy environment
- Robot powered on and in hand mode
- Robot on racks with wheels off the ground
- micro-ROS agent running: 
  ```bash
  docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v3
  ```

### Usage:
```bash
# From a Jazzy terminal with ROS sourced
cd ~/projects/white-crash/scripts
python3 characterize_motors.py
```

### Dependencies:
```bash
pip install numpy scipy matplotlib
```

### Output:
- `motor_characterization_results.txt` - Text summary of results
- `motor_characterization_plot.png` - Plots showing fitted models
- Terminal output with feedforward parameters and recommended PI gains

### Expected Results:
- Feedforward model for each motor (voltage-to-velocity relationship)
- Recommended P gain: ~1-2 V/(m/s)
- Recommended I gain: ~0.5-1.0 V/(m/s²)
- D gain: 0 (not needed for velocity control)

### Troubleshooting:
- If no data collected: Verify `/white_crash/update` topic is publishing
- If fits look bad: Check that robot wheels were free to spin
- If voltages seem wrong: Check battery is fully charged

---

## monitor_pi_control.py

Monitors and validates Step 2 of the speed control implementation: PI controller with feedforward.

### What it does:
1. Triggers the robot's PI control test mode
2. Monitors commanded vs actual velocities in real-time
3. Calculates tracking errors and performance statistics
4. Generates plots showing velocity tracking, PWM commands, and battery voltage

### Prerequisites:
- ROS 2 Jazzy environment
- Robot powered on and in hand mode
- Robot on racks (recommended) or in safe testing area
- micro-ROS agent running:
  ```bash
  docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v3
  ```

### Usage:
```bash
# Terminal output only
cd ~/projects/white-crash/scripts
python3 monitor_pi_control.py

# With plots (requires matplotlib)
python3 monitor_pi_control.py --plot

# Custom duration
python3 monitor_pi_control.py --plot --duration 60
```

### Dependencies:
```bash
# Required (ROS packages)
# Should already be available in ROS 2 Jazzy environment

# Optional (for plotting)
pip install matplotlib
```

### Output:
- Real-time terminal output showing:
  - Timestamp
  - Left/right target velocities
  - Left/right actual velocities  
  - Tracking errors (absolute and percentage)
  - Battery voltage
- Final error statistics (mean and max errors)
- Plot saved to `scripts/pi_control_test_<timestamp>.png` (if --plot used)

### Test Sequence:
The PI control test mode runs through:
1. Stopped (2s)
2. 0.5 m/s forward (5s)
3. 1.0 m/s forward (5s)
4. 0.5 m/s forward (3s) - deceleration
5. Stopped (2s)
6. -0.5 m/s reverse (3s)
7. Rotate left in place, ω=0.5 rad/s (3s)
8. Rotate right in place, ω=-0.5 rad/s (3s)

Total duration: ~35 seconds

### Expected Results:
- Velocity tracking errors < 10% in steady-state
- Minimal overshoot during velocity changes
- Feedforward should handle most of the control effort
- PI correction should be small (verify by examining voltage commands)

### Troubleshooting:
- If test doesn't start: Make sure robot is in hand mode, press SC button on transmitter
- If poor tracking: May need to adjust P/I gains or check motor calibration
- If oscillations: Reduce P gain
- If steady-state error: Increase I gain
- If no data: Verify `/white_crash/update` topic is publishing

---