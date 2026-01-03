# ROS2 Parameter Tuning

## Goal
Enable real-time tuning of speed control gains without reflashing firmware.

## Parameters to Expose

### Velocity Control (per wheel)
- `left_wheel.k_p` - Proportional gain (V/(m/s))
- `left_wheel.k_i` - Integral gain (V/(m/s²))
- `left_wheel.ff_accel` - Acceleration feedforward (V/(m/s²))
- `left_wheel.ff_gain` - Velocity feedforward gain (V/(m/s))
- `left_wheel.ff_offset` - Velocity feedforward offset (V)
- Same for `right_wheel.*`

### System Limits
- `max_linear_accel` - Max linear acceleration (m/s²)
- `max_angular_accel` - Max angular acceleration (rad/s²)

### Angular Control
- `angle_k_p` - Angular velocity P gain
- `angle_k_i` - Angular velocity I gain

## Implementation Plan

1. **Add ROS2 parameter server to node** (micro-ros-arduino supports this)
2. **Declare parameters with defaults** from current hardcoded values
3. **Add parameter update callback** to write new values to controller objects
4. **Expose getter/setter methods** on PIController class
5. **Test with:** `ros2 param set /white_crash left_wheel.k_p 2.5`

## Usage
```bash
# List all parameters
ros2 param list /white_crash

# Get current value
ros2 param get /white_crash left_wheel.k_p

# Set new value (live tuning!)
ros2 param set /white_crash left_wheel.k_p 2.5

# Save tuned values
ros2 param dump /white_crash > tuned_params.yaml
ros2 run white_crash white_crash --ros-args --params-file tuned_params.yaml
```

## Benefits
- **No reflashing** - tune gains in real-time
- **Quick iteration** - test multiple values rapidly
- **Save configurations** - dump working parameters to file
- **A/B testing** - compare different gain sets easily
