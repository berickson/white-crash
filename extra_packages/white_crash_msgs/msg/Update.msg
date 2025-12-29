float32 battery_voltage      # measured main battery voltage
float32 left_speed           # current left speed in m/s
float32 right_speed          # current right speed in m/s
float32 left_motor_command   # PWM command sent to left motor -1 to 1 
float32 right_motor_command  # PWM command sent to right motor -1 to 1
int32   left_odometer_ticks  # odometer ticks for left motor
int32   right_odometer_ticks # odometer ticks for right motor
float32 rx_esc               # throttle input -1 to 1
float32 rx_str               # str input -1 to 1
float32 yaw_degrees
int16   mag_x
int16   mag_y
int16   mag_z
float32 bno_yaw_degrees
float32 bno_pitch_degrees
float32 bno_roll_degrees
float32 bno_acceleration_x
float32 bno_acceleration_y
float32 bno_acceleration_z
float32 bno_gyro_x
float32 bno_gyro_y
float32 bno_gyro_z
float32 bno_mag_x
float32 bno_mag_y
float32 bno_mag_z
float32 twist_target_linear         # Commanded linear velocity (m/s) - NAN if not in twist control
float32 twist_target_angular        # Commanded angular velocity (rad/s) - NAN if not in twist control
float32 twist_target_accel_linear   # Commanded linear acceleration (m/s²) - NAN if not in twist control
float32 twist_target_accel_angular  # Commanded angular acceleration (rad/s²) - NAN if not in twist control
float32 v_left_target               # Commanded left wheel velocity (m/s) - NAN if not in twist control
float32 v_right_target              # Commanded right wheel velocity (m/s) - NAN if not in twist control
