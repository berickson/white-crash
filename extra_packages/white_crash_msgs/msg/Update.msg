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
