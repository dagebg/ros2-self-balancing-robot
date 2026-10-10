# State of the balance controller on the ESP32, published by robot_bridge at the telemetry rate.

std_msgs/Header header

# Safety state machine (see docs: Balance control > Safety).
uint8 STATE_DISARMED=0
uint8 STATE_ARMED=1
uint8 STATE_BALANCING=2
uint8 STATE_FAULT=3
uint8 state

# Bitwise OR of the FAULT_* flags.
uint16 FAULT_TILT_LIMIT=1
uint16 FAULT_IMU_FAILURE=2
uint16 FAULT_LINK_TIMEOUT=4
uint16 FAULT_IMU_DISAGREEMENT=8
uint16 FAULT_LOOP_OVERRUN=16
uint16 faults

float32 roll          # [rad]
float32 pitch         # [rad], positive = nose down (REP-103)
float32 pitch_rate    # [rad/s]
float32 yaw_rate      # [rad/s]
float32 left_command  # motor command in [-1, 1]
float32 right_command # motor command in [-1, 1]
uint16 max_loop_time_us  # longest control-loop iteration since the previous message
