# Published by: main_speed  →  speed_publisher
# Also published by speed_publisher → /speed_to_plc

std_msgs/Header header

float32 reference_speed_ms   # computed reference speed  (m/s)
float32 conv1_voltage        # target voltage Conv1  (0.1 – 3.3 V)
float32 conv2_voltage        # target voltage Conv2  (0.1 – 3.3 V)
float32 servo_voltage        # target voltage servo  (0.1 – 3.3 V)

string  belt_state           # "EMPTY" | "NORMAL" | "CROWDED"
int32   pear_count           # current pear count in vision zone
