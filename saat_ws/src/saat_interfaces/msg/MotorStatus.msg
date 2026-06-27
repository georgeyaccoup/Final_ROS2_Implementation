# Published by: data_collection_node
# Subscribed by: webpage_publisher_status, IoT system

std_msgs/Header header

# One entry per zone (A1, A2, A3, B1, B2, B3)
string[6]  zone_ids          # ["A1","A2","A3","B1","B2","B3"]
bool[6]    servo_active      # True if servo fired this cycle
string[6]  last_action       # "ACCEPTED" | "REJECTED" | "IDLE"

float32    conv1_speed_ms    # actual Conv1 belt speed (m/s)
float32    conv2_speed_ms    # actual Conv2 belt speed (m/s)
float32    conv1_voltage     # current PWM voltage Conv1
float32    conv2_voltage     # current PWM voltage Conv2

int32      batch_accepted
int32      batch_rejected
int32      completed_packages
