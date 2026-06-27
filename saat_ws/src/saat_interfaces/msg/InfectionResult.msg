# Published by: A1_vision … B3_vision
# Subscribed by: Ax_action, Ax_data_collector, infection_description

std_msgs/Header header

string  zone_id            # e.g. "A1", "B3"
bool    pear_detected      # True if a pear was found in this zone
bool    is_infected        # True if infection_ratio > threshold

float32 infection_ratio    # infection_area / pear_area  (0.0 – 1.0)
float32 infection_area_px  # infected pixel count
float32 pear_area_px       # total pear pixel count

float32 infection_x        # centroid of infection in sub-frame pixels
float32 infection_y

uint8   infection_r        # RGB of dominant infection colour
uint8   infection_g
uint8   infection_b

float32 pear_centroid_x    # centroid of pear (for speed node)
float32 pear_centroid_y
