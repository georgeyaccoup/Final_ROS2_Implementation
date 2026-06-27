# Published by: A1_data_collector … B3_data_collector
# Subscribed by: data_collection_node

std_msgs/Header header

string  pear_id            # e.g. "A1_00001"
string  zone_id            # e.g. "A1"
string  pear_status        # "ACCEPTED" | "REJECTED"
string  pear_category      # "SMALL" | "BIG"

float32 infection_area_px
float32 infection_x
float32 infection_y
uint8   infection_r
uint8   infection_g
uint8   infection_b
float32 infection_ratio

float32 pear_surface_area_px   # 2-D silhouette area in pixels
float32 pear_volume_cm3        # estimated from depth
float32 pear_mass_g            # estimated from volume × density
