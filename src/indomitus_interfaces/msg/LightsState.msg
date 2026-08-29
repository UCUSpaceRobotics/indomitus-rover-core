# Every light the rover can switch, as lights_can_node last confirmed it over
# CAN. Published latched on lights/state so that both operators — the onboard
# joystick and the ground station — read the same state instead of each keeping
# a copy that drifts as soon as the other one touches a light.
bool spotlight
bool beautiful
bool traffic_red
bool traffic_yellow
bool traffic_green
bool traffic_blue
