# Every light the rover can switch, as lights_can_node last confirmed it over
# CAN. Published latched on lights/state so that both operators — the onboard
# joystick and the ground station — read the same state instead of each keeping
# a copy that drifts as soon as the other one touches a light.
bool spotlight_left
bool spotlight_right
bool beautiful
bool beautiful_1
bool beautiful_2
bool beautiful_3
bool beautiful_4
bool traffic_red
bool traffic_green
bool traffic_blue
bool buzzer
