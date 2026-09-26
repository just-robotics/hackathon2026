uint8 WAIT=0
uint8 STOP=1
uint8 GOAL=2
uint8 EXPLORE=3
uint8 EVADE=4
uint8 SEARCH=5
uint8 PURSUE=6
uint8 CAPTURE=7

std_msgs/Header header
uint8 behavior
geometry_msgs/Pose target
bool has_target
float32 target_tolerance
float32 max_speed
float32 opponent_clearance
float32 opponent_cost_weight
string reason
