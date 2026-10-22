# Motion errors: joint/angular in radians, linear in metres.
# Per-task activation/reference arrays follow the task dimension; summary messages leave them empty.
std_msgs/Header header
string action
float64 joint_error
float64 linear_error
float64 angular_error
bool joint_limit_active
string id
bool enabled
bool in_current_action
float64[] internal_activation_function
float64[] external_activation_function
float64[] reference_rate
