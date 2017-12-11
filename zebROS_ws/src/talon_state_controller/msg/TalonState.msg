#s is a message that holds data to describe the state of a set of torque
#controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All
# the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint
# state. 
# The goal is to make each of the fields optional. When e.g. your joints have
# no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct

Header header

string[] name
string[] talon_mode
float64[] position
float64[] speed
float64[] output_voltage
float64[] output_current
float64[] bus_voltage
float64[] motor_output_percentage
float64[] pid_p1
float64[] pid_i1
float64[] pid_d1
float64[] pid_f1
float64[] pid_izone2
float64[] pid_p2
float64[] pid_i2
float64[] pid_d2
float64[] pid_f2
float64[] pid_izone1
float64[] set_point
float64[] VCompensationRampRate
int8[] can_id
int8[] closed_loop_error
int8[] forward_limit_switch
int8[] reverse_limit_switch
bool[] invert
bool[] sensorPhase
