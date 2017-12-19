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
float32[] position
float32[] speed
float32[] output_voltage
float32[] output_current
float32[] bus_voltage
float32[] motor_output_percent
float32[] pid_p0
float32[] pid_p1
float32[] pid_i0
float32[] pid_i1
float32[] pid_d0
float32[] pid_d1
float32[] pid_f0
float32[] pid_f1
int32[] pid_izone0
int32[] pid_izone1
int32[] pid_allowable_closed_loop_error0
int32[] pid_allowable_closed_loop_error1
float32[] pid_max_integral_accumulator0
float32[] pid_max_integral_accumulator1
float32[] set_point
float32[] VCompensationRampRate
int8[] can_id
int32[] closed_loop_error
float32[] integral_accumulator
float32[] error_derivative
int8[] forward_limit_switch
int8[] reverse_limit_switch
bool[] invert
bool[] sensorPhase
string[] neutral_mode
bool[] neutral_output
