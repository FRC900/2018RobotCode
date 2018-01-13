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
float64[] motor_output_percent
float64[] temperature
string[]  feedback_sensor
int32[]   encoder_ticks_per_rotation
int32[]   pid_slot
float64[] pid_p0
float64[] pid_p1
float64[] pid_i0
float64[] pid_i1
float64[] pid_d0
float64[] pid_d1
float64[] pid_f0
float64[] pid_f1
int32[] pid_izone0
int32[] pid_izone1
int32[] pid_allowable_closed_loop_error0
int32[] pid_allowable_closed_loop_error1
float64[] pid_max_integral_accumulator0
float64[] pid_max_integral_accumulator1
float64[] set_point
int8[] can_id
float64[] closed_loop_error
float64[] integral_accumulator
float64[] error_derivative
float64[] closed_loop_target
float64[] active_trajectory_position
float64[] active_trajectory_velocity
float64[] active_trajectory_heading
int8[] forward_limit_switch
int8[] reverse_limit_switch
bool[] invert
bool[] sensorPhase
string[] neutral_mode
bool[] neutral_output
float64[] closed_loop_ramp
float64[] open_loop_ramp
float64[] peak_output_forward
float64[] peak_output_reverse
float64[] nominal_output_forward
float64[] nominal_output_reverse
float64[] neutral_deadband
float64[] voltage_compensation_saturation
int32[]   voltage_measurement_filter
bool[]    voltage_compensation_enable
string[]  limit_switch_local_forward_source
string[]  limit_switch_local_forward_normal
string[]  limit_switch_local_reverse_source
string[]  limit_switch_local_reverse_normal
float64[] softlimit_forward_threshold
bool[]    softlimit_forward_enable
float64[] softlimit_reverse_threshold
bool[]    softlimit_reverse_enable
bool[]    softlimits_override_enable
int32[]   current_limit_peak_amps
int32[]   current_limit_peak_msec
int32[]   current_limit_continuous_amps
bool[]    current_limit_enable
float64[] motion_cruise_velocity
float64[] motion_acceleration
int32[]   motion_profile_top_level_buffer_count
bool[]    motion_profile_top_level_buffer_full
int32[]   motion_profile_status_top_buffer_rem
int32[]   motion_profile_status_top_buffer_cnt
int32[]   motion_profile_status_btm_buffer_cnt
bool[]    motion_profile_status_has_underrun
bool[]    motion_profile_status_is_underrun
bool[]    motion_profile_status_active_point_valid
bool[]    motion_profile_status_is_last
int32[]   motion_profile_status_profile_slot_select
string[]  motion_profile_status_output_enable
