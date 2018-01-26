#include <cmath>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>


void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &msg);
void evaluateState(const teleop_joystick_control::RobotState::ConstPtr &msg);
void rumbleTypeConverterPublish(uint16_t leftRumble, uint16_t rightRumble);
void evaluateTime(const ros_control_boilerplate::MatchSpecificData::ConstPtr &msg);
double navX_angle_ = M_PI/2;
int navX_index_ = -1;
ros::Subscriber navX_heading_;
void navXCallback(const sensor_msgs::JointState &navXState);
