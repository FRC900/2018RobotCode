


void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &msg);
void evaluateState(const teleop_joystick_control::RobotState::ConstPtr &msg);
void rumbleTypeConverterPublish(uint16_t leftRumble, uint16_t rightRumble);
void evaluateTime(const ros_control_boilerplate::MatchSpecificData::ConstPtr &msg);

