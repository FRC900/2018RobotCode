#include <ros/ros.h>
#include <ros_control_boilerplate/JoystickState.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_js_pub");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<ros_control_boilerplate::JoystickState>("joystick_states", 50);

	ros::Rate rate(25);
	while (ros::ok())
	{
		ros_control_boilerplate::JoystickState js;
		js.header.stamp = ros::Time::now();

		js.rightStickY = 1.0;
		js.rightStickX = 0;
		js.leftStickY = 2.0;
		js.leftStickX = 0;

		js.leftTrigger = 1;
		js.rightTrigger = 0;

		js.buttonAButton = true;
		js.buttonAPress = false;
		js.buttonARelease = false;

		js.buttonBButton = false;
		js.buttonBPress = false;
		js.buttonBRelease = false;

		js.buttonXButton = false;
		js.buttonXPress = true;
		js.buttonXRelease = false;

		js.buttonYButton = false;
		js.buttonYPress = false;
		js.buttonYRelease = false;

		js.bumperLeftButton = false;
		js.bumperLeftPress = false;
		js.bumperLeftRelease = false;

		js.bumperRightButton = false;
		js.bumperRightPress = false;
		js.bumperRightRelease = false;

		js.buttonBackButton = false;
		js.buttonBackPress = false;
		js.buttonBackRelease = false;

		js.buttonStartButton = false;
		js.buttonStartPress = false;
		js.buttonStartRelease = false;

		js.stickLeftButton = false;
		js.stickLeftPress = false;
		js.stickLeftRelease = false;

		js.stickRightButton = false;
		js.stickRightPress = false;
		js.stickRightRelease = false;

		js.directionUpButton = false;
		js.directionUpPress = false;
		js.directionUpRelease = false;

		js.directionDownButton = false;
		js.directionDownPress = false;
		js.directionDownRelease = false;


		js.directionLeftButton = false;
		js.directionLeftPress = false;
		js.directionLeftRelease = false;


		js.directionRightButton = false;
		js.directionRightPress = false;
		js.directionRightRelease = false;

		pub.publish(js);

		rate.sleep();
	}
}
