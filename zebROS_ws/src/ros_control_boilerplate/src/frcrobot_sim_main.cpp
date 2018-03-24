/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control main() entry point for controlling robots in ROS
*/

#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <ros_control_boilerplate/frcrobot_sim_interface.h>

#include <ros_control_boilerplate/JoystickState.h>

#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#define KEYCODE_a 0x61
#define KEYCODE_b 0x62
#define KEYCODE_c 0x63
#define KEYCODE_d 0x64
#define KEYCODE_e 0x65
#define KEYCODE_f 0x66
#define KEYCODE_g 0x67
#define KEYCODE_h 0x68
#define KEYCODE_i 0x69
#define KEYCODE_j 0x6a
#define KEYCODE_k 0x6b
#define KEYCODE_l 0x6c
#define KEYCODE_m 0x6d
#define KEYCODE_n 0x6e
#define KEYCODE_o 0x6f
#define KEYCODE_p 0x70
#define KEYCODE_q 0x71
#define KEYCODE_r 0x72
#define KEYCODE_s 0x73
#define KEYCODE_t 0x74
#define KEYCODE_u 0x75
#define KEYCODE_v 0x76
#define KEYCODE_w 0x77
#define KEYCODE_x 0x78
#define KEYCODE_y 0x79
#define KEYCODE_z 0x7a
#define KEYCODE_MINUS 0x2D
#define KEYCODE_EQUALS 0x3D
#define KEYCODE_ONE 0x1
#define KEYCODE_TWO 0x2
#define KEYCODE_LEFT_BRACKET 0x5B
#define KEYCODE_ESCAPE  0x1B

int kfd = 0;
struct termios cooked, raw;

void quit(int /*sig*/)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	exit(0);
}

class TeleopJointsKeyboard
{
	public:

		TeleopJointsKeyboard()
			: has_recieved_joints_(false)
		{
			std::cout << "init " << std::endl;
			// TODO: make this robot agonistic
			joints_pub_ = nh_.advertise<ros_control_boilerplate::JoystickState>("/frcrobot/joystick_states", 1);
		}

		~TeleopJointsKeyboard()
		{ }


		void keyboardLoop()
		{
			char c;

			// get the console in raw mode
			tcgetattr(kfd, &cooked);
			memcpy(&raw, &cooked, sizeof(struct termios));
			raw.c_lflag &= ~ (ICANON | ECHO);
			// Setting a new line, then end of file
			raw.c_cc[VEOL] = 1;
			raw.c_cc[VEOF] = 2;
			tcsetattr(kfd, TCSANOW, &raw);


			for (;;)
			{
				// get the next event from the keyboard
				if (read(kfd, &c, 1) < 0)
				{
					perror("read():");
					exit(-1);
				}

                cmd_.rightStickY = 0;
                cmd_.rightStickX = 0;
                cmd_.leftStickY = 0;
                cmd_.leftStickX = 0;

                cmd_.leftTrigger = 0;
                cmd_.rightTrigger = 0;
                cmd_.buttonXButton = false;
                cmd_.buttonXPress = false;
                cmd_.buttonXRelease = false;
                cmd_.buttonYButton = false;
                cmd_.buttonYPress = false;
                cmd_.buttonYRelease = false;

                cmd_.bumperLeftButton = false;
                cmd_.bumperLeftPress = false;
                cmd_.bumperLeftRelease = false;

                cmd_.bumperRightButton = false;
                cmd_.bumperRightPress = false;
                cmd_.bumperRightRelease = false;

                cmd_.stickLeftButton = false;
                cmd_.stickLeftPress = false;
                cmd_.stickLeftRelease = false;

                cmd_.stickRightButton = false;
                cmd_.stickRightPress = false;
                cmd_.stickRightRelease = false;

                cmd_.buttonAButton = false;
                cmd_.buttonAPress = false;
                cmd_.buttonARelease = false;
                cmd_.buttonBButton = false;
                cmd_.buttonBPress = false;
                cmd_.buttonBRelease = false;
                cmd_.buttonBackButton = false;
                cmd_.buttonBackPress = false;
                cmd_.buttonBackRelease = false;

                cmd_.buttonStartButton = false;
                cmd_.buttonStartPress = false;
                cmd_.buttonStartRelease = false;
				bool dirty = true;
				switch (c)
				{
				case KEYCODE_a:
                    if(cmd_last_.buttonAButton) {
                        cmd_.buttonAPress = false; // radians
                    }
                    else {
                        cmd_.buttonAPress = true; // radians
                    }
					cmd_.buttonAButton = true; // radians
					break;
				case KEYCODE_b:
                    if(cmd_last_.buttonBButton) {
                        cmd_.buttonBPress = false; // radians
                    }
                    else {
                        cmd_.buttonBPress = true; // radians
                    }
					cmd_.buttonBButton = true; // radians
					break;
				case KEYCODE_x:
                    if(cmd_last_.buttonXButton) {
                        cmd_.buttonXPress = false; // radians
                    }
                    else {
                        cmd_.buttonXPress = true; // radians
                    }
					cmd_.buttonXButton = true; // radians
					break;
				case KEYCODE_y:
                    if(cmd_last_.buttonYButton) {
                        cmd_.buttonYPress = false; // radians
                    }
                    else {
                        cmd_.buttonYPress = true; // radians
                    }
					cmd_.buttonYButton = true; // radians
					break;
				case KEYCODE_ONE:
                    if(cmd_last_.buttonBackButton) {
                        cmd_.buttonBackPress = false; // radians
                    }
                    else {
                        cmd_.buttonBackPress = true; // radians
                    }
					cmd_.buttonBackButton = true; // radians
					break;
				case KEYCODE_TWO:
                    if(cmd_last_.buttonStartButton) {
                        cmd_.buttonStartPress = false; // radians
                    }
                    else {
                        cmd_.buttonStartPress = true; // radians
                    }
					cmd_.buttonStartButton = true; // radians
					break;
				case KEYCODE_MINUS:
                    if(cmd_last_.bumperLeftButton) {
                        cmd_.bumperLeftPress = false; // radians
                    }
                    else {
                        cmd_.bumperLeftPress = true; // radians
                    }
					cmd_.bumperLeftButton = true; // radians
					break;
				case KEYCODE_EQUALS:
                    if(cmd_last_.bumperRightButton) {
                        cmd_.bumperRightPress = false; // radians
                    }
                    else {
                        cmd_.bumperRightPress = true; // radians
                    }
					cmd_.bumperRightButton = true; // radians
					break;

				case KEYCODE_LEFT_BRACKET:

                    if (read(kfd, &c, 1) < 0)
                    {
                        perror("read():");
                        exit(-1);
                    }
                    switch (c)
                    {
                        case KEYCODE_b:
                            if(cmd_last_.stickLeftButton) {
                                cmd_.stickLeftPress = false; // radians
                            }
                            else {
                                cmd_.stickLeftPress = true; // radians
                            }
                            cmd_.stickLeftButton = true;
                            break;
                        case KEYCODE_a:
                            if(cmd_last_.stickRightButton) {
                                cmd_.stickRightPress = false; // radians
                            }
                            else {
                                cmd_.stickRightPress = true; // radians
                            }
                            cmd_.stickRightButton = true;
                            break;
                        case KEYCODE_d:
                            cmd_.leftTrigger = .5;
                            break;
                        case KEYCODE_c:
                            cmd_.rightTrigger = .5;
                            break;
            
                    }
					break;

				case  KEYCODE_ESCAPE:
					std::cout << std::endl;
					std::cout << "Exiting " << std::endl;
					quit(0);
					break;
				default:
                    ROS_WARN("FaileD");
					std::cout << "CODE: "  << c << std::endl;
					dirty = false;
				}
                
                if(cmd_last_.buttonAButton && !cmd_.buttonAButton) {
                    cmd_.buttonARelease = true;
                }
                if(cmd_last_.buttonBButton && !cmd_.buttonBButton) {
                    cmd_.buttonBRelease = true;
                }
                if(cmd_last_.buttonXButton && !cmd_.buttonXButton) {
                    cmd_.buttonXRelease = true;
                }
                if(cmd_last_.buttonYButton && !cmd_.buttonYButton) {
                    cmd_.buttonYRelease = true;
                }
                if(cmd_last_.buttonStartButton && !cmd_.buttonStartButton) {
                    cmd_.buttonARelease = true;
                }
                if(cmd_last_.buttonBackButton && !cmd_.buttonBackButton) {
                    cmd_.buttonARelease = true;
                }
                if(cmd_last_.stickLeftButton && !cmd_.stickLeftButton) {
                    cmd_.stickLeftRelease = true;
                }
                if(cmd_last_.stickRightButton && !cmd_.stickRightButton) {
                    cmd_.stickRightRelease = true;
                }
                if(cmd_last_.bumperLeftButton && !cmd_.bumperLeftButton) {
                    cmd_.bumperLeftRelease = true;
                }
                if(cmd_last_.bumperRightButton && !cmd_.bumperRightButton) {
                    cmd_.bumperRightRelease = true;
                }
				// Publish command
				if (dirty)
				{
					// Important safety feature
					//if (!has_recieved_joints_)
					//{
					//	ROS_ERROR_STREAM_NAMED("joint_teleop", "Unable to send joint commands because robot state is invalid");
					//}
					//else
					//{
						std::cout << ".";
						joints_pub_.publish(cmd_);
                        cmd_last_ = cmd_;
					//}
				}
			}
		}

	private:

		ros::NodeHandle nh_;
		ros::Publisher joints_pub_;
		ros::Subscriber joints_sub_;
		ros_control_boilerplate::JoystickState cmd_;
		ros_control_boilerplate::JoystickState cmd_last_;
		bool has_recieved_joints_;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "frcrobot_hw_interface");
	ros::NodeHandle nh;

	// NOTE: We run the ROS loop in a separate thread as external calls such
	// as service callbacks to load controllers can block the (main) control loop
	ros::AsyncSpinner spinner(2);
	spinner.start();
	signal(SIGINT, quit);

	TeleopJointsKeyboard teleop;
	teleop.keyboardLoop();

	// Create the hardware interface specific to your robot
	boost::shared_ptr<frcrobot_control::FRCRobotSimInterface> frcrobot_sim_interface
	(new frcrobot_control::FRCRobotSimInterface(nh));
	frcrobot_sim_interface->init();

	// Start the control loop
	ros_control_boilerplate::GenericHWControlLoop control_loop(nh, frcrobot_sim_interface);

	control_loop.run(); // Blocks until shutdown signal recieved 

	return 0;
}
