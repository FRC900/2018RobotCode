//this file exists so src folder is uploaded for structure
#include "elevator_node/linear_control.h"

bool ifCube;
double elevatorHeight;
double pivotAngle;
bool clampState;
ros::Publisher RobotStatePub;
elevator_teleop_control::RobotState RobotStateMsg;

using namespace std;

void elevatorHeightControl( ){
     //TODO : Set the elevator height to which specified by the joystick control node    
}

void clampStateControl(){
     //TODO : Control the pneumatics to open/close clamp
}

void pivotAngleControl(){
     //TODO : set the pivot angle to which specified by the joystick control node
}
 
void evaluateCubeState(){
     //TODO : get state of linebreak and publish cube holding state
}

void main(int argc, int *argv){
 /* ros::init(argc, argv, "elevator_control");
  ros::NodeHandle n;
  RobotStatePub = n.advertise<elevator_teleop_control::RobotState>("RobotState", 1);
  ros::Rate loop_rate(10);
  RobotStatePub.publish(RobotStateMsg);
  ros::spin();*/
}
