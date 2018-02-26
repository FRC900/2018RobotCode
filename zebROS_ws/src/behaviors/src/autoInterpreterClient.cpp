#include "ros/ros.h"
#include "ros_control_boilerplate/AutoMode.h"
#include "ros_control_boilerplate/MatchSpecificData.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "behaviors/IntakeLiftAction.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/Intake.h"
#include "elevator_controller/ElevatorControlS.h"
#include "elevator_controller/bool_srv.h"
#include "cstdlib"
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <talon_swerve_drive_controller/GenerateTrajectory.h>
#include <talon_swerve_drive_controller/MotionProfilePoints.h>
#include <talon_swerve_drive_controller/FullGen.h>
#include <talon_swerve_drive_controller/FullGenCoefs.h>
#include <talon_swerve_drive_controller/Coefs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <XmlRpcValue.h>
#include <vector>

ros::ServiceClient point_gen;
ros::ServiceClient swerve_control;


static int start_pos = -1;
std::vector<int> auto_mode_vect = {-1, -1, -1, -1};
static int auto_mode = -1;
static double start_time;

//static ros::Publisher IntakeService;
static ros::ServiceClient IntakeService;
static ros::ServiceClient ElevatorService;
static ros::ServiceClient ClampService;
static ros::Publisher VelPub;

static double high_scale_config_x;
static double high_scale_config_y;
static double mid_scale_config_x;
static double mid_scale_config_y;
static double low_scale_config_x;
static double low_scale_config_y;
static double switch_config_x;
static double switch_config_y;
static double exchange_config_x;
static double exchange_config_y;
static double intake_config_x;
static double intake_config_y;
static double default_x;
static double default_y;
static double timeout;
static double autoStart;
static int layout;
static std::vector<std::vector<double>> vectTimes;
static XmlRpc::XmlRpcValue modes;
static std::vector<trajectory_msgs::JointTrajectory> trajectories;

std::vector<talon_swerve_drive_controller::FullGenCoefs> coefs_vect;
//coefs_vect.reserve(4);


bool defaultConfig(elevator_controller::ElevatorControlS srv) {
    srv.request.x = default_x;
    srv.request.y = default_y;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    return ElevatorService.call(srv);
}
bool intakeConfig(elevator_controller::ElevatorControlS srv) {
    srv.request.x = intake_config_x;
    srv.request.y = intake_config_y;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    return ElevatorService.call(srv);
}
bool switchConfig(elevator_controller::ElevatorControlS srv) {
    srv.request.x = switch_config_x;
    srv.request.y = switch_config_y;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    return ElevatorService.call(srv);
}
bool highScale(elevator_controller::ElevatorControlS srv) {
    srv.request.x = high_scale_config_x;
    srv.request.y = high_scale_config_y;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    return ElevatorService.call(srv);
}
bool midScale(elevator_controller::ElevatorControlS srv) {
    srv.request.x = mid_scale_config_x;
    srv.request.y = mid_scale_config_y;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    return ElevatorService.call(srv);
}
bool lowScale(elevator_controller::ElevatorControlS srv) {
    srv.request.x = low_scale_config_x;
    srv.request.y = low_scale_config_y;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    return ElevatorService.call(srv);
}
bool stopIntake(elevator_controller::Intake srv) {
    srv.request.power=0;
    return IntakeService.call(srv);
}
bool releaseClamp(elevator_controller::bool_srv srv) {
    srv.request.data = false;
    return ClampService.call(srv);
}
bool clamp(elevator_controller::bool_srv srv) {
    srv.request.data = true;
    return ClampService.call(srv);
}
bool releaseIntake(elevator_controller::Intake srv) {
    srv.request.spring_state = 1;
    IntakeService.call(srv);
}

void generateTrajectory(int layout, int auto_mode, int start_pos) {

    XmlRpc::XmlRpcValue &path = modes[auto_mode][layout][start_pos];
    XmlRpc::XmlRpcValue &xml_x = path["x"];
    //const int num_splines = xml_x.size();
    talon_swerve_drive_controller::FullGenCoefs srv;
    for(int num = 0; num<xml_x.size(); num++) {
        XmlRpc::XmlRpcValue &x = path["x"];
        XmlRpc::XmlRpcValue &x_num = x[num];
        XmlRpc::XmlRpcValue &y = path["y"];
        XmlRpc::XmlRpcValue &y_num = y[num];
        XmlRpc::XmlRpcValue &orient = path["orient"];
        XmlRpc::XmlRpcValue &orient_num = orient[num];

        talon_swerve_drive_controller::Coefs x_coefs;
        talon_swerve_drive_controller::Coefs y_coefs;
        talon_swerve_drive_controller::Coefs orient_coefs;
        for(int i = 0; i<x_num.size(); i++) {
            const double x_coef = x_num[i];
            const double y_coef = y_num[i];
            const double orient_coef = orient_num[i];

            ROS_WARN("%f", orient_coef);
            x_coefs.spline.push_back(x_coef);
            y_coefs.spline.push_back(y_coef);
            orient_coefs.spline.push_back(orient_coef);
        }
        /*
        if(coefs_vect[layout].request.x_coefs.size()!=0) {
            coefs_vect[layout].request.x_coefs.clear();
            coefs_vect[layout].request.y_coefs.clear();
            coefs_vect[layout].request.orient_coefs.clear();
            coefs_vect[layout].request.end_points.clear();
        }*/
        srv.request.x_coefs.push_back(x_coefs);
        srv.request.y_coefs.push_back(y_coefs);
        srv.request.orient_coefs.push_back(orient_coefs);
        srv.request.end_points.push_back(layout+1);
    }
    srv.request.initial_v = 0;
    srv.request.final_v = 0;
    coefs_vect.push_back(srv);
    point_gen.call(coefs_vect[layout]);
}

std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>> ac;
void auto_modes(const ros_control_boilerplate::AutoMode::ConstPtr & AutoMode, const ros_control_boilerplate::MatchSpecificData::ConstPtr& MatchData) {
    if(AutoMode->position != start_pos) {
        for(int i = 0; i<4; i++) {
            generateTrajectory(i, AutoMode->mode[i], AutoMode->position);
        }
    }
    else {
        for(int i = 0; i<4; i++) {
            if(AutoMode->mode[i] != auto_mode_vect[i]) {
                generateTrajectory(i, AutoMode->mode[i], AutoMode->position);
            }
            auto_mode_vect[i] = AutoMode->mode[i];
        }
    }
    start_pos = AutoMode->position;
    /*
    startPos = AutoMode->position;

    vectTimes.resize(4);
    trajectories.resize(4);
    //for(int layout = 0; layout<2; layout++) { 
      //  for(int auto_mode = 0; auto_mode<2; auto_mode++) {
    for(int mode = 0; mode < 4; mode++) {

        //std::map<std::string, XmlRpc::XmlRpcValue> spline = modes[auto_mode][layout][startPos];
        //std::map<std::string, XmlRpc::XmlRpcValue> spline = modes["0"]["0"]["0"];
        XmlRpc::XmlRpcValue &splines = modes[auto_mode][layout][startPos];
        //std::map<std::string, XmlRpc::XmlRpcValue> spline = xml_times;
        //trajectory_msgs::JointTrajectory trajectory;
        trajectories[mode].joint_names.push_back("x_linear_joint");
        trajectories[mode].joint_names.push_back("y_linear_joint");
        trajectories[mode].joint_names.push_back("z_rotation_joint");
        const size_t num_joints = trajectories[auto_mode+2*layout].joint_names.size();
        trajectories[mode].points.resize(3);
        XmlRpc::XmlRpcValue &timesVect = splines["times"];
        XmlRpc::XmlRpcValue &positionsXVect = splines["positionsX"];
        XmlRpc::XmlRpcValue &positionsYVect = splines["positionsY"];
        XmlRpc::XmlRpcValue &positionsZVect = splines["positionsZ"];

        XmlRpc::XmlRpcValue &velocitiesXVect = splines["velocitiesX"];
        XmlRpc::XmlRpcValue &velocitiesYVect = splines["velocitiesY"];
        XmlRpc::XmlRpcValue &velocitiesZVect = splines["velocitiesZ"];

        XmlRpc::XmlRpcValue &accelerationsXVect = splines["accelerationsX"];
        XmlRpc::XmlRpcValue &accelerationsYVect = splines["accelerationsY"];
        XmlRpc::XmlRpcValue &accelerationsZVect = splines["accelerationsZ"];

        vectTimes[mode].resize(timesVect.size());

        for(int i = 0; i<timesVect.size(); i++) { 
            XmlRpc::XmlRpcValue &xml_time_i = timesVect[i];
            const double  time_i = xml_time_i;
            //ROS_ERROR("Time: %d, %f, %lf", time_i, time_i, time_i); 
            vectTimes[mode].push_back(timesVect[i]);
            //vectTimes[auto_mode+layout].push_back(0.0);
            ROS_INFO("[mode: %d][layout: %d][pos: %d][i: %d]: %lf", auto_mode, layout, startPos, i, vectTimes[auto_mode+2*layout][i]);

            trajectories[mode].points[i].positions.resize(num_joints);
            trajectories[mode].points[i].positions[0] =  positionsXVect[i];
            trajectories[mode].points[i].positions[1] =  positionsYVect[i];
            trajectories[mode].points[i].positions[2] =  positionsZVect[i];
            ROS_INFO("positionsX[%d,%d]: %f", auto_mode, i, trajectories[mode].points[i].positions[0]);
            ROS_INFO("positionsY[%d,%d]: %f", auto_mode, i, trajectories[mode].points[i].positions[1]);
            ROS_INFO("positionsZ[%d,%d]: %f", auto_mode, i, trajectories[mode].points[i].positions[2]);

            trajectories[mode].points[i].velocities.resize(num_joints);
            trajectories[mode].points[i].velocities[0] =  velocitiesXVect[i];
            trajectories[mode].points[i].velocities[1] =  velocitiesYVect[i];
            trajectories[mode].points[i].velocities[2] =  velocitiesZVect[i];
            ROS_INFO("velocitiesX[%d,%d]: %f", auto_mode, i, trajectories[mode].points[i].velocities[0]);
            ROS_INFO("velocitiesY[%d,%d]: %f", auto_mode, i, trajectories[mode].points[i].velocities[1]);
            ROS_INFO("velocitiesZ[%d,%d]: %f", auto_mode, i, trajectories[mode].points[i].velocities[2]);

            trajectories[mode].points[i].accelerations.resize(num_joints);
            trajectories[mode].points[i].accelerations[0] =  accelerationsXVect[i];
            trajectories[mode].points[i].accelerations[1] =  accelerationsYVect[i];
            trajectories[mode].points[i].accelerations[2] =  accelerationsZVect[i];
            ROS_INFO("accelerationsX[%d,%d]: %f", auto_mode, i, trajectories[mode].points[i].accelerations[0]);
            ROS_INFO("accelerationsY[%d,%d]: %f", auto_mode, i, trajectories[mode].points[i].accelerations[1]);
            ROS_INFO("accelerationsZ[%d,%d]: %f", auto_mode, i, trajectories[mode].points[i].accelerations[2]);

            trajectories[auto_mode+2*layout].points[i].time_from_start = ros::Duration(2*i+1);
            talon_swerve_drive_controller::FullGen srv;
            srv.request.joint_trajectory = trajectories[auto_mode+2*layout];
            srv.request.initial_v = 0.0;
            srv.request.final_v = 0.0;
            point_gen.call(srv);
            talon_swerve_drive_controller::MotionProfilePoints srv_msg_points;

            srv_msg_points.request.dt = srv.response.dt;	
            srv_msg_points.request.points = srv.response.points;	
            srv_msg_points.request.buffer = true;	
            srv_msg_points.request.mode = false;
            srv_msg_points.request.run = false;


            //ROS_INFO("%d", xml_times[i]);
        } 
    }
    */
       // }
    //}
                /*
        talon_swerve_drive_controller::FullGen srv;
        srv.request.joint_trajectory = trajectory;
        srv.request.initial_v = 0.0;
        srv.request.final_v = 0.0;
        point_gen.call(srv);
        ROS_WARN("run_test_driver");
        talon_swerve_drive_controller::MotionProfilePoints srv_msg_points;

        srv_msg_points.request.dt = srv.response.dt;	
        srv_msg_points.request.points = srv.response.points;	
        srv_msg_points.request.buffer = true;	
        srv_msg_points.request.mode = false;
        srv_msg_points.request.run = false;

        swerve_control.call(srv_msg_points);
        */
    
    if(MatchData->isAutonomous && !MatchData->isDisabled) {
        if(MatchData->allianceData != "") {
            if(start_time==0) {
                start_time = ros::Time::now().toSec();
            }
            if(ros::Time::now().toSec() >= start_time+2){
                return;
            }
            double time_start_auto = ros::Time::now().toSec();
            elevator_controller::Intake IntakeSrv;
            elevator_controller::ElevatorControlS ElevatorSrv;
            elevator_controller::bool_srv ClampSrv;
            behaviors::IntakeLiftGoal goal;

    ///////////////TESTING/////////////////
            geometry_msgs::Twist vel;
            vel.linear.x = 2;
            vel.linear.y = 0;
            vel.linear.z = 0;
            vel.angular.x = 0;
            vel.angular.y = 0;
            vel.angular.z = 0;
            VelPub.publish(vel);
            return;
            start_pos = AutoMode->position;
            std::vector<double> times;
            if(MatchData->allianceData=="rlr") {
                auto_mode = 1;
                layout = 1;
                times = vectTimes[0];
            }
            else if(MatchData->allianceData=="lrl") {
                auto_mode = 2;
                layout = 1;
                times = vectTimes[1];
            }
            else if(MatchData->allianceData=="rrr") {
                auto_mode = 3;
                layout = 2;
                times = vectTimes[2];
            }
            else if(MatchData->allianceData =="lll") {
                auto_mode = 4;
                layout = 2;
                times = vectTimes[3];
            }

            if(AutoMode->mode[auto_mode-1]==1) {
            //3 cube switch-scale-scale
                //0: Time 1: Go to switch config
                ros::Duration(times[0]).sleep(); //TODO
                switchConfig(ElevatorSrv);
                //1: Time 2: Release Clamp
                ros::Duration(times[1]).sleep(); //TODO
                releaseClamp(ClampSrv);

                //2: Time 3: drop and start intake && go to intake config
                ros::Duration(times[2]).sleep(); //TODO
                goal.IntakeCube = true;
                ac->sendGoal(goal);

                //3: Linebreak sensor: Clamp && release intake && stop running intake

                bool success = ac->waitForResult(ros::Duration(timeout)); //TODO

                if(!success) {
                    ROS_WARN("Failed to intake cube: TIME OUT");
                }

                stopIntake(IntakeSrv);

                intakeConfig(ElevatorSrv); 
                ros::Duration(.2).sleep();

                clamp(ClampSrv);

                releaseIntake(IntakeSrv); 
                
                //4: Time 4: go to scale config && soft-in intake
                ros::Duration(times[3]).sleep();
                midScale(ElevatorSrv); 
                ros::Duration(.1).sleep(); //TODO

                IntakeSrv.request.spring_state = 2;
                IntakeService.call(IntakeSrv);

                //6: Time 5: release Clamp
                ros::Duration(times[4]).sleep(); //TODO
                releaseClamp(ClampSrv);

                //7: Time 6: go to intake config && run intake
                ros::Duration(times[5]).sleep(); // TODO
                intakeConfig(ElevatorSrv);

                goal.IntakeCube = true;
                ac->sendGoal(goal);

                //8: Linebreak sensor: Clamp && release intake && stop running intake
                success = ac->waitForResult(ros::Duration(timeout));
                
                if(!success) {
                    ROS_ERROR("Failed to intake cube: TIME OUT");
                }
                stopIntake(IntakeSrv);

                clamp(ClampSrv);

                releaseIntake(IntakeSrv);
                //10: Time 7: go to mid scale config && soft in intake
                ros::Duration(times[6]).sleep(); //TODO
                midScale(ElevatorSrv);
                IntakeSrv.request.spring_state=2; //soft_in
                IntakeService.call(IntakeSrv);
                
                //11: Time 8: release Clamp
                ros::Duration(times[7]).sleep(); //TODO
                releaseClamp(ClampSrv);

                //12: Time 9: go to intake config && start intake
                ros::Duration(times[8]).sleep(); //TODO
                intakeConfig(ElevatorSrv);

                goal.IntakeCube = true;
                ac->sendGoal(goal);
                //13: Linebreak sensor: Clamp and release cube
                clamp(ClampSrv);
                releaseIntake(IntakeSrv);
            }
            else if(AutoMode->mode[auto_mode-1]==2) {
                //2 cube long elevator_controller::bool_srve
                //0: Time 0: Go to default config && drop intake
                ros::Duration(0).sleep(); //TODO
                defaultConfig(ElevatorSrv);

                IntakeSrv.request.up = false;
                IntakeSrv.request.spring_state = 2; //soft_in

                IntakeService.call(IntakeSrv);
                        
                //1: Time 1: Go to mid scale config && Release Clamp
                ros::Duration(1).sleep(); //TODO
                midScale(ElevatorSrv);
                ros::Duration(.3).sleep(); //TODO

                releaseClamp(ClampSrv);

                //2: Time 1.5: go to intake config && start intake
                ros::Duration(1).sleep(); //TODO
                intakeConfig(ElevatorSrv);

                goal.IntakeCube = true;
                ac->sendGoal(goal);

                //3: Linebreak sensor: Clamp && release intake && stop running intake
                bool success = ac->waitForResult(ros::Duration(timeout)); //TODO
                stopIntake(IntakeSrv);

                if(!success) {
                    ROS_ERROR("Failed to intake cube! TIME OUT");
                }

                clamp(ClampSrv);

                releaseIntake(IntakeSrv);

                //4: Success of command 3: go to default config && soft-in intake
                defaultConfig(ElevatorSrv); 
                ros::Duration(.2).sleep(); //TODO
                IntakeSrv.request.spring_state = 2; //soft in
                IntakeService.call(IntakeSrv);
               
                //5: Time 2: go to mid level scale config
                ros::Duration(2).sleep(); //TODO
                midScale(ElevatorSrv);

                //6: Time 2.5: release Clamp
                ros::Duration(2.5).sleep(); //TODO
                releaseClamp(ClampSrv);

                //7: Time 3: go to intake config && run intake
                ros::Duration(3).sleep(); //TODO
                intakeConfig(ElevatorSrv); 

                goal.IntakeCube = true;
                ac->sendGoal(goal);
                
                success = ac->waitForResult(ros::Duration(timeout));

                //8: Linebreak sensor: Clamp && release intake && stop running intake
                stopIntake(IntakeSrv);
                if(!success) {
                    ROS_WARN("Failed to intake cube: TIME OUT");
                }

                clamp(ClampSrv);
                releaseIntake(IntakeSrv);


            }
            else if(AutoMode->mode[auto_mode-1]==3) {
            //3 cube switch-scale-scale
                //0: Time 1: Go to mid scale config
                ros::Duration(times[0]).sleep(); //TODO
                midScale(ElevatorSrv);
                //1: Time 2: Release Clamp
                ros::Duration(times[1]).sleep(); //TODO
                releaseClamp(ClampSrv);

                //2: Time 3: drop and start intake && go to intake config
                ros::Duration(times[2]).sleep(); //TODO
                goal.IntakeCube = true;
                ac->sendGoal(goal);

                //3: Linebreak sensor: Clamp && release intake && stop running intake

                bool success = ac->waitForResult(ros::Duration(timeout)); //TODO

                if(!success) {
                    ROS_WARN("Failed to intake cube: TIME OUT");
                }

                stopIntake(IntakeSrv);

                intakeConfig(ElevatorSrv); 
                ros::Duration(.2).sleep();

                clamp(ClampSrv);

                releaseIntake(IntakeSrv); 
                
                //4: Time 4: go to switch config && soft-in intake
                ros::Duration(times[3]).sleep();
                switchConfig(ElevatorSrv); 
                ros::Duration(.1).sleep(); //TODO

                IntakeSrv.request.spring_state = 2;
                IntakeService.call(IntakeSrv);

                //6: Time 5: release Clamp
                ros::Duration(times[4]).sleep(); //TODO
                releaseClamp(ClampSrv);

                //7: Time 6: go to intake config && run intake
                ros::Duration(times[5]).sleep(); // TODO
                intakeConfig(ElevatorSrv);

                goal.IntakeCube = true;
                ac->sendGoal(goal);

                //8: Linebreak sensor: Clamp && release intake && stop running intake
                success = ac->waitForResult(ros::Duration(timeout));
                
                if(!success) {
                    ROS_ERROR("Failed to intake cube: TIME OUT");
                }
                stopIntake(IntakeSrv);

                clamp(ClampSrv);

                releaseIntake(IntakeSrv);
                //10: Time 7: go to mid scale config && soft in intake
                ros::Duration(times[6]).sleep(); //TODO
                midScale(ElevatorSrv);
                IntakeSrv.request.spring_state=2; //soft_in
                IntakeService.call(IntakeSrv);
                
                //11: Time 8: release Clamp
                ros::Duration(times[7]).sleep(); //TODO
                releaseClamp(ClampSrv);

                //12: Time 9: go to intake config && start intake
                ros::Duration(times[8]).sleep(); //TODO
                intakeConfig(ElevatorSrv);

                goal.IntakeCube = true;
                ac->sendGoal(goal);
                //13: Linebreak sensor: Clamp and release cube
                clamp(ClampSrv);
                releaseIntake(IntakeSrv);
            }
            else if(AutoMode->mode[auto_mode-1]==4) {

            }
            else if(AutoMode->mode[auto_mode-1]==5) {

            }
            else if(AutoMode->mode[auto_mode-1]==6) {

            }
            else if(AutoMode->mode[auto_mode-1]==7) {

            }
            else if(AutoMode->mode[auto_mode-1]==8) {

            }
            else if(AutoMode->mode[auto_mode-1]==9) {

            }
            else if(AutoMode->mode[auto_mode-1]==10) {

            }
            else if(AutoMode->mode[auto_mode-1]==11) {

            }
            else if(AutoMode->mode[auto_mode-1]==12) {

            }
            else{
                
            }
        }
        else {
            if(!autoStart) {
                autoStart = ros::Time::now().toSec();
            }
            else {
                if(ros::Time::now().toSec() > autoStart + 5) {
                    //RUN CROSS LINE AUTO
                }
            }   
        }   
    }
    else{
        start_time = 0;
    }
    /*
    edlse {
        if(auto_mode == AutoMode->mode[0] || startPos == AutoMode->position) {
            auto_mode = AutoMode->mode[0];
            startPos = AutoMode->position;
            //TODO generate 4 motion profiles
        }
    }*/
}







int main(int argc, char** argv) {
    ros::init(argc, argv, "Auto_state_subscriber");
    ros::NodeHandle n;
    ros::NodeHandle n_params(n, "teleop_params");

    ros::NodeHandle n_params_behaviors(n, "auto_params");
   
    n_params.getParam("high_scale_config_y", high_scale_config_y);
    n_params.getParam("mid_scale_config_x", mid_scale_config_x);
    n_params.getParam("mid_scale_config_y", mid_scale_config_y);
    n_params.getParam("low_scale_config_x", low_scale_config_x);
    n_params.getParam("low_scale_config_y", low_scale_config_y);
    n_params.getParam("switch_config_x", switch_config_x);
    n_params.getParam("switch_config_y", switch_config_y);
    n_params.getParam("exchange_config_x", exchange_config_x);
    n_params.getParam("exchange_config_y", exchange_config_y);
    n_params.getParam("intake_config_x", intake_config_x);
    n_params.getParam("intake_config_y", intake_config_y);
    n_params.getParam("default_x", default_x);
    n_params.getParam("default_y", default_y);
    n_params.getParam("timeout", timeout);

    n_params_behaviors.getParam("modes", modes);
    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>>("auto_interpreter_server", true);
    ac->waitForServer(); 
	point_gen = n.serviceClient<talon_swerve_drive_controller::FullGenCoefs>("/point_gen/command");
	swerve_control = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile");

    //IntakeService = n.advertise<elevator_controller::Intake>("elevator/Intake", 1);
    IntakeService = n.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake");
    //ElevatorService = n.advertise<elevator_controller::ElevatorControl>("elevator/cmd_pos", 1);
    ElevatorService = n.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS");
    //ClampService = n.advertise<std_msgs::Bool>("elevator/Clamp", 1);
    ClampService = n.serviceClient<elevator_controller::bool_srv>("/frcrobot/elevator_controller/clamp");
    VelPub = n.advertise<geometry_msgs::Twist>("/frcrobot/swerve_drive_controller/cmd_vel", 1);

    message_filters::Subscriber<ros_control_boilerplate::AutoMode> auto_mode_sub(n, "Autonomous_Mode", 1);
    message_filters::Subscriber<ros_control_boilerplate::MatchSpecificData> match_data_sub(n, "match_data", 1);
    typedef message_filters::sync_policies::ApproximateTime<ros_control_boilerplate::AutoMode, ros_control_boilerplate::MatchSpecificData> data_sync;
    message_filters::Synchronizer<data_sync> sync(data_sync(10), auto_mode_sub, match_data_sub);
    sync.registerCallback(boost::bind(&auto_modes, _1, _2));
    ROS_WARN("Auto Client loaded");
    ros::Duration(2).sleep();
    generateTrajectory(0, 0, 0);
    ////////////////////////////
    std::vector<int> modess = {0, 0, 0, 0};
    ///////////////////////////
    /*
    ROS_WARN("1");
    if(1 != start_pos) {
        ROS_WARN("2");
        for(int i = 0; i<4; i++) {
            ROS_WARN("3");
            generateTrajectory(i, modess[i], 0);
        }
    }
    /*else {
        for(int i = 0; i<4; i++) {
            if(AutoMode->mode[i] != auto_mode_vect[i]) {
                generateTrajectory(i, AutoMode->mode[i], AutoMode->position);
            }
            auto_mode_vect[i] = AutoMode->mode[i];
        }
    }
    start_pos = AutoMode->position;
    */
    /*
    startPos = 0;
    vectTimes.resize(4);
    trajectories.resize(4);
    for(int layout = 0; layout<2; layout++) { 
        for(int auto_mode = 0; auto_mode<2; auto_mode++) {
            //std::map<std::string, XmlRpc::XmlRpcValue> spline = modes[auto_mode][layout][startPos];
            //std::map<std::string, XmlRpc::XmlRpcValue> spline = modes["0"]["0"]["0"];
            XmlRpc::XmlRpcValue &splines = modes[auto_mode][layout][startPos];
            //std::map<std::string, XmlRpc::XmlRpcValue> spline = xml_times;
            //trajectory_msgs::JointTrajectory trajectory;
            trajectories[auto_mode+2*layout].joint_names.push_back("x_linear_joint");
            trajectories[auto_mode+2*layout].joint_names.push_back("y_linear_joint");
            trajectories[auto_mode+2*layout].joint_names.push_back("z_rotation_joint");
            const size_t num_joints = trajectories[auto_mode+2*layout].joint_names.size();
            trajectories[auto_mode+2*layout].points.resize(3);
            XmlRpc::XmlRpcValue &timesVect = splines["times"];
            XmlRpc::XmlRpcValue &positionsXVect = splines["positionsX"];
            XmlRpc::XmlRpcValue &positionsYVect = splines["positionsY"];
            XmlRpc::XmlRpcValue &positionsZVect = splines["positionsZ"];

            XmlRpc::XmlRpcValue &velocitiesXVect = splines["velocitiesX"];
            XmlRpc::XmlRpcValue &velocitiesYVect = splines["velocitiesY"];
            XmlRpc::XmlRpcValue &velocitiesZVect = splines["velocitiesZ"];

            XmlRpc::XmlRpcValue &accelerationsXVect = splines["accelerationsX"];
            XmlRpc::XmlRpcValue &accelerationsYVect = splines["accelerationsY"];
            XmlRpc::XmlRpcValue &accelerationsZVect = splines["accelerationsZ"];

            vectTimes[auto_mode+layout].resize(timesVect.size());

            for(int i = 0; i<timesVect.size(); i++) { 
                XmlRpc::XmlRpcValue &xml_time_i = timesVect[i];
                const double  time_i = xml_time_i;
                //ROS_ERROR("Time: %d, %f, %lf", time_i, time_i, time_i); 
                vectTimes[auto_mode+2*layout].push_back(timesVect[i]);
                //vectTimes[auto_mode+layout].push_back(0.0);
                ROS_INFO("[mode: %d][layout: %d][pos: %d][i: %d]: %lf", auto_mode, layout, startPos, i, vectTimes[auto_mode+2*layout][i]);

                trajectories[auto_mode+2*layout].points[i].positions.resize(num_joints);
                trajectories[auto_mode+2*layout].points[i].positions[0] =  positionsXVect[i];
                trajectories[auto_mode+2*layout].points[i].positions[1] =  positionsYVect[i];
                trajectories[auto_mode+2*layout].points[i].positions[2] =  positionsZVect[i];
                ROS_INFO("positionsX[%d,%d]: %f", auto_mode, i, trajectories[auto_mode+2*layout].points[i].positions[0]);
                ROS_INFO("positionsY[%d,%d]: %f", auto_mode, i, trajectories[auto_mode+2*layout].points[i].positions[1]);
                ROS_INFO("positionsZ[%d,%d]: %f", auto_mode, i, trajectories[auto_mode+2*layout].points[i].positions[2]);

                trajectories[auto_mode+2*layout].points[i].velocities.resize(num_joints);
                trajectories[auto_mode+2*layout].points[i].velocities[0] =  velocitiesXVect[i];
                trajectories[auto_mode+2*layout].points[i].velocities[1] =  velocitiesYVect[i];
                trajectories[auto_mode+2*layout].points[i].velocities[2] =  velocitiesZVect[i];
                ROS_INFO("velocitiesX[%d,%d]: %f", auto_mode, i, trajectories[auto_mode+2*layout].points[i].velocities[0]);
                ROS_INFO("velocitiesY[%d,%d]: %f", auto_mode, i, trajectories[auto_mode+2*layout].points[i].velocities[1]);
                ROS_INFO("velocitiesZ[%d,%d]: %f", auto_mode, i, trajectories[auto_mode+2*layout].points[i].velocities[2]);

                trajectories[auto_mode+2*layout].points[i].accelerations.resize(num_joints);
                trajectories[auto_mode+2*layout].points[i].accelerations[0] =  accelerationsXVect[i];
                trajectories[auto_mode+2*layout].points[i].accelerations[1] =  accelerationsYVect[i];
                trajectories[auto_mode+2*layout].points[i].accelerations[2] =  accelerationsZVect[i];
                ROS_INFO("accelerationsX[%d,%d]: %f", auto_mode, i, trajectories[auto_mode+2*layout].points[i].accelerations[0]);
                ROS_INFO("accelerationsY[%d,%d]: %f", auto_mode, i, trajectories[auto_mode+2*layout].points[i].accelerations[1]);
                ROS_INFO("accelerationsZ[%d,%d]: %f", auto_mode, i, trajectories[auto_mode+2*layout].points[i].accelerations[2]);

                trajectories[auto_mode+2*layout].points[i].time_from_start = ros::Duration(2*i+1);
                talon_swerve_drive_controller::FullGen srv;
                srv.request.joint_trajectory = trajectories[auto_mode+2*layout];
                srv.request.initial_v = 0.0;
                srv.request.final_v = 0.0;
                point_gen.call(srv);
                ROS_WARN("run_test_driver");
                talon_swerve_drive_controller::MotionProfilePoints srv_msg_points;

                srv_msg_points.request.dt = srv.response.dt;	
                srv_msg_points.request.points = srv.response.points;	
                srv_msg_points.request.buffer = true;	
                srv_msg_points.request.mode = false;
                srv_msg_points.request.run = false;


                //ROS_INFO("%d", xml_times[i]);
            } 
        }
    }
    */
    ROS_WARN("SUCCESS IN autoInterpreterClient.cpp");
    ros::spin();
    return 0;

}
