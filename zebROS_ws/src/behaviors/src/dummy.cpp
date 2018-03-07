/*
//#include "geometry_msgs/Twist.h"
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
//#include "ros/console.h"
*/
/*
ros::ServiceClient point_gen;
ros::ServiceClient swerve_control;


static int startPos = -1;
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

//static std::vector<

int main(int argc, char** argv) {
    ros::init(argc, argv, "Auto_state_subscriber");
    ros::NodeHandle n;
    //ros::NodeHandle n_params(n, "teleop_params");

    //ros::NodeHandle n_params_behaviors(n, "auto_params");
   */
	/*
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

    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>>("auto_interpreter_server", true);
    ac->waitForServer(); 
    */
    /*
	point_gen = n.serviceClient<talon_swerve_drive_controller::FullGen>("/point_gen/command");
    talon_swerve_drive_controller::FullGenCoefs srv;
    std::vector<double> v = {0, 0, 0, 0, 0, 0}; 
    talon_swerve_drive_controller::Coefs orients1;
    talon_swerve_drive_controller::Coefs orients2;
    talon_swerve_drive_controller::Coefs orients3;
    talon_swerve_drive_controller::Coefs orients4;
    talon_swerve_drive_controller::Coefs orients5;

    orients1.push_back(0.0);
    orients1.push_back(0.0);
    orients1.push_back(0.0);
    orients1.push_back(0.0);
    orients1.push_back(0.0);
    orients1.push_back(0.0);
    orients2.push_back(0.0);
    orients2.push_back(0.0);
    orients2.push_back(0.0);
    orients2.push_back(0.0);
    orients2.push_back(0.0);
    orients2.push_back(0.0);
    orients3.push_back(0.0);
    orients3.push_back(0.0);
    orients3.push_back(0.0);
    orients3.push_back(0.0);
    orients3.push_back(0.0);
    orients3.push_back(0.0);
    orients4.push_back(0.0);
    orients4.push_back(0.0);
    orients4.push_back(0.0);
    orients4.push_back(0.0);
    orients4.push_back(0.0);
    orients4.push_back(0.0);
    orients5.push_back(0.0);
    orients5.push_back(0.0);
    orients5.push_back(0.0);
    orients5.push_back(0.0);
    orients5.push_back(0.0);
    orients5.push_back(0.0);

    talon_swerve_drive_controller::Coefs x1 = [6.276, -17.843, 13.332, 4.585, -9.12, -.01];
    x1.push_back(6.276);
    x1.push_back(-17.843);
    x1.push_back(13.332);
    x1.push_back(4.585);
    x1.push_back(-9.12);
    x1.push_back(-.01);
    talon_swerve_drive_controller::Coefs y1 = [7.307, -15.396, 6.709, -.06, 5,63, .37];
    

    talon_swerve_drive_controller::Coefs x2 = [.72, -5.199, 14.608, -19.547, 12.52, -5.882];
    talon_swerve_drive_controller::Coefs y2 = [13.1375, -98.4705, 283.5635, -390.4815, 259.055, -62.244];

    talon_swerve_drive_controller::Coefs x3 = [1.105, -10.645, 38.195, -61.165, 39.42, -7.09];
    talon_swerve_drive_controller::Coefs y3 = [-7.8375, 98.2175, -484.8275, 1177.3525, -1407.605, 671.06];

    talon_swerve_drive_controller::Coefs x4 = [-6.329, 115.497, -838.103, 3021.029, -5406.264, 3839.222];
    talon_swerve_drive_controller::Coefs y4 = [11.375, -199.24, 1385.465, -4779.95, 8182.66, -5556.49];

    talon_swerve_drive_controller::Coefs x5 = [-3.157, 67.893, -582.209, 2490.205, -5316.12, 4533.11];
    talon_swerve_drive_controller::Coefs y5 = [-7.97, 179.62, -1611.54, 7194.35, -15982.06, 14143.03];

    double[] points = [1, 2, 3, 4, 5];

    double v_init = 0;
    double v_fin = 0;

    srv.orient_coefs[0] = orients1;
    srv.orient_coefs[1] = orients2;
    srv.orient_coefs[2] = orients3;
    srv.orient_coefs[3] = orients4;
    srv.orient_coefs[4] = orients5;

    srv.x_coefs[0] = x1;
    srv.x_coefs[1] = x2;
    srv.x_coefs[2] = x3;
    srv.x_coefs[3] = x4;
    srv.x_coefs[4] = x5;

    srv.y_coefs[0] = y1;
    srv.y_coefs[1] = y2;
    srv.y_coefs[2] = y3;
    srv.y_coefs[3] = y4;
    srv.y_coefs[4] = y5;

    srv.end_points = points;

    srv.inital_v = v_init;
    srv.final_v = v_fin;
    
    point_gen.call(srv);
    /*
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
    //ros::spin();
    return 0;

}
