#include <ros/ros.h>
#include <talon_swerve_drive_controller/Swerve.h>
#include <talon_swerve_drive_controller/FullGen.h>
#include <base_trajectory/GenerateSwerveProfile.h>
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <string>

//Get swerve info here:


std::shared_ptr<swerve> swerve_math;

ros::ServiceClient prof_gen;

bool full_gen(talon_swerve_drive_controller::FullGen::Request &req, talon_swerve_drive_controller::FullGen::Response &res)
{
	ROS_WARN("run_point_gen 1");
	base_trajectory::GenerateSwerveProfile srv_msg;
	srv_msg.request.joint_trajectory  = req.joint_trajectory;
	srv_msg.request.initial_v  = req.initial_v;
	srv_msg.request.final_v  = req.final_v;

	ROS_WARN("run_point_gen 2");
	
	prof_gen.call(srv_msg);

	const int point_count = srv_msg.response.points.size();

	
	res.dt = srv_msg.response.points[1].time_from_start.toSec() - srv_msg.response.points[0].time_from_start.toSec();

	//ROS_INFO_STREAM("dt: " << res.dt);	

	ROS_WARN("BUFFERING");
	//TODO: optimize code?
	std::array<double, WHEELCOUNT> curPos;
	for (int i = 0; i < WHEELCOUNT; i++)
		curPos[i] = 0; //TODO: FILL THIS OUT SOMEHOW

	std::array<bool, WHEELCOUNT> holder;
	
	std::array<Eigen::Vector2d, WHEELCOUNT> angles_positions  = swerve_math->motorOutputs({srv_msg.response.points[0].positions[0], srv_msg.response.points[0].positions[1]}, srv_msg.response.points[0].positions[2], M_PI/2 + srv_msg.response.points[0].positions[2], false, holder, false, curPos, false);
		//TODO: angles on the velocity array below are superfluous, could remove
	std::array<Eigen::Vector2d, WHEELCOUNT> angles_velocities  = swerve_math->motorOutputs({srv_msg.response.points[0].velocities[0], srv_msg.response.points[0].velocities[1]}, srv_msg.response.points[0].velocities[2], M_PI/2 + srv_msg.response.points[0].positions[2], false, holder, false, curPos, false);
	for (size_t i = 0; i < WHEELCOUNT; i++)
		curPos[i] = angles_positions[i][1];
	//Do first point and initialize stuff
	
	/*
	TODO: IMPLEMENT BELOW
	if(motion_profile_mode == steering_joints_[0].getMode())
	{
		for(size_t i = 0; i < WHEELCOUNT; i++)
		{
			speed_joints_[i].setCommand(0);
			steering_joints_[i].setCommand(0);
		}
	}
	*/
	ROS_WARN("data");
	//ROS_INFO_STREAM("pos_0:" << srv_msg.response.points[0].positions[0] << "pos_1:" << srv_msg.response.points[0].positions[1] <<"pos_2:" <<  srv_msg.response.points[0].positions[2]);

	res.points.resize(point_count);

	for(size_t i = 0; i < WHEELCOUNT; i++)
	{
		res.points[0].drive_pos.push_back(angles_positions[i][0]);
		res.points[0].drive_vel.push_back(angles_velocities[i][0]);
		res.points[0].steer_pos.push_back(angles_positions[i][1]);
		//ROS_INFO_STREAM("drive_pos: " << res.points[0].drive_pos[i] << "drive_vel: " << res.points[0].drive_vel[i] << "steer_pos: " << res.points[0].steer_pos[i] << " nan_test: " <<angles_positions[i][0]); 
	}

	for(int i = 0; i < point_count - 1; i++)
	{
	std::array<Eigen::Vector2d, WHEELCOUNT> angles_positions  = swerve_math->motorOutputs({srv_msg.response.points[i+1].positions[0] - srv_msg.response.points[i].positions[0], srv_msg.response.points[i+1].positions[1] - srv_msg.response.points[i].positions[1]}, srv_msg.response.points[i+1].positions[2] - srv_msg.response.points[i].positions[2], M_PI/2 + srv_msg.response.points[i+1].positions[2], false, holder, false, curPos, false);
		//TODO: angles on the velocity array below are superfluous, could remove
	std::array<Eigen::Vector2d, WHEELCOUNT> angles_velocities  = swerve_math->motorOutputs({srv_msg.response.points[i+1].velocities[0], srv_msg.response.points[i+1].velocities[1]}, srv_msg.response.points[i+1].velocities[2], M_PI/2 + srv_msg.response.points[i+1].positions[2], false, holder, false, curPos, false);
		for (size_t k = 0; k < WHEELCOUNT; k++)
			curPos[k] = angles_positions[k][1];

		//ROS_INFO_STREAM("pos_0:" << srv_msg.response.points[i+1].positions[0] << "pos_1:" << srv_msg.response.points[i+1].positions[1] <<"pos_2:" <<  srv_msg.response.points[i+1].positions[2]);
		for(size_t k = 0; k < WHEELCOUNT; k++)
		{
			res.points[i+1].drive_pos.push_back(angles_positions[k][0] + res.points[i].drive_pos[k]);
			res.points[i+1].drive_vel.push_back(angles_velocities[k][0]);
			res.points[i+1].steer_pos.push_back(angles_positions[k][1]);
			//ROS_INFO_STREAM("drive_pos: " << res.points[i+1].drive_pos[k] << "drive_vel: " << res.points[i+1].drive_vel[k] << "steer_pos: " << res.points[i+1].steer_pos[i]); 
		}
	}

	return true;	
}
int main(int argc, char **argv)
{
        ros::init(argc, argv, "point_gen");
        ros::NodeHandle nh;

	ros::NodeHandle controller_nh(nh, "swerve_drive_controller");
	

	ros::ServiceServer service = nh.advertiseService("/point_gen/command", full_gen);

	//double wheel_radius;
	swerveVar::driveModel model;
	bool invert_wheel_angle;
        swerveVar::ratios drive_ratios;
        swerveVar::encoderUnits units;
	

	bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", model.wheelRadius);
        bool lookup_max_speed = !controller_nh.getParam("max_speed", model.maxSpeed);
        bool lookup_mass = !controller_nh.getParam("mass", model.mass);
        bool lookup_motor_free_speed = !controller_nh.getParam("motor_free_speed", model.motorFreeSpeed);
        bool lookup_motor_stall_torque = !controller_nh.getParam("motor_stall_torque", model.motorStallTorque);
        // TODO : why not just use the number of wheels read from yaml?
        bool lookup_motor_quantity = !controller_nh.getParam("motor_quantity", model.motorQuantity);
        bool lookup_invert_wheel_angle = !controller_nh.getParam("invert_wheel_angle", invert_wheel_angle);
        bool lookup_ratio_encoder_to_rotations = !controller_nh.getParam("ratio_encoder_to_rotations", drive_ratios.encodertoRotations);
        bool lookup_ratio_motor_to_rotations = !controller_nh.getParam("ratio_motor_to_rotations", drive_ratios.motortoRotations);
        bool lookup_ratio_motor_to_steering = !controller_nh.getParam("ratio_motor_to_steering", drive_ratios.motortoSteering); // TODO : not used?
        bool lookup_encoder_drive_get_V_units = !controller_nh.getParam("encoder_drive_get_V_units", units.rotationGetV);
        bool lookup_encoder_drive_get_P_units = !controller_nh.getParam("encoder_drive_get_P_units", units.rotationGetP);
        bool lookup_encoder_drive_set_V_units = !controller_nh.getParam("encoder_drive_set_V_units", units.rotationSetV);
        bool lookup_encoder_drive_set_P_units = !controller_nh.getParam("encoder_drive_set_P_units", units.rotationSetP);
        bool lookup_encoder_steering_get_units = !controller_nh.getParam("encoder_steering_get_units", units.steeringGet);
        bool lookup_encoder_steering_set_units = !controller_nh.getParam("encoder_steering_set_units", units.steeringSet);
	std::array<Eigen::Vector2d, WHEELCOUNT> wheel_coords;
        bool lookup_wheel1x = !controller_nh.getParam("wheel_coords1x", wheel_coords[0][0]);
        bool lookup_wheel2x = !controller_nh.getParam("wheel_coords2x", wheel_coords[1][0]);
        bool lookup_wheel3x = !controller_nh.getParam("wheel_coords3x", wheel_coords[2][0]);
        bool lookup_wheel4x = !controller_nh.getParam("wheel_coords4x", wheel_coords[3][0]);
        bool lookup_wheel1y = !controller_nh.getParam("wheel_coords1y", wheel_coords[0][1]);
        bool lookup_wheel2y = !controller_nh.getParam("wheel_coords2y", wheel_coords[1][1]);
        bool lookup_wheel3y = !controller_nh.getParam("wheel_coords3y", wheel_coords[2][1]);
        bool lookup_wheel4y = !controller_nh.getParam("wheel_coords4y", wheel_coords[3][1]);	
	
	//ROS_WARN("point_init");
	//ROS_INFO_STREAM("model max speed: " << model.maxSpeed << " radius: " << model.wheelRadius);

	XmlRpc::XmlRpcValue wheel_list;
	controller_nh.getParam("steering", wheel_list);
	
	std::vector<std::string> wheel_names;
	wheel_names.resize(wheel_list.size());
	for (int i = 0; i < wheel_list.size(); ++i)
        {	
		wheel_names[i] = static_cast<std::string>(wheel_list[i]);
	}
	std::vector<double> offsets;
        for (auto it = wheel_names.cbegin(); it != wheel_names.cend(); ++it)
        {
                ros::NodeHandle nh(controller_nh, *it);
                double dbl_val = 0;
                if (!nh.getParam("offset", dbl_val))
                        ROS_ERROR_STREAM("Can not read offset for " << *it);
                offsets.push_back(dbl_val);
        }


	swerve_math = std::make_shared<swerve>(wheel_coords, offsets, invert_wheel_angle, drive_ratios, units, model);
	//Something to get intial wheel position
	
	prof_gen = nh.serviceClient<base_trajectory::GenerateSwerveProfile>("/base_trajectory/command");

	ros::spin();
}
