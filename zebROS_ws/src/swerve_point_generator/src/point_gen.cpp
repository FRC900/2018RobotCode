#include <Eigen/Dense>
#include <vector>
#include <array>
#include <string>
#include <ros/ros.h>
#include <swerve_math/Swerve.h>
#include <swerve_point_generator/profiler.h>
#include <swerve_point_generator/FullGenCoefs.h>
#include <swerve_point_generator/GenerateSwerveProfile.h>
#include <talon_swerve_drive_controller/MotionProfile.h> //Only needed for visualization
#include <talon_swerve_drive_controller/MotionProfilePoints.h> //Only needed for visualization
#include <talon_swerve_drive_controller/WheelPos.h>

//Get swerve info here:

std::shared_ptr<swerve> swerve_math;
std::shared_ptr<swerve_profile::swerve_profiler> profile_gen;

ros::ServiceClient graph_prof;
ros::ServiceClient get_pos;
ros::ServiceClient graph_swerve_prof;
double defined_dt;

double f_v;
double f_s;
double f_a;
double f_s_s;
double f_s_v;


swerveVar::driveModel model;

bool full_gen(swerve_point_generator::FullGenCoefs::Request &req, swerve_point_generator::FullGenCoefs::Response &res)
{
	ROS_ERROR("running point gen");

	std::array<double, WHEELCOUNT> curPos;
	talon_swerve_drive_controller::WheelPos pos_msg;
	if (!get_pos.call(pos_msg))
	{
		ROS_ERROR("failed to get wheel pos in point gen, maybe asked swerve drive controller before it was started up?");
		return false;
	}
	for (int i = 0; i < WHEELCOUNT; i++)
		curPos[i] = pos_msg.response.positions[i]; //TODO: FILL THIS OUT SOMEHOW
	int n = 8;
	int k_p = 1;
	res.points.resize(155/defined_dt);
	int prev_point_count = 0;
	talon_swerve_drive_controller::MotionProfile graph_msg;
	ROS_INFO_STREAM("here1");
	for(int s = 0; s < req.spline_groups.size(); s++)
	{
		int priv_num = 0;
		if(s>0)
		{
			priv_num = req.spline_groups[s-1];
		}
		n = round(req.wait_before_group[s] / defined_dt);
		std::vector<swerve_profile::spline_coefs> x_splines, y_splines, orient_splines;
		swerve_profile::spline_coefs temp_holder_s;

		ROS_INFO_STREAM("HERE2");

		int neg_x = req.x_invert[s] ? -1 : 1;
		ROS_INFO_STREAM("why1");
		ROS_INFO_STREAM("neg_x: " << neg_x);
		//ROS_INFO_STREAM("req.x_coefs[0].spline in point_gen: " << req.x_coefs[1].spline[0] << req.x_coefs[1].spline[1] << req.x_coefs[1].spline[2] << req.x_coefs[1].spline[3] << req.x_coefs[1].spline[4]);
		ROS_INFO_STREAM("x_spline in point gen: " << req.x_coefs.size());
		for (size_t i = priv_num; i < req.spline_groups[s]; i++)
		{
			temp_holder_s.a = req.orient_coefs[i].spline[0] * neg_x;
			temp_holder_s.b = req.orient_coefs[i].spline[1] * neg_x;
			temp_holder_s.c = req.orient_coefs[i].spline[2] * neg_x;
			temp_holder_s.d = req.orient_coefs[i].spline[3] * neg_x;
			temp_holder_s.e = req.orient_coefs[i].spline[4] * neg_x;
			temp_holder_s.f = req.orient_coefs[i].spline[5] * neg_x;

			orient_splines.push_back(temp_holder_s);
		}
		ROS_INFO_STREAM("why2");
		for (size_t i = priv_num; i < req.spline_groups[s]; i++)
		{
			temp_holder_s.a = req.x_coefs[i].spline[0] * neg_x;
			temp_holder_s.b = req.x_coefs[i].spline[1] * neg_x;
			temp_holder_s.c = req.x_coefs[i].spline[2] * neg_x;
			temp_holder_s.d = req.x_coefs[i].spline[3] * neg_x;
			temp_holder_s.e = req.x_coefs[i].spline[4] * neg_x;
			temp_holder_s.f = req.x_coefs[i].spline[5] * neg_x;
			ROS_INFO_STREAM("this");

			x_splines.push_back(temp_holder_s);
		}
		ROS_INFO_STREAM("why3");
		for (size_t i = priv_num; i < req.spline_groups[s]; i++)
		{
			temp_holder_s.a = req.y_coefs[i].spline[0];
			temp_holder_s.b = req.y_coefs[i].spline[1];
			temp_holder_s.c = req.y_coefs[i].spline[2];
			temp_holder_s.d = req.y_coefs[i].spline[3];
			temp_holder_s.e = req.y_coefs[i].spline[4];
			temp_holder_s.f = req.y_coefs[i].spline[5];

			y_splines.push_back(temp_holder_s);
		}
		ROS_INFO_STREAM("here3");
		std::vector<double> end_points_holder;
		double shift_by = 0;
		if(s!=0)
		{
			shift_by = req.end_points[priv_num-1];
		}
		for (size_t i = priv_num; i < req.spline_groups[s]; i++)
		{
			//ROS_INFO_STREAM("hrer: " << req.end_points[i] - shift_by<< " r_s: " <<  req.spline_groups[s] <<  " s: "<< s);
			end_points_holder.push_back(req.end_points[i] - shift_by);
			
		}
		double t_shift = req.t_shift[s];
		bool flip_dirc = req.flip[s];	
	
		swerve_point_generator::GenerateSwerveProfile::Response srv_msg; //TODO FIX THIS, HACK
		ROS_INFO_STREAM("i assume this should be working");
		//srv_msg.points.resize(0);
		//ROS_INFO_STREAM("x_splines in point_gen: " << x_splines[1].a << x_splines[1].b << x_splines[1].c << x_splines[1].d << x_splines[1].e);
		ROS_INFO_STREAM("x_splines size in point_gen: " << x_splines.size());
		profile_gen->generate_profile(x_splines, y_splines, orient_splines, req.initial_v, req.final_v, srv_msg, end_points_holder, t_shift, flip_dirc);
		ROS_INFO_STREAM("i assume this should not be working???");
		const int point_count = srv_msg.points.size();
		ROS_INFO_STREAM("point count: " << point_count);


		

		graph_msg.request.joint_trajectory.header = srv_msg.header;


		

		res.dt = defined_dt;

		//ROS_INFO_STREAM("dt: " << res.dt);

		//ROS_WARN("BUFFERING");
		//TODO: optimize code?



		std::array<bool, WHEELCOUNT> holder;

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
		//ROS_INFO_STREAM("pos_0:" << srv_msg.points[0].positions[0] << "pos_1:" << srv_msg.points[0].positions[1] <<"pos_2:" <<  srv_msg.points[0].positions[2]);



		// Bounds checking - not safe to proceed with setting up angle
		// positions and velocities if data is not as expected.
		if (srv_msg.points.size() < 2) {
			ROS_ERROR("Need at least 2 points");
			return false;
		}
		for (const auto point : srv_msg.points) {
			if (point.positions.size() < 3) {
				ROS_ERROR("Not enough positions in point");
				return false;
			}
		}
		std::array<Eigen::Vector2d, WHEELCOUNT> angles_positions  = swerve_math->motorOutputs({srv_msg.points[1].positions[0] - srv_msg.points[0].positions[0], srv_msg.points[1].positions[1] - srv_msg.points[0].positions[1]}, srv_msg.points[1].positions[2] - srv_msg.points[0].positions[2], srv_msg.points[1].positions[2], false, holder, false, curPos, false);
		//TODO: angles on the velocity array below are superfluous, could remove
		std::array<Eigen::Vector2d, WHEELCOUNT> angles_velocities  = swerve_math->motorOutputs({srv_msg.points[1].velocities[0], srv_msg.points[1].velocities[1]}, -srv_msg.points[1].velocities[2], srv_msg.points[1].positions[2], false, holder, false, curPos, false);
		for (size_t k = 0; k < WHEELCOUNT; k++)
			curPos[k] = angles_positions[k][1];

		//ROS_INFO_STREAM("pos_0:" << srv_msg.points[i+1].positions[0] << "pos_1:" << srv_msg.points[i+1].positions[1] <<"pos_2:" <<  srv_msg.points[i+1].positions[2] << " counts: " << point_count << " i: "<< i << " wheels: " << WHEELCOUNT);
		std::array<double, WHEELCOUNT> vel_sum;
		for (int i = prev_point_count; i < n+prev_point_count; i++)
		{
				
			graph_msg.request.joint_trajectory.points.push_back(srv_msg.points[0]);
	
			for (size_t k = 0; k < WHEELCOUNT; k++)
			{
				

				res.points[i].hold.push_back(true);

				if(s==0)
				{
					res.points[i].drive_pos.push_back(angles_positions[k][0]);
				}
				else
				{
					res.points[i].drive_pos.push_back(res.points[i-1].drive_pos[k]);

				}
				//ROS_WARN("re");

				res.points[i].drive_f.push_back(0);
				vel_sum[k] = 0;

				res.points[i].steer_pos.push_back(angles_positions[k][1]);
				res.points[i].steer_f.push_back(0); 
				//ROS_INFO_STREAM("drive_pos: " << res.points[i+1].drive_pos[k] << "drive_f: " << res.points[i+1].drive_vel[k] << "steer_pos: " << res.points[i+1].steer_pos[i]);
			}
		}

		
		graph_msg.request.joint_trajectory.points.insert(graph_msg.request.joint_trajectory.points.end(), srv_msg.points.begin(), srv_msg.points.end());

		std::array<double, WHEELCOUNT> prev_vels;
		std::array<double, WHEELCOUNT> prev_steer_pos;
		for (size_t k = 0; k < WHEELCOUNT; k++)
		{
			prev_vels[k] = 0;
			prev_steer_pos[k] = angles_positions[k][1]; 
		}		
		for (int i = 0; i < point_count - k_p; i++)
		{
			std::array<Eigen::Vector2d, WHEELCOUNT> angles_positions  = swerve_math->motorOutputs({srv_msg.points[i + 1].positions[0] - srv_msg.points[i].positions[0], srv_msg.points[i + 1].positions[1] - srv_msg.points[i].positions[1]}, srv_msg.points[i + 1].positions[2] - srv_msg.points[i].positions[2], srv_msg.points[i + 1].positions[2], false, holder, false, curPos, false);
			//TODO: angles on the velocity array below are superfluous, could remove
			std::array<Eigen::Vector2d, WHEELCOUNT> angles_velocities  = swerve_math->motorOutputs({srv_msg.points[i + 1].velocities[0], srv_msg.points[i + 1].velocities[1]}, srv_msg.points[i + 1].velocities[2], srv_msg.points[i + 1].positions[2], false, holder, false, curPos, false);
			for (size_t k = 0; k < WHEELCOUNT; k++)
				curPos[k] = angles_positions[k][1];

			//ROS_INFO_STREAM("pos_0:" << srv_msg.points[i+1].positions[0] << "pos_1:" << srv_msg.points[i+1].positions[1] <<"pos_2:" <<  srv_msg.points[i+1].positions[2] << " counts: " << point_count << " i: "<< i << " wheels: " << WHEELCOUNT);
			for (size_t k = 0; k < WHEELCOUNT; k++)
			{
				res.points[i + n + prev_point_count].hold.push_back(false);


				//ROS_WARN("hhhhere");
				if (i != 0 || n != 0 || s != 0)
				{
					res.points[i + n + prev_point_count].drive_pos.push_back(angles_positions[k][0] + res.points[i + n - 1 + prev_point_count].drive_pos[k]);
				}
				else
				{
					res.points[i + n + prev_point_count ].drive_pos.push_back(angles_positions[k][0]);
				}

				if (i > point_count - k_p - 2)
				{
					//ROS_INFO_STREAM("final pos" << angles_positions[k][0] + res.points[i + n - 1 + prev_point_count].drive_pos[k]);
					//ROS_INFO_STREAM("vel sum" << vel_sum[k]);
					res.points[i + n + prev_point_count].drive_f.push_back(0);
					res.points[i + n + prev_point_count].steer_f.push_back(0);
				}
				else
				{
					int sign_v = angles_velocities[k][0] < 0 ? -1 : angles_velocities[k][0] > 0 ? 1 : 0; 
					res.points[i + n + prev_point_count].drive_f.push_back(angles_velocities[k][0] * f_v + sign_v * f_s + f_a /* / ( -fabs(angles_velocities[k][0]) / (model.maxSpeed * 1.2) + 1.05 ) */ * (angles_velocities[k][0] - prev_vels[k]) / defined_dt);
					prev_vels[k] = angles_velocities[k][0];
					vel_sum[k] += angles_velocities[k][0];
		
					
					double steer_v = (angles_positions[k][1] - prev_steer_pos[k]) / defined_dt;
					int sign_steer_v = steer_v < 0 ? -1 : steer_v > 0 ? 1 : 0; 
					res.points[i + n + prev_point_count].steer_f.push_back(steer_v * f_s_v + sign_steer_v * f_s_s);
				}

				res.points[i + n + prev_point_count].steer_pos.push_back(angles_positions[k][1]);
				
				prev_steer_pos[k] = angles_positions[k][1]; 
				
				//ROS_INFO_STREAM("drive_pos: " << res.points[i+n+prev_point_count].drive_pos[k] << "drive_f: " << res.points[i+n+prev_point_count].drive_f[k] << "steer_pos: " << res.points[i+n+prev_point_count].steer_pos[k]);
			}
		}	
		prev_point_count += point_count + n - k_p;	
		//ROS_ERROR_STREAM("l: " <<  prev_point_count << " P: " << point_count);
	}
			
	
	graph_prof.call(graph_msg);
	res.points.erase(res.points.begin() + prev_point_count, res.points.end());
	ROS_INFO_STREAM("profile time: " << res.points.size() * defined_dt);
	
	res.joint_trajectory = graph_msg.request.joint_trajectory;

	//talon_swerve_drive_controller::MotionProfilePoints graph_swerve_msg;
	//graph_swerve_msg.request.points = res.points;
	//graph_swerve_prof.call(graph_swerve_msg);
	//ROS_WARN("FIN");
	return true;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "point_gen");
	ros::NodeHandle nh;

	ros::NodeHandle controller_nh(nh, "swerve_drive_controller");

	//double wheel_radius;
	bool invert_wheel_angle;
	swerveVar::ratios drive_ratios;
	swerveVar::encoderUnits units;
	double max_accel;
	double max_brake_accel;
	double ang_accel_conv;


	
	if (!controller_nh.getParam("f_s", f_s))
		ROS_ERROR("Could not read f_s in point gen");
	if (!controller_nh.getParam("f_a", f_a))
		ROS_ERROR("Could not read f_a in point gen");
	if (!controller_nh.getParam("f_v", f_v))
		ROS_ERROR("Could not read f_v in point gen");
	if (!controller_nh.getParam("f_s_v", f_s_v))
		ROS_ERROR("Could not read f_s_v in point gen");
	if (!controller_nh.getParam("f_s_s", f_s_s))
		ROS_ERROR("Could not read f_s_s in point gen");

	if (!controller_nh.getParam("wheel_radius", model.wheelRadius))
		ROS_ERROR("Could not read wheel_radius in point_gen");
	if (!controller_nh.getParam("max_accel", max_accel))
		ROS_ERROR("Could not read max_accel in point_gen");
	if (!controller_nh.getParam("max_brake_accel", max_brake_accel))
		ROS_ERROR("Could not read max_brake_accel in point_gen");
	if (!controller_nh.getParam("ang_accel_conv", ang_accel_conv))
		ROS_ERROR("Could not read ang_accel_conv in point_gen");
	if (!controller_nh.getParam("max_speed", model.maxSpeed))
		ROS_ERROR("Could not read max_speed in point_gen");
	if (!controller_nh.getParam("mass", model.mass))
		ROS_ERROR("Could not read mass in point_gen");
	if (!controller_nh.getParam("motor_free_speed", model.motorFreeSpeed))
		ROS_ERROR("Could not read motor_free_speed in point_gen");
	if (!controller_nh.getParam("motor_stall_torque", model.motorStallTorque))
		ROS_ERROR("Could not read motor_stall_torque in point_gen");
	// TODO : why not just use the number of wheels read from yaml?
	if (!controller_nh.getParam("motor_quantity", model.motorQuantity))
		ROS_ERROR("Could not read motor_quantity in point_gen");
	if (!controller_nh.getParam("invert_wheel_angle", invert_wheel_angle))
		ROS_ERROR("Could not read invert_wheel_angle in point_gen");
	if (!controller_nh.getParam("ratio_encoder_to_rotations", drive_ratios.encodertoRotations))
		ROS_ERROR("Could not read ratio_encoder_to_rotations in point_gen");
	if (!controller_nh.getParam("ratio_motor_to_rotations", drive_ratios.motortoRotations))
		ROS_ERROR("Could not read ratio_motor_to_rotations in point_gen");
	if (!controller_nh.getParam("ratio_motor_to_steering", drive_ratios.motortoSteering))
		ROS_ERROR("Could not read ratio_motor_to_steering in point_gen");
	if (!controller_nh.getParam("encoder_drive_get_V_units", units.rotationGetV))
		ROS_ERROR("Could not read encoder_drive_get_V_units in point_gen");
	if (!controller_nh.getParam("encoder_drive_get_P_units", units.rotationGetP))
		ROS_ERROR("Could not read encoder_drive_get_P_units in point_gen");
	if (!controller_nh.getParam("encoder_drive_set_V_units", units.rotationSetV))
		ROS_ERROR("Could not read encoder_drive_set_V_units in point_gen");
	if (!controller_nh.getParam("encoder_drive_set_P_units", units.rotationSetP))
		ROS_ERROR("Could not read encoder_drive_set_P_units in point_gen");
	if (!controller_nh.getParam("encoder_steering_get_units", units.steeringGet))
		ROS_ERROR("Could not read encoder_steering_get_units in point_gen");
	if (!controller_nh.getParam("encoder_steering_set_units", units.steeringSet))
		ROS_ERROR("Could not read encoder_steering_set_units in point_gen");
	std::array<Eigen::Vector2d, WHEELCOUNT> wheel_coords;
	if (!controller_nh.getParam("wheel_coords1x", wheel_coords[0][0]))
		ROS_ERROR("Could not read wheel_coords1x in point_gen");
	if (!controller_nh.getParam("wheel_coords2x", wheel_coords[1][0]))
		ROS_ERROR("Could not read wheel_coords2x in point_gen");
	if (!controller_nh.getParam("wheel_coords3x", wheel_coords[2][0]))
		ROS_ERROR("Could not read wheel_coords3x in point_gen");
	if (!controller_nh.getParam("wheel_coords4x", wheel_coords[3][0]))
		ROS_ERROR("Could not read wheel_coords4x in point_gen");
	if (!controller_nh.getParam("wheel_coords1y", wheel_coords[0][1]))
		ROS_ERROR("Could not read wheel_coords1y in point_gen");
	if (!controller_nh.getParam("wheel_coords2y", wheel_coords[1][1]))
		ROS_ERROR("Could not read wheel_coords2y in point_gen");
	if (!controller_nh.getParam("wheel_coords3y", wheel_coords[2][1]))
		ROS_ERROR("Could not read wheel_coords3y in point_gen");
	if (!controller_nh.getParam("wheel_coords4y", wheel_coords[3][1]))
		ROS_ERROR("Could not read wheel_coords4y in point_gen");

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
	defined_dt = .02;
	profile_gen = std::make_shared<swerve_profile::swerve_profiler>(hypot(wheel_coords[0][0], wheel_coords[0][1]), max_accel, model.maxSpeed, 1, 1, defined_dt, ang_accel_conv, max_brake_accel); //Fix last val
	//Something to get intial wheel position

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	graph_prof = nh.serviceClient<talon_swerve_drive_controller::MotionProfile>("/visualize_profile", false, service_connection_header);
	graph_swerve_prof = nh.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/visualize_swerve_profile", false, service_connection_header);

	ros::service::waitForService("/frcrobot/swerve_drive_controller/wheel_pos");
	ROS_ERROR("DONE WAITING FOR wheel_pos");
	get_pos = nh.serviceClient<talon_swerve_drive_controller::WheelPos>("/frcrobot/swerve_drive_controller/wheel_pos", false, service_connection_header);

	// Once everything this node needs is available, open
	// it up to connections from the outside
	//ROS_ERROR("BEFORE advertiseService");
	ros::ServiceServer service = nh.advertiseService("/point_gen/command", full_gen);
	//ROS_ERROR("AFTER advertiseService");

	ros::spin();
}
