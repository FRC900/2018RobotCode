#include "robot_visualizer/robot_visualize.h"

double width;
double length;
double wheel_base_pos_offset;
double intake_length;
double intake_width;
double intake_angle_in;
double intake_angle_out;
double intake_position;
double arm_length_place_forward;
double arm_length_place_backward;
double arm_length_place_switch;
double arm_length_intake;
double arm_pivot_position;

double cube = 13. * 2.54 / 100.;

ros::Publisher robot_pub, arm_pub, intake_pub;
ros::Subscriber robot_state_sub;

geometry_msgs::PolygonStamped robot_poly;
std::vector<geometry_msgs::PolygonStamped> arm_polys, intake_polys;


int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_visualize");
    ros::NodeHandle n;

    ros::NodeHandle n_params(n, "robot_visual");
	
	if (!n_params.getParam("width", width))
        ROS_ERROR("could not read width in robot_visualize");
	if (!n_params.getParam("length", length))
        ROS_ERROR("Could not read length in robot_visualize");
	if (!n_params.getParam("wheel_base_pos_offset", wheel_base_pos_offset))
        ROS_ERROR("could not read wheel_base_pos_offset in robot_visualize");
	if (!n_params.getParam("intake_length", intake_length))
        ROS_ERROR("Could not read intake_length in robot_visualize");
	if (!n_params.getParam("intake_width", intake_width))
        ROS_ERROR("Could not read intake_width in robot_visualize");
	if (!n_params.getParam("intake_angle_in", intake_angle_in))
        ROS_ERROR("Could not read intake_angle_in in robot_visualize");
	if (!n_params.getParam("intake_angle_out", intake_angle_out))
        ROS_ERROR("Could not read intake_angle_out in robot_visualize");
	if (!n_params.getParam("intake_position", intake_position))
        ROS_ERROR("could not read intake_position in robot_visualize");
	if (!n_params.getParam("arm_length_place_forward", arm_length_place_forward))
        ROS_ERROR("Could not read arm_length_place_forward in robot_visualize");
	if (!n_params.getParam("arm_length_place_backward", arm_length_place_backward))
        ROS_ERROR("Could not read arm_length_place_backward in robot_visualize");
	if (!n_params.getParam("arm_length_place_switch", arm_length_place_switch))
        ROS_ERROR("Could not read arm_length_place_switch in robot_visualize");
	if (!n_params.getParam("arm_length_intake", arm_length_intake))
        ROS_ERROR("could not read arm_length_intake in robot_visualize");
	if (!n_params.getParam("arm_pivot_position", arm_pivot_position))
        ROS_ERROR("Could not read arm_pivot_position in robot_visualize");	

	arm_pub = n.advertise<geometry_msgs::PolygonStamped>("arm_state_viz", 10);
	intake_pub = n.advertise<geometry_msgs::PolygonStamped>("intake_state_viz", 10);
	robot_pub = n.advertise<geometry_msgs::PolygonStamped>("robot_state_viz", 10);

	robot_state_sub = n.subscribe("/frcrobot/robot_viz_state", 1, &plot_robot_cb);


	intake_polys.resize(2);
	arm_polys.resize(4);
	
	for(size_t i = 0; i < arm_polys.size(); i++)
	{		
		arm_polys[i].header.frame_id = "/robot_viz";
		int size = i < 1 ? 5 : 9;
		arm_polys[i].polygon.points.resize(size);
		arm_polys[i].polygon.points[0].y = arm_pivot_position;
		arm_polys[i].polygon.points[0].x = .0125;
		arm_polys[i].polygon.points[1].y = arm_pivot_position;
		arm_polys[i].polygon.points[1].x = -.0125;
		arm_polys[i].polygon.points[size - 1].y = arm_pivot_position;
		arm_polys[i].polygon.points[size - 1].x = .0125;
	}
	for(size_t i = 0; i < intake_polys.size(); i++)
	{		
		intake_polys[i].header.frame_id = "/robot_viz";
		intake_polys[i].polygon.points.resize(9);
		intake_polys[i].polygon.points[0].y = intake_position;
		intake_polys[i].polygon.points[0].x = intake_width / 2;
		intake_polys[i].polygon.points[1].y = intake_position;
		intake_polys[i].polygon.points[1].x = -intake_width / 2;
		intake_polys[i].polygon.points[4].y = intake_position + .03;
		intake_polys[i].polygon.points[4].x = -intake_width / 2.05;
		intake_polys[i].polygon.points[5].y = intake_position + .03;
		intake_polys[i].polygon.points[5].x = intake_width / 2.05;
		intake_polys[i].polygon.points[8].y = intake_position;
		intake_polys[i].polygon.points[8].x = intake_width / 2;
	}
	robot_poly.header.frame_id = "/robot_viz";
	robot_poly.polygon.points.resize(5);
	
	robot_poly.polygon.points[0].x = -width/2.0;
	robot_poly.polygon.points[0].y = -length/2.0 + wheel_base_pos_offset;
	robot_poly.polygon.points[0].z = 0.0;
	robot_poly.polygon.points[1].x = -width/2.0;
	robot_poly.polygon.points[1].y = length/2.0+ wheel_base_pos_offset;
	robot_poly.polygon.points[1].z = 0.0;
	robot_poly.polygon.points[2].x = width/2.0;
	robot_poly.polygon.points[2].y = length/2.0+ wheel_base_pos_offset;
	robot_poly.polygon.points[2].z = 0.0;
	robot_poly.polygon.points[3].x = width/2.0;
	robot_poly.polygon.points[3].y = -length/2.0+ wheel_base_pos_offset;
	robot_poly.polygon.points[3].z = 0.0;
	robot_poly.polygon.points[4].x = -width/2.0;
	robot_poly.polygon.points[4].y = -length/2.0+ wheel_base_pos_offset;
	robot_poly.polygon.points[4].z = 0.0;

	arm_polys[0].polygon.points[2].x = -.0125;
	arm_polys[0].polygon.points[2].y = arm_length_intake + arm_pivot_position;
	arm_polys[0].polygon.points[3].x = .0125;
	arm_polys[0].polygon.points[3].y = arm_length_intake + arm_pivot_position;
	
	
	arm_polys[1].polygon.points[2].x = -.0125;
	arm_polys[1].polygon.points[2].y = arm_length_place_forward + arm_pivot_position;
	arm_polys[1].polygon.points[3].x = -cube / 2;
	arm_polys[1].polygon.points[3].y = arm_length_place_forward + arm_pivot_position;
	arm_polys[1].polygon.points[4].x = -cube / 2;
	arm_polys[1].polygon.points[4].y = arm_length_place_forward + arm_pivot_position + cube;
	arm_polys[1].polygon.points[5].x = cube / 2;
	arm_polys[1].polygon.points[5].y = arm_length_place_forward + arm_pivot_position + cube;
	arm_polys[1].polygon.points[6].x = cube / 2;
	arm_polys[1].polygon.points[6].y = arm_length_place_forward + arm_pivot_position;
	arm_polys[1].polygon.points[7].x = .0125;
	arm_polys[1].polygon.points[7].y = arm_length_place_forward + arm_pivot_position;


	arm_polys[2].polygon.points[2].x = -.0125;
	arm_polys[2].polygon.points[2].y = arm_length_place_backward + arm_pivot_position;
	arm_polys[2].polygon.points[3].x = -cube / 2;
	arm_polys[2].polygon.points[3].y = arm_length_place_backward + arm_pivot_position;
	arm_polys[2].polygon.points[4].x = -cube / 2;
	arm_polys[2].polygon.points[4].y = arm_length_place_backward + arm_pivot_position - cube;
	arm_polys[2].polygon.points[5].x = cube / 2;
	arm_polys[2].polygon.points[5].y = arm_length_place_backward + arm_pivot_position - cube;
	arm_polys[2].polygon.points[6].x = cube / 2;
	arm_polys[2].polygon.points[6].y = arm_length_place_backward + arm_pivot_position;
	arm_polys[2].polygon.points[7].x = .0125;
	arm_polys[2].polygon.points[7].y = arm_length_place_backward + arm_pivot_position;

	arm_polys[3].polygon.points[2].x = -.0125;
	arm_polys[3].polygon.points[2].y = arm_length_place_switch + arm_pivot_position;
	arm_polys[3].polygon.points[3].x = -cube / 2;
	arm_polys[3].polygon.points[3].y = arm_length_place_switch + arm_pivot_position;
	arm_polys[3].polygon.points[4].x = -cube / 2;
	arm_polys[3].polygon.points[4].y = arm_length_place_switch + arm_pivot_position + cube;
	arm_polys[3].polygon.points[5].x = cube / 2;
	arm_polys[3].polygon.points[5].y = arm_length_place_switch + arm_pivot_position + cube;
	arm_polys[3].polygon.points[6].x = cube / 2;
	arm_polys[3].polygon.points[6].y = arm_length_place_switch + arm_pivot_position;
	arm_polys[3].polygon.points[7].x = .0125;
	arm_polys[3].polygon.points[7].y = arm_length_place_switch + arm_pivot_position;
	
	intake_polys[0].polygon.points[2].x = cos(intake_angle_in)*intake_length - intake_width / 2;	
	intake_polys[0].polygon.points[2].y = sin(intake_angle_in)*intake_length + intake_position;	
	intake_polys[0].polygon.points[3].x = cos(intake_angle_in)*intake_length - intake_width / 2 + .03;	
	intake_polys[0].polygon.points[3].y = sin(intake_angle_in)*intake_length + intake_position;	
	intake_polys[0].polygon.points[6].x = -cos(intake_angle_in)*intake_length + intake_width / 2 - .03;	
	intake_polys[0].polygon.points[6].y = sin(intake_angle_in)*intake_length + intake_position;	
	intake_polys[0].polygon.points[7].x = -cos(intake_angle_in)*intake_length + intake_width / 2;	
	intake_polys[0].polygon.points[7].y = sin(intake_angle_in)*intake_length + intake_position;	

	
	intake_polys[1].polygon.points[2].x = cos(intake_angle_out)*intake_length - intake_width / 2;	
	intake_polys[1].polygon.points[2].y = sin(intake_angle_out)*intake_length + intake_position;	
	intake_polys[1].polygon.points[3].x = cos(intake_angle_out)*intake_length - intake_width / 2 + .03;	
	intake_polys[1].polygon.points[3].y = sin(intake_angle_out)*intake_length + intake_position;	
	intake_polys[1].polygon.points[6].x = -cos(intake_angle_out)*intake_length + intake_width / 2 - .03;	
	intake_polys[1].polygon.points[6].y = sin(intake_angle_out)*intake_length + intake_position;	
	intake_polys[1].polygon.points[7].x = -cos(intake_angle_out)*intake_length + intake_width / 2;	
	intake_polys[1].polygon.points[7].y = sin(intake_angle_out)*intake_length + intake_position;	

	ros::spin();
}

void rotate_polygon(geometry_msgs::Polygon &poly, double angle)
{
	for( int i = 0; i < poly.points.size(); i++)
	{
		Eigen::Rotation2Dd r(angle);
		Eigen::Vector2d point = {poly.points[i].x, poly.points[i].y};
		point = r.toRotationMatrix() * point;
		poly.points[i].x = point[0];
		poly.points[i].y = point[1];
	}
}

void translate_polygon(geometry_msgs::Polygon &poly, double x, double y)
{
	for( int i = 0; i < poly.points.size(); i++)
	{
		poly.points[i].x = poly.points[i].x + x;
		poly.points[i].y = poly.points[i].y + y;
	}
}



void plot_robot_cb(const robot_visualizer::RobotVisualizeState &robot_viz)
{	
	std::vector<geometry_msgs::PolygonStamped> pub_polys;	
	pub_polys.push_back(robot_poly);
	pub_polys.push_back(arm_polys[robot_viz.arm_pos]);
	pub_polys.push_back(intake_polys[robot_viz.intake_pos]);

	for(size_t i = 0; i < pub_polys.size(); i++)
	{
		rotate_polygon(pub_polys[i].polygon, robot_viz.theta);
		translate_polygon(pub_polys[i].polygon, robot_viz.x, robot_viz.y);

		pub_polys[i].header.stamp = ros::Time::now();
	}
	robot_pub.publish(pub_polys[0]);
	arm_pub.publish(pub_polys[1]);
	intake_pub.publish(pub_polys[2]);
}
