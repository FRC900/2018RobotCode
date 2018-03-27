#include <math.h>
#include <array>
#include <swerve_math/SwerveMath.h>
#include <ros/ros.h>
#include <ros/console.h>

using namespace std;

// TODO : use intializer list
// Make arg const &
swerveDriveMath::swerveDriveMath(array<Eigen::Vector2d, WHEELCOUNT> wheelCoordinate)
{
	wheelCoordinate_ = wheelCoordinate;
	parkingAngle_ = parkingAngles();
	//The below coordinate pair can potentially change, it is relative to the wheel coordinates
	Eigen::Vector2d baseRotationCenter = {0, 0};
	baseWheelMultipliersXY_ = wheelMultipliersXY(baseRotationCenter);
}

//used for varying center of rotation and must be run once for initialization
array<Eigen::Vector2d, WHEELCOUNT> swerveDriveMath::wheelMultipliersXY(const Eigen::Vector2d &rotationCenter)
{
	array<double, WHEELCOUNT> wheelAngles;
	array<double, WHEELCOUNT> wheelMultipliers;
	//int size =  wheelCoordinate::size;
	for (int i = 0; i < WHEELCOUNT; i++) //increment for each wheel
	{
		double x = wheelCoordinate_[i][0] - rotationCenter[0];
		double y = wheelCoordinate_[i][1] - rotationCenter[1];
		wheelMultipliers[i] = -1;//-sqrt(x * x + y * y); // TODO : use hypot function
		wheelAngles[i] = atan2(x, y) + .5 * M_PI;
	}
	wheelMultipliers = normalize(wheelMultipliers);
	array<Eigen::Vector2d, WHEELCOUNT> multipliersXY;
	for (int i = 0; i < WHEELCOUNT; i++)
	{
		multipliersXY[i][0] = wheelMultipliers[i] * cos(wheelAngles[i]);
		multipliersXY[i][1] = wheelMultipliers[i] * sin(wheelAngles[i]);
	}
	return multipliersXY; //change to array
}
//Below function calculates wheel speeds and angles for some target rotation and translation velocity
//Rotation is positive counter clockwise
//Angle is the angle of the gyro for field centric driving
//In radians, 0 is horizontal, increases counterclockwise
//For non field centric set angle to pi/2
// TODO : pass array as const & argument
array<Eigen::Vector2d, WHEELCOUNT> swerveDriveMath::wheelSpeedsAngles(const array<Eigen::Vector2d, WHEELCOUNT> &wheelMultipliersXY, const Eigen::Vector2d &velocityVector, double rotation, double angle, bool norm) const
{
	/*if (rotation == 0 && velocityVector[0] == 0 && velocityVector[1] == 0)
	{
	return
	}
	*/

	//Parking config isn't handled here
	//The code commented out above is an outline of how it could be handled here.
	//Only call function if the robot should be moving.

	//Rotate the target velocity by the robots angle to make it field centric
	Eigen::Rotation2Dd r(M_PI / 2 - angle);
	Eigen::Vector2d rotatedVelocity = r.toRotationMatrix() * velocityVector;
	//Should this instead be a function in 900Math of the form: rotate(vector, angle) rather than 2 lines of eigen stuff?
	array<double, WHEELCOUNT> speeds;
	array<double, WHEELCOUNT> angles;
	//Sum cartisian velocity for each wheel and then convert to polar coordinates

	for (int i = 0; i < WHEELCOUNT; i++)
	{
		//int inverterD = (i%2==0) ? -1 : 1;
		//Only the rotation of the robot differently effects each wheel
		const double x = wheelMultipliersXY[i][0] * rotation + rotatedVelocity[0];
		const double y = wheelMultipliersXY[i][1] * rotation - rotatedVelocity[1];
		//ROS_INFO_STREAM("rot: " << rotation << " wheel_multipliers_x: " << wheelMultipliersXY[i][0]<< " wheel_multipliers_y " << wheelMultipliersXY[i][1]);
		angles[i] = atan2(x, y);
		speeds[i] = hypot(x, y);
	}
	if(norm)
	{
		speeds = normalize(speeds);
	}
	//Speed and angles are put into one array here because speeds needed to be normalized
	array<Eigen::Vector2d, WHEELCOUNT> speedsAngles;
	for (int i = 0; i < WHEELCOUNT; i++)
	{
		speedsAngles[i][0] = speeds[i];
		speedsAngles[i][1] = angles[i];
	}
	return speedsAngles;
}
array<double, WHEELCOUNT> swerveDriveMath::parkingAngles(void) const
{
	//only must be run once to determine the angles of the wheels in parking config
	array<double, WHEELCOUNT> angles;
	for (size_t i = 0; i < wheelCoordinate_.size(); i++)
	{
		angles[i] = atan2(wheelCoordinate_[i][0], wheelCoordinate_[i][1]);
	}
	return angles;
}

// TODO : modify input arg rather than returning
// a new array.  Change input to & arg, return type to void,
// modfiy input rather than normalzied in the last loop,
// remove the else statement
array<double, WHEELCOUNT> swerveDriveMath::normalize(const array<double, WHEELCOUNT> &input) const
{
	//Note that this function only works on arrays of size WHEELCOUNT
	double maxi = *max_element(input.begin(), input.end());
	double mini = *min_element(input.begin(), input.end());
	double absoluteMax;
	if ( abs(maxi) > abs(mini) )
	{
		absoluteMax = abs(maxi);
	}
	else
	{
		absoluteMax = abs(mini);
	}
	if (absoluteMax > 1)
	{
		array<double, WHEELCOUNT> normalized;
		for (size_t i = 0; i < input.size(); i++)
		{
			normalized[i] = input[i] / absoluteMax;
		}
		return normalized;
	}
	else
	{
		return input;
	}
}

//odometry/foward kinematic functions below, TODO, use ROS function
/*
swerveDriveMath::movement swerveDriveMath::wheelAverage(array<Eigen::Vector2d, WHEELCOUNT> wheelMove, double angle, bool rotation)
{
	Eigen::Vector2d avgMove = (wheelMove[0] +  wheelMove[1] +  wheelMove[2] +  wheelMove[3])/4;

	Eigen::Rotation2Dd r(angle - M_PI/2);
	Eigen::Vector2d rotatedMove = r.toRotationMatrix()*avgMove; //Should this instead be a function in 900Math of the form: rotate(vector, angle) rather than 2 lines of eigen stuff?
	double dRotation;
	if(rotation)
	{
		//TODO: put code here to calculate rotation
	}
	else
	{
		dRotation  = 0;
	}
	movement delta;
	delta.translation = rotatedMove;
	delta.rotation = dRotation;
	return delta;

}
*/
/*
movement threeWheelAvg( array<Eigen::Vector2d, WHEELCOUNT> wheelMove, double angle, bool rotation?)
{
	//use horizontally adjacent wheels somehow?, needs to be generalized
	//Is this needed?
}
*/
