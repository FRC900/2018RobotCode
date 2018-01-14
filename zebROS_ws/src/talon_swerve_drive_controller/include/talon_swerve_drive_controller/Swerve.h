#ifndef INC_SWERVE
#define INC_SWERVE

#include <vector>
#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "SwerveMath.h"

//meters, radians, newtons, kg
//This class will need access to the swerve drive talons
//Gets should be rotations/encoder unit
//Sets should be encoder unit/rotation (or rotations/second)

namespace swerveVar
{
struct ratios
{
	double encodertoRotations;
	double motortoRotations;
	double motortoSteering;
};

struct encoderUnits
{
	double steeringGet;
	double steeringSet;
	double rotationGetV;
	double rotationGetP;
	double rotationSetV;
	double rotationSetP;
};
struct driveModel
{
	double maxSpeed;
	double wheelRadius;
	double mass;
	double motorFreeSpeed;
	double motorStallTorque;
	int motorQuantity;
	double speedLossConstant = .81; // Don't set this here
}; //more info should be added to this struct
}

#define WHEELCOUNT 4
// TODO : remove WHEELCOUNT, use vectors
class swerve
{
	public:
		swerve(std::array<Eigen::Vector2d, WHEELCOUNT> wheelCoordinates, std::vector<double> offsets, bool wheelAngleInvert, swerveVar::ratios ratio, swerveVar::encoderUnits units, swerveVar::driveModel drive);

		std::array<Eigen::Vector2d, WHEELCOUNT> motorOutputs(Eigen::Vector2d velocityVector, double rotation, double angle, bool forceRead, std::array<bool, WHEELCOUNT> &reverses, bool park, std::array<double, WHEELCOUNT> positionsNew, int rotationCenterID = 0);
		//for non field centric drive set angle = pi/2
		//if rotationCenterID == 0 we will use the base center of rotation
		void saveNewOffsets(bool useVals, std::array<double, WHEELCOUNT> newOffsets, std::array<double, WHEELCOUNT> newPosition); //should these be doubles?
		//Note that unless you pass vals in and set useVals to true, it will use the current wheel positions, wheels should be pointing to the right.
		//Eigen::Vector2d currentOdom;
		//Eigen::Vector2d calculateOdom(); //might be some associated private variables
		//Probably should be called every

		std::array<Eigen::Vector2d, WHEELCOUNT> wheelCoordinates_;
		swerveDriveMath swerveMath_; //this should be public

		double getWheelAngle(int index, double pos) const;
	private:
		//should we get them together instead?
		//the angle it passes out isn't normalized
		double furthestWheel(Eigen::Vector2d centerOfRotation) const;

		void setCenterOfRotation(int ID, const Eigen::Vector2d &centerOfRotation);

		std::array<double, WHEELCOUNT> encoderPosition_;
		std::array<double, WHEELCOUNT> offsets_; //Should these be doubles?
		//Second piece of data is here just for physics/modeling

		//std::array<double, WHEELCOUNT> savedEncoderVals_;
		int8_t wheelAngleInvert_;

		struct multiplierSet
		{
			std::array<Eigen::Vector2d, WHEELCOUNT> multipliers_;
			double maxRotRate_;
		};
		std::array<multiplierSet, 63> multiplierSets_;
		swerveVar::ratios ratio_;
		swerveVar::encoderUnits units_;
		swerveVar::	driveModel drive_;
};
#endif
