#ifndef INC_SWERVE_MATH
#define INC_SWERVE_MATH
#include <array>
#include <vector>
#include "900Math.h"
#include <Eigen/Dense>

class swerveDriveMath
{
#define WHEELCOUNT 4
		//WHEELCOUNT is defined so arrays can be used.
		//Arrays are used over vectors for optimization, which may be invalid
		//There is likely a better way to handle WHEELCOUNT
		//  TODO : yes, use vectors
	public:
		swerveDriveMath(std::array<Eigen::Vector2d, WHEELCOUNT> wheelCoordinate);
		swerveDriveMath() {};
		std::array<Eigen::Vector2d, WHEELCOUNT> wheelMultipliersXY(const Eigen::Vector2d &rotationCenter);

		std::array<Eigen::Vector2d, WHEELCOUNT> wheelSpeedsAngles(const std::array<Eigen::Vector2d, WHEELCOUNT> &wheelMultipliersXY, const Eigen::Vector2d &velocityVector, double rotation, double angle, bool norm) const; //for non field centric set angle to pi/2

		//Variables which need to be used externally
		std::array<double, WHEELCOUNT> parkingAngle_;
		std::array<Eigen::Vector2d, WHEELCOUNT> baseWheelMultipliersXY_;
		//Wheel multipliers would need to be rerun if wheels somehow get moved around

	private:
		std::array<double, WHEELCOUNT> parkingAngles(void) const;
		//only must be run once to determine the angles of the wheels in parking config

		//movement wheelAverage(std::array<Eigen::Vector2d, WHEELCOUNT> wheelMove, double angle, bool rotation);

		//All variables here which don't need to be accessed externally
		std::array<Eigen::Vector2d, WHEELCOUNT> wheelCoordinate_;
		std::array<double, WHEELCOUNT> normalize(const std::array<double, WHEELCOUNT> &input) const;
		struct movement
		{
			Eigen::Vector2d translation;
			double rotation;
		};
};
#endif
