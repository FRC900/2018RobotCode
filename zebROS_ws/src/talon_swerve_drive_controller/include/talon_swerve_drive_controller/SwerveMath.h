#ifndef INC_SWERVE_MATH
#define INC_SWERVE_MATH
#include <vector>
#include <array>
#include <math.h>
#include <stdlib.h>
#include "900Math.h"
#include <Eigen/Dense>

class swerveDriveMath
{
#define WHEELCOUNT 4
		//WHEELCOUNT is defined so arrays can be used.
		//Arrays are used over vectors for optimization, which may be invalid
		//There is likely a better way to handle WHEELCOUNT
	public:
		swerveDriveMath(std::array<Eigen::Vector2d, WHEELCOUNT> _wheelCoordinate);
		swerveDriveMath() {};
		std::array<Eigen::Vector2d, WHEELCOUNT> wheelMultipliersXY(Eigen::Vector2d rotationCenter);


		std::array<Eigen::Vector2d, WHEELCOUNT> wheelSpeedsAngles(std::array<Eigen::Vector2d, WHEELCOUNT> wheelMultipliersXY, Eigen::Vector2d velocityVector, double rotation, double angle); //for non field centric set angle to pi/2



		//Variables which need to be used externally
		std::array<double, WHEELCOUNT> parkingAngle;
		std::array<Eigen::Vector2d, WHEELCOUNT> baseWheelMultipliersXY;
		//Wheel multipliers would need to be rerun if wheels somehow get moved around



	private:
		std::array<double, WHEELCOUNT> parkingAngles();
		//only must be run once to determine the angles of the wheels in parking config

		//movement wheelAverage(std::array<Eigen::Vector2d, WHEELCOUNT> wheelMove, double angle, bool rotation);

		//All variables here which don't need to be accessed externally
		std::array<Eigen::Vector2d, WHEELCOUNT> wheelCoordinate;
		std::array<double, WHEELCOUNT> normalize(std::array<double, WHEELCOUNT> input);
		struct movement
		{
			Eigen::Vector2d translation;
			double rotation;
		};


};
#endif
