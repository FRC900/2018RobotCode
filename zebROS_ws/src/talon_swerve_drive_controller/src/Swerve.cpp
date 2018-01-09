#include <talon_swerve_drive_controller/Swerve.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <iomanip>	
#include <functional>
#include <cmath>
using namespace std;
using namespace Eigen;

swerve::swerve(array<Vector2d, WHEELCOUNT> _wheelCoordinates, string _fileAddress, bool _wheelAngleInvert, ratios _ratio, encoderUnits _units, driveModel _drive)
{
	wheelCoordinates = _wheelCoordinates;

	swerveMath = swerveDriveMath(wheelCoordinates);

	ratio = _ratio;
	units = _units;
	drive = _drive;
	fileAddress = _fileAddress;
	for(int i =0; i < WHEELCOUNT; i++)
	{
		getWheelAngle(i);
	}
	wheelAngleInvert = _wheelAngleInvert ? -1 : 1;
	ifstream offsetRead;
	offsetRead.open(fileAddress);
	if(!offsetRead)
	{
		cout<< "No Offset File!!!" << endl;
		array<double, WHEELCOUNT> darray; 
                for(int i = 0; i < WHEELCOUNT; i++) 
                { 
                        offsets[i] = 0; 
                }
	}
	else
	{
		double offset;
		int i = 0;
		while(offsetRead >> offset)
		{
			offsets[i] = offset;
			i++;
		}
	}

};
array<Vector2d, WHEELCOUNT> swerve::motorOutputs(Vector2d velocityVector, double rotation, double angle, bool forceRead, array<bool, WHEELCOUNT> &reverses, bool park, array<double, WHEELCOUNT> positionsNew, int rotationCenterID, bool overrideID, Vector2d centerOfRotation)
{
	encoderPosition = positionsNew;
        array<Vector2d, WHEELCOUNT> speedsAndAngles;
	if(!park)
	{
		velocityVector/=drive.maxSpeed;

		if(overrideID)
		{
			multiplierSet newSet;
			newSet.multipliers = swerveMath.wheelMultipliersXY(centerOfRotation);
			newSet.maxRotRate = furthestWheel(centerOfRotation)/drive.maxSpeed;	
			multiplierSets[rotationCenterID] = newSet;
		}
		rotation/=multiplierSets[rotationCenterID].maxRotRate;
		speedsAndAngles = swerveMath.wheelSpeedsAngles(multiplierSets[rotationCenterID].multipliers, velocityVector, rotation, angle);
		for(int i = 0; i < WHEELCOUNT; i++)
		{
			double nearestangle;
			bool reverse;
				getWheelAngle(i);
				nearestangle = leastDistantAngleWithinHalfPi(encoderPosition[i], speedsAndAngles[i][1], reverse);
			reverses[i] = reverse;
			speedsAndAngles[i][0]*=((drive.maxSpeed/(drive.wheelRadius*2*M_PI))/ratio.encodertoRotations)*units.rotationSetV* (reverse ? -1 : 1);
			speedsAndAngles[i][1] = (nearestangle/(2*M_PI))*units.steeringSet - offsets[i];
		}
	}
	else
	{	
		for(int i = 0; i < WHEELCOUNT; i++)
		{
			speedsAndAngles[i][1] = swerveMath.parkingAngle[i];
			speedsAndAngles[i][0] = 0;
			
			double nearestangle;
			bool reverse;
			getWheelAngle(i);
			nearestangle = leastDistantAngleWithinHalfPi(encoderPosition[i], speedsAndAngles[i][1], reverse);
			speedsAndAngles[i][1] = (nearestangle/(2*M_PI))*units.steeringSet - offsets[i];
		}

	}
	return speedsAndAngles;
}
void swerve::saveNewOffsets(bool useVals, array<double, WHEELCOUNT> newOffsets, array<double, WHEELCOUNT> newPosition)
{
	encoderPosition = newPosition;
	if(!useVals)
	{
		for(int i = 0; i < WHEELCOUNT; i++)
		{
			newOffsets[i] = encoderPosition[i];
		}
	}
	offsets = newOffsets;
	ofstream offsetFile;
	offsetFile.open(fileAddress);
	if(offsetFile)
	{
		offsetFile.close();
		offsetFile.open(fileAddress, ios::out | ios::trunc);

	}
	for(int i = 0; i < WHEELCOUNT; i++)
	{
		offsetFile << offsets[i] << endl;
	}
	offsetFile.close();
}
/*
Vector2d calculateOdom()
{

//Steal code from steered wheel base

}
*/

double swerve::getWheelAngle(int index)
{
	savedEncoderVals[index] = (encoderPosition[index]+offsets[index])*units.steeringGet*2*M_PI*wheelAngleInvert;
	return savedEncoderVals[index];
}

double swerve::furthestWheel(Vector2d centerOfRotation)
{
	double maxD = 0;
	for(int i = 0; i < WHEELCOUNT; i++)
	{
		double dist = sqrt(pow(wheelCoordinates[i][0] - centerOfRotation[0], 2) + pow(wheelCoordinates[i][1] - centerOfRotation[1], 2));
		if(dist>maxD){maxD=dist;}
	}
	return maxD;
}
