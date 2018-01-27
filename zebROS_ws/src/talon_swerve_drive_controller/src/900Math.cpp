#include <math.h>
#include "talon_swerve_drive_controller/900Math.h"

/*
//Run once somewhere
//The following is code to generate a limited square root look up table for improved performance.
float lookup_table[2000];
for(int i = 0; i < sizeof(lookup_table)/sizeof(lookup_table[0]); i++) {
	float num = i/1000.0;
	float sqrt_num = sqrt(num);
	lookup_table[i] = sqrt_num;
}
*/

/*
double pythaguptotwo(double a, double b)
{
	return lookup_table[a*a+b*b];
}
*/

double sign(double number)
{
	return (number > 0) ? 1 : ((number < 0) ? -1 : 0);
}

// KCJ - if you're returning more than 1 thing, probably best to
// return them both via reference.
// For more fun, though, create a type which has a double plus a bool
// in it.  Return that type.
// Maybe use it for both of the leastDistant
// calls - set as is here and unconditionally set to false in the other
// function below?  Code that uses it can query the reverse bool var
// without worring about which function it called, maybe?

double leastDistantAngleWithinHalfPi(double currentAngle, double targetAngle, bool &reverse)
{
	//returns the closest angle to the current angle = to x*.5*M_PI + target angle where x is any integer
	//used for turning wheels to the target angle (swerve)
	double normalizedDiff = normalizeAngle(targetAngle) - normalizeAngle(currentAngle);

	double withinPi = (fabs(normalizedDiff) < M_PI) ? normalizedDiff : (normalizedDiff - (2 * M_PI * sign(normalizedDiff)));
	double withinHalfPi;

	if (fabs(withinPi) < (M_PI / 2) )
	{
		withinHalfPi =  withinPi;
		reverse = false;
	}
	else
	{
		withinHalfPi = (withinPi - sign(withinPi) * M_PI);
		reverse = true;
	}
	return withinHalfPi + currentAngle;
}

double leastDistantAngleWithinPi(double currentAngle, double targetAngle)
{
	double normalizedDiff = normalizeAngle(targetAngle) - normalizeAngle(currentAngle);
	double withinPi = (fabs(normalizedDiff) < M_PI) ? normalizedDiff : (normalizedDiff - (2 * M_PI * sign(normalizedDiff)));
	return withinPi + currentAngle;
}

double normalizeAngle(double angle) //normalizes between -M_PI and M_PI
{
	return angle - floor((angle + M_PI) / (2 * M_PI)) * 2.0 * M_PI;
}

double coerce(double value, double lowerBound, double upperBound)
{
	return (value < lowerBound) ? lowerBound : (value > upperBound) ? upperBound : value;
}

