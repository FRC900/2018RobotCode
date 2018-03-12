#ifndef INC_900_MATH
#define INC_900_MATH
#include <array>

//double pythaguptotwo(double a, double b);

double leastDistantAngleWithinHalfPi(double currentAngle, double targetAngle, bool &reverse);
double normalizeAngle(double angle);
double leastDistantAngleWithinPi(double currentAngle, double targetAngle);
double coerce(double value, double lowerBound, double upperBound);
double sign(double number);

#endif
