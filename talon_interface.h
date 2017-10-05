// Basic idea for Talon wrapper code?
#pragma once

#include "talon includes"

// Base class. Encapsulates all the functions we'd need for running
// a talon
class TalonBase
{
	public:
		TalonBase(int id, whatever other params are used in all cases) :
			srx(id)
		{
			// create a CTRE talon object but don't configure it - that's done
			// in each derived class
		}
		Set(); // Set output in whatever mode the derived class set up
		Get(); // get encoder value
		SetSoftLimits(); // might or might not make sense here.
	protected:
		CanTalonSRX srx; // Actual CTRE talon object used to communicate via HW

};

class TalonVeclocityPID : public TalonBase
{
	TalonVelocityPID(int id, int p, int i, int d):
		TalonBase(id) // initialize common stuff in base class
	{
		// Use the initializaed srx object from the base class to
		// do other init
		srx.setMode(closed loop velocity);
		srx.setPID(p, i ,d);
	}
	// Set, Get() and whatever are handled by the default in the base class
};

class TalonVeclocityPIDF : public TalonVelocityPID
{
	TalonVelocityPIDF(int id, int p, int i, int d, int f):
		TalonVelocityPID(id, p, i, d) // initialize common stuff in base classes
	{
		// Use the initializaed srx object from the base class to
		// do other init
		srx.setMode(closed loop velocity with feed forward); // or might not be a separate mode, just PID where f is also set
		srx.setF(f);
	}
	// Set, Get() and whatever are handled by the default in the base class
	// Maybe override Set() to hack in a Ka term
};

// Do the same for 

using :

// A base class pointer is used to access any of the derived classes :

// This creates a TalonVelocityPID object with id=1, PID=10, .5, .1
TalonBase *talon = new TalonVelocityPid(1, 10, 5, .1);

talon->Set(.75); // calls the derived class Set() if it exists, the base class Set() otherwise

// This seems dumb, but you can do something like 
// to create an array of motor controllers
vector<TalonBase *> talons;

talons.push_back(new TalonVelotalcityPID(1, 10, .5, .2));
talons.push_back(new TalonPositionPID(blah));
talons.push_back(new TalonVelocityPID(blah));
talons.push_back(new TalonPositionPID(blah));
talons.push_back(new TalonVelocityPID(blah));
talons.push_back(new TalonPositionPID(blah));
talons.push_back(new TalonVelocityPID(blah));
talons.push_back(new TalonPositionPID(blah));

double motorSetpoints[talons.size()];

// other code sets motorSetpoint each iteration
// If each separate controller only hits one 
// array entry the controller library code can
// be multithreaded?

// write computed setpoints to motors 
for (size_t i = 0; i < talons.size(); i++)
	talons[i].Set(motorSetpoints[i]);
