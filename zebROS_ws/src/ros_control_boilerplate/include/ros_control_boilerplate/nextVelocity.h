#ifndef INC_NEXT_VELOCITY
#define INC_NEXT_VELOCITY
#include <math.h>

int sign(double n)
{
	if(n > 0)
	{
		return 1;
	}
	else
	{
		return -1;
	}
}

double nextVelocity(double v, double vt, double &a, double am, double jm, double cr) 
{
	
	double diffv = vt - v;
	int signDiffV = sign(diffv);
	int signA = sign(a);
	double nV;
	bool finish;
	if(pow(a, 2)/jm >= diffv*signA) //Fix tendency to overshoot?
	{
		//overshoot
		if(((diffv/cr) -(a)/2)*signDiffV <= 0 && (a - (signA * jm * cr)/2)*signA < 0) //verify and fix
		{
			//cout<<"finish"<<endl;
			a = (diffv/cr); //if we can get to target now, do it
			finish = true;
		}
		else
		{
			a = (a -  signA * jm * cr);
			finish = false;
			//cout<<"return from peak accel"<<endl;
		}										 
	}
	else
	{
		//upswing or flat	
		a = (a + signDiffV * jm * cr);
		if(abs(a) > am)
		{
			a = signDiffV*am;
			finish = false;
		}
		if(abs(a*cr) > abs(diffv)) //verify and fix
		{
			a = (diffv/cr); //if we can get to target now, do it
			finish =true;
		}	
	}
	nV = v + a * cr;
	if(finish) {a=0;} 
	return nV;


}
#endif
