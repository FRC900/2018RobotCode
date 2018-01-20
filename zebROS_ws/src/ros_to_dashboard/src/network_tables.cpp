#include <WPILib.h>
#include <NetworkTables/NetworkTables.h>


class RobotDemo : public SimpleRobot
{
public:
	NetworkTable *table;

	RobotDemo(void)
	{
		table = NetworkTable::GetTable("DB/String 0");
	}

	void OperatorControl (void)
	{
		double x = 0;
		double y = 0;
		while (IsOperatorControl() && IsEnabled())
		{
			Wait(1.0);
			table->PutNumber("X", x);
			table->PutNumber("Y", y);
			x += 0.25;
			y += 0.25;
		}
	}
};

START_ROBOT_CLASS(RobotDemo);
