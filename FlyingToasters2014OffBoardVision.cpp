#include "WPILib.h"
#include "NetworkTables/NetworkTable.h"
#include "Math.h"

class FlyingToasters2014OffBoardVision : public SimpleRobot
{
	//Structure to represent the scores for the various tests used for target identification
	struct Scores {
		double rectangularity;
		double aspectRatioVertical;
		double aspectRatioHorizontal;
	};
	
	struct TargetReport {
		int verticalIndex;
		int horizontalIndex;
		bool Hot;
		double totalScore;
		double leftScore;
		double rightScore;
		double tapeWidthScore;
		double verticalScore;
	};
	//Sensors
	AnalogChannel RangeFinder;
	AnalogChannel Potentiometer;
	//Actuators
	RobotDrive myRobot; // robot drive system
	Joystick Driver; //Logitech Game-pad
	Joystick Operator;
	Talon Arm;
	Talon Launcher;
	Solenoid ShiftersHigh;
	Solenoid ShiftersLow;
	Solenoid PlungerIn;
	Solenoid PlungerOut;
	Solenoid ClawIn;
	Solenoid ClawOut;
	Compressor c;


public:
	NetworkTable *table;
	FlyingToasters2014OffBoardVision(void):
		RangeFinder(1),
		Potentiometer(2),
		myRobot(1, 2),	
		Driver(1),
		Operator(2), 
		Arm(3),
		Launcher(4),
		ShiftersHigh(1),
		ShiftersLow(2),
		PlungerIn(3),
		PlungerOut(4),
		ClawIn(5),
		ClawOut(6),
		c(1,1)
	{
		table = NetworkTable::GetTable("datatable");
		myRobot.SetExpiration(0.1);
		myRobot.SetSafetyEnabled(false);
	}

	/**
	 * Image processing code to identify 2014 Vision targets
	 */
	void Autonomous(void)
	{
		c.Start();
		double range = RangeFinder.GetAverageVoltage()/0.0248158; //range is in inches
		int PistonCounter = 0;
		bool hot = false;
		DriverStationLCD *ds = DriverStationLCD::GetInstance();
		
		while (IsAutonomous() && IsEnabled()) {	
			table->GetBoolean("Hot", hot);
			
			if (range >= 75)
			{
				myRobot.TankDrive(0.5, 0.5);
			}
			if (range < 75 && range > 28)
			{
				myRobot.TankDrive(0.25, 0.25);
			}
			
			else
			{
				myRobot.TankDrive(0.0,0.0);
			}
			ds->UpdateLCD();
		}
			
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		c.Start();
		double range = RangeFinder.GetAverageVoltage()/0.0248158;
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			myRobot.TankDrive(Driver.GetRawAxis(2),Driver.GetRawAxis(4)); //Tank Drive contol for chassis
			Wait(0.005);				// wait for a motor update time
			
			//Arm control
			if (Potentiometer.GetAverageVoltage() >= 4.5 && Operator.GetRawAxis(1) > 0.0)
			{
				Arm.Set(0.0);
			}
			if (Potentiometer.GetAverageVoltage() <= 1 && Operator.GetRawAxis(1) < 0.0)
			{
				Arm.Set(0.0);
			}
			if (Potentiometer.GetAverageVoltage() < 2.75 && Operator.GetRawButton(3) == true)
			{
				Arm.Set(-0.75);
			}
			if (Potentiometer.GetAverageVoltage() > 2.25 && Operator.GetRawButton(3) == true)
			{
				Arm.Set(0.75);
			}
			else
			{
				Arm.Set(Operator.GetRawAxis(1));
			}
			Wait(0.005);
			
			//Plunger control
			if (Operator.GetRawButton(1) == true)
			{
				PlungerOut.Set(true);
				PlungerIn.Set(false);
				Wait(1.0);
				PlungerIn.Set(true);
				PlungerOut.Set(false);
			}
			else if (Operator.GetRawButton(1) != true)
			{
				PlungerIn.Set(true);
			}
		}
	}
	
};

START_ROBOT_CLASS(FlyingToasters2014OffBoardVision);
