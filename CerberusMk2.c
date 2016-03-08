#pragma config(Sensor, dgtl1,  Flywheel, sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightEncoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  leftEncoder,    sensorQuadEncoder)
#pragma config(Motor,  port1,           ce,            tmotorVex393_HBridge, openLoop)//6 motor launch
#pragma config(Motor,  port2,           rb,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           er,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           us,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           FRight,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           BRight,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           BLeft,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           FLeft,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           FeedMe,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          seymore,       tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

float Kp = 0; //Default to 2
int Error = 0; //How wrong I am
float Ki  = 0; //Default to 0.001
float Integral = 0; //How wrong I've been
float Kd = 0; //Default to 1.7
int DeltaE = 0;
int power = 0;
float Flyspeed = 0;
int TargetSpeed;
int PrevError;
int setpower;
int Flypower = 0;
float Kps[5] = {0, 2.4, 56, 2.4, 2.4};
float Kis[5] = {0, 0.001, 0.01, 0.001, 0.001};
float Kds[5] = {0, 2.5, 70, 2.5, 2.5};
int TargetSpeeds[5] = {0, 205, 245, 183, 191};//Off, Skillz, Long, 1st, 2nd
int setpowers[5] = {0, 70, 100, 64, 68};
int repeaters[5] = {127, 127, 90, 127, 127};
int repeater = 127;
TVexJoysticks buttons[5] = {Btn8D, Btn7U, Btn7R, Btn7D, Btn7L};
int n = 0;
float Kerror = 0;
float Kintegral = 0;
float KdeltaE = 0;
int PIDLoop = 0;
float AvgError = 0;
int Overshoot;
int SecondInt = 0;


//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!

/////////////////////////////////////////////////////////////////////////////////////////
//
//                          Pre-Autonomous Functions
//
// You may want to perform some actions before the competition starts. Do them in the
// following function.
//
/////////////////////////////////////////////////////////////////////////////////////////

void pre_auton()
{
	SensorValue[Flywheel] = 0;
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
  // Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Autonomous Task
//
// This task is used to control your robot during the autonomous phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task motorcontrol()
{
	while(1)
	{
		Flyspeed = abs(SensorValue[Flywheel]);
		Error = TargetSpeed - Flyspeed;
		DeltaE = Error - PrevError;
		Integral += Error;
		Kerror = Kp*Error;
		Kintegral = Ki*Integral;
		KdeltaE = Kd*DeltaE;
		Overshoot = TargetSpeed - DeltaE;
		PIDLoop = PIDLoop > 40 ? 1 : PIDLoop + 1;
		SecondInt = PIDLoop > 40 ? 1 : SecondInt + Error;
		AvgError = PIDLoop > 40 ? 1 : SecondInt/PIDLoop;
		power = setpower + Kerror + Kintegral + KdeltaE;
		SensorValue[Flywheel] = 0;
		PrevError = Error;
		wait1Msec(25);
	}
}

task autonomous()
{
	TargetSpeed = 950;
	setpower = 104;
	startTask(motorcontrol);
	motor[ce] = power;
	motor[rb] = power;
	motor[er] = power;
	motor[us] = power;
	wait1Msec(1000);
	clearTimer(T1);
	while(true)
	{
		if(abs(Error) < 60)
		{
			motor[FeedMe] = 127;
			motor[seymore] = 127;
		}
		else
		{
			motor[FeedMe] = 0;
			motor[seymore] = 0;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 User Control Task
//
// This task is used to control your robot during the user control phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task usercontrol()
{
	startTask(motorcontrol);
	while(1)
	{
		Flypower = (sgn(power) > 0 ? power : 0 );
		motor[FeedMe] = vexRT[Btn6U]*127 + vexRT[Btn6D]*-127;
		motor[seymore] = vexRT[Btn5U]*repeater + vexRT[Btn5D]*-repeater;
		motor[ce] = Flypower;
		motor[rb] = Flypower;
		motor[er] = Flypower;
		motor[us] = Flypower;
		motor[BLeft] = vexRT[Ch3];
		motor[FLeft] = vexRT[Ch3];
		motor[BRight] = vexRT[Ch2];
		motor[FRight] = vexRT[Ch2];
		for (int i = 0; i < 5; i++)
		{
			n = (vexRT[buttons[i]] == 1 ? i : n);
		}
		Kp = Kps[n];
		Kd = Kds[n];
		Ki = Kis[n];
		repeater = repeaters[n];
		TargetSpeed = TargetSpeeds[n];
		setpower = setpowers[n];
		Integral = vexRT[Btn8D] == 1 ? 0 : Integral;
	}
}