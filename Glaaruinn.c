#pragma config(Sensor, in1,    Yaw,           sensorGyro)
#pragma config(Sensor, in2,    BallFeed,      sensorLineFollower)
#pragma config(Sensor, in3,    BallLaunch,    sensorLineFollower)
#pragma config(Sensor, in4,		 AutoChoice,    sensorPotentiometer)
#pragma config(Sensor, in5,    ColorBlind,    sensorPotentiometer)
#pragma config(Sensor, dgtl1,  FlyWheel,      sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  leftE,         sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  rightE,        sensorQuadEncoder)
#pragma config(Motor,  port1,  Seymore,       tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,  LeftDrive1,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,  LeftDrive2,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,  Fly1,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,  Fly2,          tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,  Fly3,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,  Fly4,          tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,  RightDrive2,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,  RightDrive1,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10, FeedMe,        tmotorVex393_HBridge, openLoop)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
//Variables
	//Gyro SetUp
long cumBias = 0;
	//MotorSpeeds
int LeftSpeed = 0;
int RightSpeed = 0;
int Flyspeed = 0;
int SeymoreSpeed = 0;
int FeedSpeed = 0;
	//Selection Variables
int n = 0;
TVexJoysticks buttons[4] = {Btn8D, Btn7U, Btn7R, Btn7D};
	//PID Control
int TargetSpeed[4] = {0, 317, 352, 430}; //W Team
int Error = 0;//Error stuff
float Kp[4] = {0, 0.79, 0.56, 0.32};//E Team
float KpError = 0;
long Integral[4] = {0, 0, 0, 0};//Integral stuff
float Ki[4] = {0, 0.002, 0.001, 0.001};//E Team
float KiIntegral = 0;
int DeltaE = 0;//DeltaError stuff
float Kd[4] = {0, 0.69, 0.63, 0.36};//E Team
float KdDeltaE = 0;
int PrevError = 0;
int stillspeed[4] = {0, 25, 30, 45};
int PIDPower = 0;//PidBang Selection
int AccError[4]  = {-1, 25, 20, 10};
float ErrorMargarine[4] = {0, 0.17, 0.05, 0.06};
int BangBang = 0;
int PIDBang = 0;
	//AutomaticSeymore
int BallThreshold = 3040;
bool AutoGo = false;
bool Meter = true;
bool PossBall = false;
int BallCount = 0;
int PrevBallCount = 0;
bool BallLoss = false;
int AutoToggle = 1;
bool SecondToggle = false;
int AutoMode = 0;
	//Autonomous choice
bool Classic = SensorValue[AutoChoice]>90?true:false;//Tynan
bool Team = SensorValue[AutoChoice]<90&&SensorValue[AutoChoice]>65?true:false;//You Guyz
bool Skillz = SensorValue[AutoChoice]<65&&SensorValue[AutoChoice]>25?true:false;//Probs Tynan
bool Attack = SensorValue[AutoChoice]<25?true:false;//Gaberino the Magnifico
bool RedRight = SensorValue[ColorBlind]>90?true:false;//Location
bool RedLeft = SensorValue[ColorBlind]<90&&SensorValue[AutoChoice]>65?true:false;
bool BlueRight = SensorValue[ColorBlind]<65&&SensorValue[AutoChoice]>25?true:false;
bool BlueLeft = SensorValue[ColorBlind]<25?true:false;
	//Gabe Autonomous
int AutoStep = 0;
bool run = false;
int ChoiceArray[18];
int DegreeArray[18];
int SpeedArray[18];
int RedAttackChoiceArray[18] = {2, 2, 2, 3, 1, 3, 2, 2, 2, 3, 1, 3, 2, 2, 2, 3, 1, 1};
int RedAttackDegreeArray[18] = {310, 20, 110, -36, 3, 12, 220, 20, 110, -17, 2, -12, 850, 18, 110, 35, 2, 0};
int RedAttackSpeedArray[18] = {60, 40, 40, 50, 0, 40, 60, 40, 40, 40, 0, 50, 60, 40, 40, 40, 0, 0};
int BlueAttackChoiceArray[18] = {2, 2, 2, 3, 1, 3, 2, 2, 2, 3, 1, 3, 2, 2, 2, 3, 1, 1};
int BlueAttackDegreeArray[18] = {310, 20, 110, 36, 3, -12, 220, 20, 110, 17, 2, 12, 850, 18, 110, -35, 2, 0};
int BlueAttackSpeedArray[18] = {60, 40, 40, 50, 0, 40, 60, 40, 40, 40, 0, 50, 60, 40, 40, 40, 0, 0};
bool turning = false;
bool safety = true;
	//Tynan Autonomous
float degreesToTurn;
int maxTurnSpeed, waitAtEnd;
float coeff = 15;
bool driveStraightRunning = false;
int clicks, direction, drivePower, delayAtEnd, driveTimeout, totalClicks, slavePower, driveStraightError;
int fireTimeout, ballsToFire;
bool firing = false;
	//Timers
#define flywheelTimer T1
#define driveTimer T2
#define fireTimer T3
#define feedTimer T4

void pre_auton()
{
	bStopTasksBetweenModes = true;
	SensorType[Yaw] = sensorNone;
	for(int i = 0; i<2000; i++)
	{
		cumBias += SensorValue[Yaw];
		wait1Msec(1);
	}
	for(int i = 0; i<2000; i++)
	{
		ChoiceArray[i] = RedRight||RedLeft?RedAttackChoiceArray[i]:BlueAttackChoiceArray[i];
		DegreeArray[i] = RedRight||RedLeft?RedAttackDegreeArray[i]:BlueAttackDegreeArray[i];
		SpeedArray[i] = RedRight||RedLeft?RedAttackSpeedArray[i]:BlueAttackSpeedArray[i];
	}
	SensorType[Yaw] = sensorGyro;
	SensorBias[Yaw] = cumBias/2000;
}

int limit(int input, int min, int max)//Tynan
{
	input = input<=max?(input>=min?input:min):max;
	return input;
}

void setDrivePower(int left, int right)//Tynan
{
	RightSpeed = right;
	LeftSpeed = left;
}

void turn(float _degreesToTurn_, int _maxTurnSpeed_=50, int _waitAtEnd_=250)//Tynan
{
	degreesToTurn = _degreesToTurn_;
	maxTurnSpeed = _maxTurnSpeed_;
	waitAtEnd = _waitAtEnd_;
	turning = true;
	SensorValue[Yaw] = 0; //clear the gyro
	setDrivePower(-sgn(degreesToTurn) * maxTurnSpeed, sgn(degreesToTurn) * maxTurnSpeed); //begin turn
	while (abs(SensorValue[Yaw]) < abs(degreesToTurn * 10)) { EndTimeSlice(); }
	setDrivePower(sgn(degreesToTurn) * 10, -sgn(degreesToTurn) * 10);//End Turn
	int brakeDelay = limit(250, 0, waitAtEnd);
	wait1Msec(brakeDelay);//Slide to the left (or Right)
	setDrivePower(0, 0);
	if (waitAtEnd > 250) wait1Msec(waitAtEnd - 250);
	turning = false;
}

void driveStraight(int _clicks_, int _drivePower_=60, int _delayAtEnd_=100, int _timeout_=15000)//Tynan
{
	driveStraightRunning = true;
	clicks = abs(_clicks_);//initialize global variables
	direction = sgn(_clicks_);
	drivePower = _drivePower_;
	delayAtEnd = _delayAtEnd_;
	driveTimeout = _timeout_;
	totalClicks = 0;
	slavePower = drivePower - 5;
	driveStraightError = 0;
	SensorValue[leftE] = 0;//initialize sensors
	SensorValue[rightE] = 0;
	SensorValue[Yaw] = 0;
	clearTimer(driveTimer);
	while (abs(totalClicks) < clicks  && time1(driveTimer) < driveTimeout) //Task and not task
		{
			setDrivePower(slavePower * direction, drivePower * direction);//driveStraightRuntime
			driveStraightError = SensorValue[Yaw];
			slavePower += driveStraightError / coeff;
			totalClicks += (abs(SensorValue[leftE]) + abs(SensorValue[rightE])) / 2;
			SensorValue[leftE] = 0;
			SensorValue[rightE] = 0;
			wait1Msec(100);
		}
	setDrivePower(0, 0);
	wait1Msec(delayAtEnd);
	driveStraightRunning = false;
	n = 0;
}

void fire(int _ballsToFire_=BallCount, int _timeout_=6000)//Tynan
{
	fireTimeout = _timeout_;
	ballsToFire = _ballsToFire_;
	firing = true;
	AutoToggle = 0;
	clearTimer(fireTimer);
	//int targetBalls = BallCount - ballsToFire;
	while (/*BallCount > targetBalls && */time1(fireTimer) < fireTimeout) {
		AutoMode = Meter&&n!=0 ? 127 : 0;
		EndTimeSlice();
	}
	firing = false;
	BallCount = 0;
	AutoToggle = 1;
}

void AutoFunction(int choice, int degree, int speed)//Gabe
{
	run = true;
	if(run)
	{
		if(choice == 1)//Firing
		{
			n = degree;
			fire();
			waitUntil(firing == false);
			run = false;
		}
		if(choice == 2)//Driving
		{
			driveStraight(degree, speed);
			waitUntil(driveStraightRunning == false);
			run = false;
		}
		if(choice == 3)//Turning
		{
			turn(degree, speed);
			waitUntil(turning == false);
			run = false;
		}
		run = false;
	}
}

void Motorspeeds()//Gabe
{
	motor[LeftDrive1] = vexRT[Btn8L]==0?(LeftSpeed):0;//Single Variable Drive Control
	motor[LeftDrive2] = vexRT[Btn8L]==0?(LeftSpeed):0;
	motor[RightDrive1] = RightSpeed;
	motor[RightDrive2] = RightSpeed;
	motor[Fly1] = vexRT[Btn8L]==0?(Flyspeed):-vexRT[Ch3];//See Task PIDControl
	motor[Fly2] = vexRT[Btn8L]==0?(Flyspeed):-vexRT[Ch3];
	motor[Fly3] = vexRT[Btn8L]==0?(Flyspeed):-vexRT[Ch3];
	motor[Fly4] = vexRT[Btn8L]==0?(Flyspeed):-vexRT[Ch3];
	motor[Seymore] = SeymoreSpeed;//Function MechSeymore
	motor[FeedMe] = FeedSpeed;
}

task MechSeymore()//Gabe
{
	while(true)
	{
		AutoGo = AutoToggle==1&&SensorValue[BallFeed]<BallThreshold&&SensorValue[BallLaunch]>BallThreshold&&vexRT[Btn5D]==0?true:false;//Check for a ball
		Meter = SensorValue[BallLaunch]>=BallThreshold||(abs(Error)<=(ErrorMargarine[n]*TargetSpeed[n]))?true:false;
		AutoToggle = vexRT[Btn8R]==0&&SecondToggle==true?abs(AutoToggle-1):AutoToggle;
		SecondToggle = vexRT[Btn8R]==1?true:false;
		SeymoreSpeed = 100*AutoGo+127*vexRT[Btn5U]*Meter-127*vexRT[Btn5D]+bIfiAutonomousMode*AutoMode*firing;//AOI Combination
		BallCount += PossBall==true&&AutoGo==false?1:0;
		PossBall = AutoGo==true?true:false;
		BallCount += BallLoss==true&&SensorValue[BallLaunch]>=BallThreshold?-1:0;
		BallLoss = SensorValue[BallLaunch]<=BallThreshold?true:false;
		wait10Msec(PrevBallCount==BallCount?0:10);
		PrevBallCount = BallCount;
	}
}

task PIDControl()//Gabe
{
	while(true)
	{
		SensorValue[FlyWheel] = 0;
		wait1Msec(60);//TimeSample
		Error = TargetSpeed[n] - SensorValue[FlyWheel];//How much I am wrong
		KpError = Kp[n]*Error;
		Integral[n] += (Error + PrevError)/2;//How wrong I've been
		KiIntegral = Ki[n]*Integral[n];
		DeltaE = PrevError - Error;//How much less wrong I am
		KdDeltaE = Kd[n]*DeltaE;
		PrevError = Error;
		PIDPower = KpError + KdDeltaE + KiIntegral + stillspeed[n];//PID Equation
		BangBang = Error > 0 ? 127 : 0;//KbBattery;//On or Off, Pure Binary
		PIDBang = abs(Error)>AccError[n] ? BangBang : PIDPower;//Axiom of choice
	  Flyspeed = PIDBang<0?0:PIDBang*(n==0?0:1);//No constant flywheel :(
	}
}

task BeAggresive()//Gabe
{
	while(true)
	{
		AutoFunction(ChoiceArray[AutoStep], DegreeArray[AutoStep], SpeedArray[AutoStep]);
		waitUntil(run == false);
		AutoStep++;
		AutoStep = limit(AutoStep, 0 , 17);
	}
}

task autonomous()//Gabe
{
	FeedSpeed = 100;
	startTask(PIDControl);
	startTask(MechSeymore);
	if(Attack&&safety)
	{
		startTask(BeAggresive);
		safety = false;
	}
	if(Skillz&&safety)
	{
		startTask(BeAggresive);
		safety = false;
	}
	if(Classic&&safety)
	{
		startTask(BeAggresive);
		safety = false;
	}
	if(Team&&safety)
	{
		startTask(BeAggresive);
		safety = false;
	}
	while (true) {
		FeedSpeed = 127;
		Motorspeeds();
		EndTimeSlice();
	}
}

task usercontrol()//Gabe
{
	startTask(PIDControl);
	startTask(MechSeymore);
	while(true)
	{
		Motorspeeds();
		FeedSpeed = vexRT[Btn6U]*127 + vexRT[Btn6D]*-127;
		setDrivePower(abs(vexRT[Ch3])>10?vexRT[Ch3]:0, abs(vexRT[Ch2])>10?vexRT[Ch2]:0);//Human Control
		for(int i = 0; i<4; i++)//Human Choice
		{
			n = vexRT[buttons[i]] == 1 ? i : n;
		}
	}
}
