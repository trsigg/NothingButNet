#pragma config(Sensor, in1,    Yaw,           sensorGyro)
#pragma config(Sensor, in2,    BallFeed,      sensorLineFollower)
#pragma config(Sensor, in3,    BallLaunch,    sensorLineFollower)
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
	//Selection Variables
int n = 0;
TVexJoysticks buttons[4] = {Btn8D, Btn7U, Btn7R, Btn7D};
	//PID Control
int TargetSpeed[4] = {0, 332, 340, 433}; //E Team
//int TargetSpeed[4] = {0, 342, 355, 415}; //X Team
//int TargetSpeed[4] = {0, 350, 388, 465}; //G Team
//int TargetSpeed[4] = {0, 175, 155, 190}; //W Team
//int TargetSpeed[4] = {0, 350, 370, 445}; //C (F) Team
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
int PIDPower = 0;//PdBang Selection
int AccError[4]  = {-1, -1, 7, 10};
float ErrorMargarine[4] = {1.0,1.0,1.0,1.0}/*{0, 0.17, 0.05, 0.02}*/;
int BangBang = 0;
int PIDBang = 0;
	//AutomaticSeymore
int BallThreshold = 3020; //Line
bool AutoGo = false;
bool Meter = true;
bool PossBall = false;
int BallCount = 0;
bool BallLoss = false;
int AutoToggle = 1;
bool SecondToggle = false;

void pre_auton()
{
	bStopTasksBetweenModes = true;
	SensorType[Yaw] = sensorNone;
	for(int i = 0; i<2000; i++)
	{
		cumBias += SensorValue[Yaw];
	}
	SensorType[Yaw] = sensorGyro;
	SensorBias[Yaw] = cumBias/2000;
}

void Motorspeeds()
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
	motor[FeedMe] = vexRT[Btn6U]*127 + vexRT[Btn6D]*-127;//AOI Logic
}

void MechSeymore()
{
	AutoGo = AutoToggle==1&&SensorValue[BallLaunch]>=BallThreshold&&SensorValue[BallFeed]<BallThreshold&&vexRT[Btn5D]==0&&n==0?true:false;//Check for a ball
	Meter = SensorValue[BallLaunch]>=BallThreshold||(abs(Error)<=(ErrorMargarine[n]*TargetSpeed[n]))?true:false;
	BallCount += PossBall==true&&AutoGo==false?1:0;
	PossBall = AutoGo==true?true:false;
	BallCount += BallLoss==true&&SensorValue[BallLaunch]>=BallThreshold?-1:0;
	BallLoss = SensorValue[BallLaunch]>=BallThreshold?false:true;
	AutoToggle = vexRT[Btn8L]==0&&SecondToggle==true?abs(AutoToggle-1):AutoToggle;
	SecondToggle = vexRT[Btn8L]==1?true:false;
	SeymoreSpeed = 100*AutoGo+127*vexRT[Btn5U]*Meter-127*vexRT[Btn5D];//AOI Combination
}

task PIDControl()
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
		KbBattery = n==0?0:1/(Kb[n]*nAvgBatteryLevel);//Battery Level
		PIDPower = KpError + KdDeltaE + KiIntegral + stillspeed[n];//PID Equation
		BangBang = Error > 0 ? 127 : 0;//KbBattery;//On or Off, Pure Binary
		PIDBang = abs(Error)>AccError[n] ? BangBang : PIDPower;//Axiom of choice
		Flyspeed = PIDBang<0?0:PIDBang*(n==0?0:1);//No constant flywheel :(
	}
}

task Timer()
{
	datalogClear();
 	while(true)
	{
		if(n!=0)
		{
			clearTimer(T1);
			waitUntil(abs(Error)<AccError[n]);
			datalogAddValue(0, time1[T1]);
			waitUntil(abs(Error)>AccError[n]);
		}
	}
}

task usercontrol()
{
	startTask(PIDControl);
	startTask(Timer);
	while(true)
	{
		Motorspeeds();
		MechSeymore();
		LeftSpeed = abs(vexRT[Ch3])>10?vexRT[Ch3]:0;//Human Control
		RightSpeed = abs(vexRT[Ch2])>10?vexRT[Ch2]:0;
		for(int i = 0; i<4; i++)//Human Choice
		{
			n = vexRT[buttons[i]] == 1 ? i : n;
		}
	}
}

//////////////////////////////////////
//           Tynan's region         //
//////////////////////////////////////

#define flywheelTimer T1
#define driveTimer T2
#define fireTimer T3
#define feedTimer T4

int limit(int input, int min, int max) {
	if (input <= max && input >= min) {
		return input;
	}
	else {
		return (input > max ? max : min);
	}
}

void setDrivePower(int left, int right) {
	motor[RightDrive1] = right;
	motor[RightDrive2] = right;
	motor[LeftDrive1] = left;
	motor[LeftDrive2] = left;
}

//turn
float degreesToTurn;
int maxTurnSpeed, waitAtEnd;

void turnEnd() {
	setDrivePower(sgn(degreesToTurn) * 10, -sgn(degreesToTurn) * 10);
	int brakeDelay = limit(250, 0, waitAtEnd);
	wait1Msec(brakeDelay);
	setDrivePower(0, 0);

	if (waitAtEnd > 250) wait1Msec(waitAtEnd - 250); //wait at end
}

task turnTask() {
	while (abs(SensorValue[Yaw]) < abs(degreesToTurn * 10)) { EndTimeSlice(); }
	turnEnd();
}

void turn(float _degreesToTurn_, int _maxTurnSpeed_=50, bool runAsTask=false, int _waitAtEnd_=250) {
	degreesToTurn = _degreesToTurn_;
	maxTurnSpeed = _maxTurnSpeed_;
	waitAtEnd = _waitAtEnd_;

	SensorValue[Yaw] = 0; //clear the gyro
	setDrivePower(-sgn(degreesToTurn) * maxTurnSpeed, sgn(degreesToTurn) * maxTurnSpeed); //begin turn

	if (runAsTask) {
		startTask(turnTask);
	}
	else {
		while (abs(SensorValue[Yaw]) < abs(degreesToTurn * 10)) { EndTimeSlice(); }
		turnEnd();
	}
}
//end turn

//driveStraight
float coeff = 15;
bool driveStraightRunning = false;
int clicks, direction, drivePower, delayAtEnd, driveTimeout, totalClicks, slavePower, driveStraightError;

void driveStraightRuntime() {
	setDrivePower(slavePower * direction, drivePower * direction);

	driveStraightError = SensorValue[Yaw];

	slavePower += driveStraightError / coeff;

	totalClicks += (abs(SensorValue[leftE]) + abs(SensorValue[rightE])) / 2;
	SensorValue[leftE] = 0;
	SensorValue[rightE] = 0;
}

void driveStraightEnd() {
	setDrivePower(0, 0);
	wait1Msec(delayAtEnd);
	driveStraightRunning = false;
}

task driveStraightTask() {
	while (abs(totalClicks) < clicks  && time1(driveTimer) < driveTimeout) {
		driveStraightRuntime();

		wait1Msec(100);
	}
	driveStraightEnd();
}

void driveStraight(int _clicks_, int _delayAtEnd_=250, int _drivePower_=60, bool startAsTask=false, int _timeout_=15000) {
	//initialize global variables
	clicks = abs(_clicks_);
	direction = sgn(_clicks_);
	drivePower = _drivePower_;
	delayAtEnd = _delayAtEnd_;
	driveTimeout = _timeout_;

	totalClicks = 0;
	slavePower = drivePower - 5;
	driveStraightError = 0;

	//initialize sensors
	SensorValue[leftE] = 0;
	SensorValue[rightE] = 0;
	SensorValue[Yaw] = 0;
	clearTimer(driveTimer);

	if (startAsTask) {
		startTask(driveStraightTask);
	}
	else { //runs as function
		while (abs(totalClicks) < clicks  && time1(driveTimer) < driveTimeout) {
			driveStraightRuntime();
			wait1Msec(100);
		}
		driveStraightEnd();
	}
}
//end driveStraight

//ball counting
int ballsInFeed; //also used in fire and photoresistor

task fireCounting() {
	while (true) {
		while (SensorValue[BallLaunch] > BallThreshold) { EndTimeSlice(); }
		while (SensorValue[BallLaunch] < BallThreshold) { EndTimeSlice(); }
		ballsInFeed = limit(ballsInFeed-1, 0, 4);
		wait1Msec(250);
	}
}
//end ball counting

//photoresistor
int photoFeedPower = 0; //also used in feedControl

task autoFeeding() {
	motor[FeedMe] = 127;
	while (true) { motor[Seymore] = photoFeedPower;	}
}

task photoresistor() {
	startTask(fireCounting);
	ballsInFeed = 0;

	while (true) {
		while (ballsInFeed < 4) {
			while (SensorValue[BallFeed] > BallThreshold) { EndTimeSlice(); }
			while (SensorValue[BallFeed] < BallThreshold) {
				photoFeedPower = (SensorValue[BallLaunch] > BallThreshold) ? 127 : 0;
				EndTimeSlice();
			}

			ballsInFeed = limit(ballsInFeed+1, 0, 4);

			clearTimer(feedTimer);
			wait1Msec(50);
			photoFeedPower = 0;
		}

		photoFeedPower = (SensorValue[BallLaunch] > BallThreshold) ? 127 : 0;
		while (SensorValue[BallLaunch] > BallThreshold) { EndTimeSlice(); }
		photoFeedPower = 0;
	}
}
//end photoresistor

//fire
bool firing = false; //also used in autofeeding
int fireTimeout, ballsToFire;

task fireTask() {
	firing = true;
	stopTask(autoFeeding);
	clearTimer(fireTimer);
	int targetBalls = ballsInFeed - ballsToFire;

	while (ballsInFeed > targetBalls && time1(fireTimer) < fireTimeout) {
		motor[Seymore] = (abs(Error) < TargetSpeed[n] * ErrorMargarine[n] || SensorValue[BallLaunch] > BallThreshold) ? 127 : 0;
		motor[FeedMe] = motor[Seymore];
		EndTimeSlice();
	}

	firing = false;
	ballsInFeed = 0;
	startTask(autoFeeding);
}

void fire(int _ballsToFire_=ballsInFeed, int _timeout_=6000) {
	fireTimeout = _timeout_;
	ballsToFire = _ballsToFire_;

	startTask(fireTask);
}
//end fire

void initializeTasks(bool autonomous) {
	startTask(PIDControl);
	startTask(Timer);
	startTask(photoresistor);
	startTask(autoFeeding);
}

task skillPointAuto() {
	n = 2;
	wait1Msec(2000);
	fire(5);
	while (true) { EndTimeSlice(); }
}

task stationaryAuto() {
	n = 3;
	wait1Msec(2000);
	fire(5);
	while (true) { EndTimeSlice(); }
}

task hoardingAuto() {
	driveStraight(2000); //drive forward
	turn(-15); //turn
	driveStraight(-1000, 80); //back up to push first stack into start zone
	turn(18); //turn toward second stack
	n = 1;
	driveStraight(2300, 750); //pick up second stack
	//fire second stack
	fire();
	while (firing) { EndTimeSlice(); }

	turn(-65); //turn toward third stack
	//pick up third stack
	driveStraight(1100);
}

task classicAuto() {
	n = 3;
	//fire four initial preloads
	fire(4);
	while (firing) { EndTimeSlice(); }

	n = 2;
	turn(16); //turn toward first stack
	//pick up first stack
	driveStraight(800);

	turn(-16); //turn toward net
	driveStraight(1150); //drive toward net
	fire();
	while (firing) { EndTimeSlice(); }

	//pick up second stack
	driveStraight(950); //drive into net for realignment
	driveStraight(-750); //move back
	//fire second stack
	fire();
	while (firing) { EndTimeSlice(); }

	turn(-65); //turn toward third stack
	//pick up third stack
	driveStraight(1100);
}

task pskillz() {
	//start flywheel
	n = 2;

	wait1Msec(1000);
	fire(32);
	//wait until first set of preloads are fired
	while (firing) { EndTimeSlice(); }

	turn(108); //turn toward middle stack
	driveStraight(2300); //drive across field
	turn(-15); // turn toward starting tiles
	driveStraight(1200); //drive across field
	turn(-60); //turn toward net

	//fire remaining balls
	fire();
	while (true) { EndTimeSlice(); }
}

task autonomous() {
	initializeTasks(true);

	//startTask(skillPointAuto);
	//startTask(stationaryAuto);
	//startTask(hoardingAuto);
	startTask(classicAuto);
	//startTask(pskillz);

	while (true) {
		motor[Fly1] = vexRT[Btn8L]==0?(Flyspeed):-vexRT[Ch3];//See Task PIDControl
		motor[Fly2] = vexRT[Btn8L]==0?(Flyspeed):-vexRT[Ch3];
		motor[Fly3] = vexRT[Btn8L]==0?(Flyspeed):-vexRT[Ch3];
		motor[Fly4] = vexRT[Btn8L]==0?(Flyspeed):-vexRT[Ch3];
		EndTimeSlice();
	}
}
