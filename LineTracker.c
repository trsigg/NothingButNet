#pragma config(Sensor, in1,    LineLeft,       sensorLineFollower)
#pragma config(Sensor, in2,    LineRight,      sensorLineFollower)
#pragma config(Motor,  port2,           MotorLeft,     tmotorVex269_MC29, openLoop)
#pragma config(Motor,  port3,           MotorRight,    tmotorVex269_MC29, openLoop)

task main ()
{

	while(true)
  {

    displayLCDPos(1,6);
    displayNextLCDNumber(SensorValue(LineLeft));
    displayLCDPos(1,6);
    displayNextLCDNumber(SensorValue(LineRight));

  	{while (true)
    	int Left = 505;
    	int Right = 505;


    	if (SensorValue(LineLeft) > Left && SensorValue(LineRight) > Right){      // both sensors see white, the robot must go straight
     		motor[MotorLeft] = 35;
      	motor[MotorRight] = -35;     }

    	if (SensorValue(LineLeft) == Left ){                  // left sensor sees black, so the robot must go to the right
      	motor[MotorLeft] = -20;
      	motor[MotorRight] = -50;      }

   		if (SensorValue(LineRight) == Right){                  // right sensor sees black, so the robot must go to the left
      	motor[MotorLeft] = 50;
      	motor[MotorRight] = 20;     }
      }
	}

}
