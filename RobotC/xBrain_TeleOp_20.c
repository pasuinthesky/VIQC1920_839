#pragma config(Sensor, port10, LED,            sensorVexIQ_LED)
#pragma config(Sensor, port12, gyro,           sensorVexIQ_Gyro)
#pragma config(Motor,  motor1,          BR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor4,          clawMotor,     tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor5,          BL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor6,          FL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor7,          FR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         armMotor,      tmotorVexIQ, PIDControl, reversed, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define INERTIA_DIE_DOWN 300

#define RELEASED 275
#define HOOKED 35

#define ARM_DELTA 5
#define CLAW_DELTA 15
#define MS_PER_ENCODER_UNIT 2

#define LIFT_LEVELS 3

int error;
float integral = 0;
float dt = 25;
float prev_error = 0;
float allowed_time = 7.5;
float output;
float delta = 10;
int artificial_ChC_Reading = 0;
float desired_heading;
bool claw_working = false;
bool claw_to_release = false;

int iArmLevel[LIFT_LEVELS] = {40, 455, 550};
int in_between_level = 400;

bool drive_override = false;
bool slow_drive = false;

int iChC_filtered;
int iChA_filtered;
int iChB_filtered;

#define GYRO_SAMPLING_SECONDS 10000
#define ACCEPTABLE_DRIFT_RANGE 0.08
float fGyroDriftRate;

float timestamp;

/*
int iDriveMapping[101] = {
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
10,10,10,10,10,10,10,10,10,10,
10,10,10,10,10,10,10,10,10,10,
20,20,20,20,20,20,20,20,20,20,
30,30,30,30,30,30,30,30,30,30,
40,40,40,40,40,40,40,40,40,40,
50,50,50,50,50,50,50,50,50,50,
60,62,64,66,68,70,72,74,76,78,
84,88,92,96,100,100,100,100,100,100,100};

*/
int iStrafeMapping[101] = {
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	10,10,10,10,10,10,10,10,10,10,
	10,10,10,10,10,10,10,10,10,10,
	20,20,20,20,20,20,20,20,20,20,
	20,20,20,20,20,20,20,20,20,20,
	30,30,30,30,30,30,30,30,30,30,
	30,30,30,30,30,30,30,30,30,30,
	40,40,40,40,40,60,60,60,60,60,
	60,60,60,60,60,80,80,80,80,80,80}; // Max not to 100 to allow room for PID.

int iSlowStrafeMapping[101] = {
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	10,10,10,10,10,10,10,10,10,10,
	10,10,10,10,10,10,10,10,10,10,
	15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,
	20,20,20,20,20,20,20,20,20,20,
	20,20,20,20,20,20,20,20,20,20,
	20,20,20,20,20,20,20,20,20,20,
	20,20,20,20,20,30,30,30,30,30,30};


int iTurnMapping[101] = {
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	10,10,10,10,10,10,10,10,10,10,
	10,10,10,10,10,10,10,10,10,10,
	20,20,20,20,20,20,20,20,20,20,
	20,20,20,20,20,20,20,20,20,20,
	30,30,30,30,30,30,30,30,30,30,
	30,30,30,30,30,30,30,30,30,30,
	30,30,30,30,30,30,30,30,30,30,
	40,40,40,40,40,40,40,60,60,60,60};

int iSlowTurnMapping[101] = {
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	5,5,5,5,5,5,5,5,5,5,
	5,5,5,5,5,5,5,5,5,5,
	10,10,10,10,10,10,10,10,10,10,
	10,10,10,10,10,10,10,10,10,10,
	15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,
	20,20,20,20,20,20,20,30,30,30,30};


void setGyroStable()
{
	fGyroDriftRate = 100;

	while (fGyroDriftRate > ACCEPTABLE_DRIFT_RANGE)
	{
		// Let the LDH flashing to remind people do not move the robot.
		setTouchLEDColor(LED,colorRed);
		setTouchLEDBlinkTime(LED, 14,6);
		wait1Msec(3);
		setTouchLEDBlinkTime(LED, 0,1);
		wait1Msec(2);

		setTouchLEDColor(LED,colorGreen);
		setTouchLEDBlinkTime(LED, 8, 12);

		resetGyro(gyro);
		clearTimer(T4);
		wait1Msec(GYRO_SAMPLING_SECONDS);

		fGyroDriftRate = getGyroDegreesFloat(gyro) / GYRO_SAMPLING_SECONDS;

		if (fGyroDriftRate < ACCEPTABLE_DRIFT_RANGE)
			setTouchLEDColor(LED, colorGreen);
		else
			setTouchLEDColor(LED, colorRed);

		setTouchLEDBlinkTime(LED, 4, 4);
		wait1Msec(1);
		setTouchLEDColor(LED,colorNone);
		setTouchLEDBlinkTime(LED, 1, 0);
	}

}

float getGyroStable()
{
	return getGyroDegreesFloat(gyro) - fGyroDriftRate * time1[T4] / 1000;
}

void resetGyroStable()
{
	resetGyro(gyro);
	clearTimer(T4);
}

void eightDirectionalLimitedJoystick()
{
	int x = getJoystickValue(ChB);
	int y = getJoystickValue(ChA);

	int strafeSpeed = sqrt(x*x + y*y); //this is the hypotenuse

	if (strafeSpeed == 0)
	{
		iChA_filtered = 0;
		iChB_filtered = 0;
	}
	else
	{
		// In C language, int devided by int equals int.
		// When y smaller than strafeSpeed, y/strafeSpeed will always equals 0 if not forced into the float type.
		float abs_sin = abs((float)y/strafeSpeed);

		// We have to do this to avoid referring iStrafeMapping array out of boundary.
		// We have to do this after calculating abs_sin to get accurate value.
		if (strafeSpeed > 100)
			strafeSpeed = 100;

		// Default to diagnal directions and slow mode
		if (slow_drive)
		{
			iChA_filtered = sgn(y) * iSlowStrafeMapping[strafeSpeed];
			iChB_filtered = sgn(x) * iSlowStrafeMapping[strafeSpeed];
		}
		else
		{
			iChA_filtered = sgn(y) * iStrafeMapping[strafeSpeed];
			iChB_filtered = sgn(x) * iStrafeMapping[strafeSpeed];
		}

		if( abs_sin <= sinDegrees(22.5))
		{
			iChA_filtered = 0;
		}
		else if ( abs_sin >= sinDegrees(67.5))
		{
			iChB_filtered = 0;
		}
	}
}
/*
task claw_preset()
{
int claw_position, prev_claw = 0;

while (true)
{
if (getJoystickValue(BtnLUp)==1 && claw_working == false)
{
if (claw_to_release)
{
claw_to_release = false
}
else
{
claw_to_release = true
}
claw_working = true
}

if (claw_working)
{
if (claw_to_release)
{
if ( abs(getMotorEncoder(armMotor)) >= iArmLevel[1] - ARM_DELTA )
{
drive_override = true;
setMotorTarget(armMotor, in_between_level, 100);
wait1Msec(150);
}
setMotorBrakeMode(clawMotor, motorCoast);
setMotorTarget(clawMotor,CLAW_CLOSED,100);
wait1Msec( abs( CLAW_CLOSED - CLAW_OPEN ) * MS_PER_ENCODER_UNIT );
setMotorSpeed(clawMotor, 0);
if ( drive_override  )
{
setMotorSpeed(BL, -50 );
setMotorSpeed(BR, 50 );
setMotorSpeed(FL, -50 );
setMotorSpeed(FR, 50 );
wait1Msec(500);
setMotorSpeed(BL, 0 );
setMotorSpeed(BR, 0 );
setMotorSpeed(FL, 0 );
setMotorSpeed(FR, 0 );
setMotorTarget(armMotor, iArmLevel[0], 100);
drive_override = false;
}
claw_working = false
}
else
{
setMotorBrakeMode(clawMotor, motorHold);
claw_position = getMotorEncoder(clawMotor);
if (prev_claw == claw_position)
{
setMotorSpeed(clawMotor, 0);
claw_working = false;
prev_claw = 0;
}
else
{
setMotorSpeed(clawMotor, 100);
wait1Msec(100);
}
prev_claw = claw_position;
}
}
wait1Msec(dt);
}
}
*/

task claw_preset()
{
	int claw_position, prev_claw = -1000;
	bool grab_again = false;

	while (true)
	{
		if ( getJoystickValue(BtnLUp)==1 && claw_working == false )
		{
			if (claw_to_release)
			{
				claw_to_release = false;
			}
			else
			{
				claw_to_release = true;
			}
			claw_working = true;
		}

		if (claw_working)
		{
			if (claw_to_release)
			{
				if ( abs(getMotorEncoder(armMotor)) >= iArmLevel[1] - ARM_DELTA )
				{
					drive_override = true;
					setMotorSpeed(BL, 0 );
					setMotorSpeed(BR, 0 );
					setMotorSpeed(FL, 0 );
					setMotorSpeed(FR, 0 );
					setMotorTarget(armMotor, in_between_level, 100);
					wait1Msec(150);
				}


				setMotorTarget(clawMotor,RELEASED,100);
				wait1Msec( abs( getMotorEncoder(clawMotor) - RELEASED ) * MS_PER_ENCODER_UNIT );
				setMotorSpeed(clawMotor, 0);
				if ( drive_override  )
				{
					desired_heading += 3;
					setMotorSpeed(BL, -50 );
					setMotorSpeed(BR, 50 );
					setMotorSpeed(FL, -50 );
					setMotorSpeed(FR, 50 );
					wait1Msec(375);
					setMotorSpeed(BL, 0 );
					setMotorSpeed(BR, 0 );
					setMotorSpeed(FL, 0 );
					setMotorSpeed(FR, 0 );
					setMotorTarget(armMotor, iArmLevel[0], 100);
					drive_override = false;
				}
				claw_working = false;

			}
			else
			{
				setMotorBrakeMode(clawMotor, motorHold);
				claw_position = getMotorEncoder(clawMotor);
				if (prev_claw == claw_position)
				{
					setMotorSpeed(clawMotor, 0);
					claw_working = false;
					prev_claw = 0;
				}
				else
				{
					prev_claw = claw_position;
					setMotorSpeed(clawMotor, -100);
					wait1Msec(100);
				}
			}
		}
		wait1Msec(dt);
	}
}

float PIDControl (float setpoint, float measured_value, float Kp, float Ki, float Kd, int delta)
{
	error = measured_value - setpoint;

	if ( abs(error) < delta )
		error = 0;

	integral = integral + error * dt;
	float derivative = ( error - prev_error ) / dt;
	output = Kp * error + Ki * integral + Kd * derivative;
	prev_error = error;

	return output;
}

void lift_preset()
{
	if (getJoystickValue(BtnRUp)== 1)
	{
		setMotorSpeed(BL, 0 );
		setMotorSpeed(BR, 0 );
		setMotorSpeed(FL, 0 );
		setMotorSpeed(FR, 0 );

		for (int i=0;i<LIFT_LEVELS-1;i=i+1)
		{
			if ( getMotorEncoder(armMotor) < ( iArmLevel[i+1] - ARM_DELTA ) )
			{
				setMotorTarget(armMotor, iArmLevel[i+1], 100);
				wait1Msec( ( iArmLevel[i+1] - iArmLevel[i] ) * MS_PER_ENCODER_UNIT );
				break;
			}
		}
	}
	else if (getJoystickValue(BtnRDown)==1)
	{
		for (int i=LIFT_LEVELS-1;i>0;i=i-1)
		{
			if ( getMotorEncoder(armMotor) > ( iArmLevel[i-1] + ARM_DELTA ) )
			{
				setMotorTarget(armMotor, iArmLevel[i-1],100);
				wait1Msec( ( iArmLevel[i] - iArmLevel[i-1] ) * MS_PER_ENCODER_UNIT + 100 );
				break;
			}
		}
	}
}
/*
task flashLED()
{
while( true )
{
setTouchLEDRGB(LED, 0, 255, 0);
sleep( 500 );
setTouchLEDRGB(LED, 0, 255, 255);
sleep( 500 );
}
}*/
task flashLED ()
{
	while (true)
	{
		setTouchLEDColor(LED, colorRed);
		wait1Msec(10);
		setTouchLEDColor(LED, colorBlue);
		wait1Msec(10);
		setTouchLEDColor(LED, colorYellow);
		wait1Msec(10);
		setTouchLEDColor(LED, colorOrange);
		wait1Msec(10);
		setTouchLEDColor(LED, colorGreen);
		wait1Msec(10);
		setTouchLEDColor(LED, colorViolet);
	}
}




task main()
{
	setMotorBrakeMode(FL, motorHold);
	setMotorBrakeMode(FR, motorHold);
	setMotorBrakeMode(BR, motorHold);
	setMotorBrakeMode(BL, motorHold);
//setMotorBrakeMode(clawMotor, motorCoast);
	setMotorBrakeMode(clawMotor, motorHold);
	setMotorBrakeMode( armMotor, motorHold );

	setMotorSpeed(clawMotor, -100);
	setMotorSpeed(armMotor, -50);
	wait1Msec(3000);

	resetMotorEncoder(clawMotor);
	resetMotorEncoder(armMotor);

	setMotorSpeed(clawMotor, 0);
	setMotorSpeed(armMotor, 0);

	setMotorTarget(clawMotor, RELEASED, 60);
	setMotorTarget(armMotor, iArmLevel[0], 100);
	wait1Msec(1000);
	setMotorSpeed(clawMotor, 0);

	setGyroStable();
	resetGyroStable();

	timestamp = getTimerValue(T1);

	desired_heading = 0;

	//startTask( flashLED );
	startTask(claw_preset);

	resetTimer(T2);
	//clearDebugStream();

	while(true)
	{
		//		iChA_filtered = iDriveMapping[abs(getJoystickValue(ChA))]*sgn(getJoystickValue(ChA));
		//		iChB_filtered = iDriveMapping[abs(getJoystickValue(ChB))]*sgn(getJoystickValue(ChB));

		if(slow_drive)
		{
			iChC_filtered = iSlowTurnMapping[abs(getJoystickValue(ChC))]*sgn(getJoystickValue(ChC));
		}
		else
		{
			iChC_filtered = iTurnMapping[abs(getJoystickValue(ChC))]*sgn(getJoystickValue(ChC));
		}
		eightDirectionalLimitedJoystick();

		if (getJoystickValue(BtnFUp) && getTimerValue(T1) > (500 + timestamp))
		{
			timestamp = getTimerValue(T1);

			if(slow_drive)
			{
				slow_drive = false;
			}
			else
			{
				slow_drive = true;
			}
		}

		if ( abs(getJoystickValue(ChC)) <= 5 )
		{
			if ( getTimerValue(T2) > INERTIA_DIE_DOWN )
			{
				iChC_filtered = PIDControl( desired_heading, getGyroStable(), 1, 0.00001, 0.0001, 0.6 );
				setTouchLEDColor(LED, colorBlue);
			}
			else
			{
				desired_heading = getGyroStable();
				setTouchLEDColor(LED, colorYellow);
			}
		}
		else
		{
			desired_heading = getGyroStable();
			setTouchLEDColor(LED, colorYellow);
			clearTimer(T2);
		}

		//writeDebugStreamLine("%f %f %f %f %f %f %f", getTimerValue(T1), getTimerValue(T2), getJoystickValue(ChC), iChC_filtered, desired_heading, getGyroStable(), getTouchLEDBlue(LED));

		//displayCenteredTextLine(3, "%d, %d, %d", getGyroStable(), desired_heading, getMotorBrakeMode(clawMotor));

		lift_preset();

		if (! drive_override)
		{
			setMotorSpeed(BL, 0 + iChA_filtered - iChB_filtered - iChC_filtered );
			setMotorSpeed(BR, 0 - iChA_filtered - iChB_filtered - iChC_filtered );
			setMotorSpeed(FL, 0 + iChA_filtered + iChB_filtered - iChC_filtered );
			setMotorSpeed(FR, 0 - iChA_filtered + iChB_filtered - iChC_filtered );
		}

		wait1Msec(dt);
	}
}
