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

#define CLAW_RELEASE 390
#define CLAW_HOOKED 90

#define ARM_CARRY 220
#define ARM_DELTA 15
#define CLAW_DELTA 80
#define MS_PER_ENCODER_UNIT 2

#define LIFT_LEVELS 3

int error;
float integral = 0;
float dt = 25;
float prev_error = 0;
float output;
float desired_heading;

bool claw_working = false;
bool claw_to_release = false;

int iArmLevel[LIFT_LEVELS] = {150, 1360, 1630};
int in_between_level = 1090;

bool drive_override = false;
bool slow_drive = false;

int iChC_filtered;
int iChA_filtered;
int iChB_filtered;

#define GYRO_SAMPLING_SECONDS 10000
#define ACCEPTABLE_DRIFT_RANGE 0.05
float fGyroDriftRate;

float timestamp1, timestamp2;
int macroName = 0;

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
	60,60,60,80,80,95,95,95,95,95,95}; // Max not to 100 to allow room for PID.

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

void eightDirectionalLimitedJoystick()
{
	int x = getJoystickValue(ChB);
	int y = getJoystickValue(ChA);

	int strafeSpeed = sqrt(x*x + y*y); //this is the hypotenuse

	if ( strafeSpeed == 0 )
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

void stopDrivetrain()
{
	setMotorSpeed(BL, 0);
	setMotorSpeed(BR, 0);
	setMotorSpeed(FL, 0);
	setMotorSpeed(FR, 0);
}

/*
void manual_45degreeTurn()
{
bool turn45 = false;

if ( getJoystickValue(BtnFDown) == 1)
{
desired_heading = desired_heading - 45;
turn45 = true;
}
else if ( getJoystickValue(BtnEDown) )
{
desired_heading = desired_heading + 45;
turn45 = true;
}
if(turn45)
{
drive_override = true;
stopDrivetrain();
waitUntil( abs( getGyroStable() - desired_heading ) < delta);
drive_override = false;
turn45 = false;
}

}*/

task claw_preset()
{
	int claw_position, prev_claw = -1000;
	bool land_riser = false;

	while (true)
	{
		if (claw_working)
		{
			if (claw_to_release)
			{
				if ( abs(getMotorEncoder(armMotor)) >= iArmLevel[1] - ARM_DELTA )
				{
					land_riser = true;
					stopDrivetrain();
					setMotorTarget(armMotor, in_between_level, 100);
					wait1Msec(150);
				}

				setMotorTarget(clawMotor,CLAW_RELEASE,100);
				waitUntilMotorStop(clawMotor);
				setMotorSpeed(clawMotor, 0);
				if ( land_riser  )
				{
					drive_override = true;
					iChA_filtered = -90;
					wait1Msec(400);
					stopDrivetrain();
					drive_override = false;
					land_riser = false;
					setMotorTarget(armMotor, iArmLevel[0], 100);
				}
				claw_working = false;

			}
			else
			{
				setMotorBrakeMode(clawMotor, motorHold);
				claw_position = getMotorEncoder(clawMotor);
				if (prev_claw == claw_position)
				{
					//setMotorSpeed(clawMotor, 0);
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

void lift_preset()
{
	if (getJoystickValue(BtnRUp)== 1)
	{/*
		drive_override = true;
		stopDrivetrain();*/

		for (int i=0;i<LIFT_LEVELS-1;i=i+1)
		{
			if ( getMotorEncoder(armMotor) < ( iArmLevel[i+1] - ARM_DELTA ) )
			{
				setMotorTarget(armMotor, iArmLevel[i+1], 100);
				/*wait1Msec(100);
				waitUntilMotorStop(armMotor);
				drive_override = false;*/
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
				break;
			}
		}
	}
}

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

void initialize()
{
	setMotorEncoderUnits(encoderCounts);

	setMotorBrakeMode(FL, motorCoast);
	setMotorBrakeMode(FR, motorCoast);
	setMotorBrakeMode(BR, motorCoast);
	setMotorBrakeMode(BL, motorCoast);
	setMotorBrakeMode(clawMotor, motorHold);
	setMotorBrakeMode( armMotor, motorHold );

	setMotorSpeed(clawMotor, -100);
	setMotorSpeed(armMotor, -50);
	wait1Msec(3000);

	resetMotorEncoder(clawMotor);
	resetMotorEncoder(armMotor);

	setMotorSpeed(clawMotor, 0);
	setMotorSpeed(armMotor, 0);

	setMotorTarget(clawMotor, CLAW_RELEASE, 100);
	setMotorTarget(armMotor, iArmLevel[0], 100);
	wait1Msec(100);
	waitUntilMotorStop(clawMotor);
	waitUntilMotorStop(armMotor);
	setMotorSpeed(clawMotor, 0);

	setTouchLEDColor(LED,colorYellow);
	waitUntil(getTouchLEDValue(LED));

	setMotorBrakeMode(FL, motorHold);
	setMotorBrakeMode(FR, motorHold);
	setMotorBrakeMode(BR, motorHold);
	setMotorBrakeMode(BL, motorHold);
}

task drive()
{
	while (true)
	{
		if ( getTimerValue(T2) > INERTIA_DIE_DOWN || drive_override )
		{
			iChC_filtered = PIDControl( desired_heading, getGyroStable(), 1, 0.00001, 0.0001, 0.6 );
			setTouchLEDColor(LED, colorBlue);
		}
		else
		{
			desired_heading = getGyroStable();
			setTouchLEDColor(LED, colorYellow);
		}

		setMotorSpeed( FL, 0 + iChA_filtered + iChB_filtered - iChC_filtered );
		setMotorSpeed( BL, 0 + iChA_filtered - iChB_filtered - iChC_filtered );
		setMotorSpeed( FR, 0 - iChA_filtered + iChB_filtered - iChC_filtered );
		setMotorSpeed( BR, 0 - iChA_filtered - iChB_filtered - iChC_filtered );

		wait1Msec(dt);
	}
}

void macro(int name)
{
	drive_override = true;

	switch (name)
	{
	case 0:
		iChA_filtered = 60;
		iChB_filtered = 10;
		wait1Msec(400);
		iChA_filtered = 0;
		iChB_filtered = 0;

		desired_heading = 90;
		waitUntil( abs( getGyroStable() - desired_heading ) < 10);

		iChA_filtered = 60;
		iChB_filtered = -10;
		wait1Msec(300);
		iChA_filtered = 0;
		iChB_filtered = 0;

		setMotorSpeed(clawMotor, -100);
		setMotorTarget(armMotor, ARM_CARRY, 100);
		waitUntilMotorStop(clawMotor);
		waitUntilMotorStop(armMotor);


		iChA_filtered = 0;
		iChB_filtered = -90;
		wait1Msec(460);
		iChA_filtered = 0;
		iChB_filtered = 0;

		break;

	case 1:
		iChA_filtered = 0;
		iChB_filtered = 90;
		wait1Msec(900);
		break;

	case 2:
		iChA_filtered = 60;
		iChB_filtered = 10;
		wait1Msec(250);
		iChA_filtered = 0;
		iChB_filtered = 0;
		waitUntil(getTouchLEDValue(LED) == 1 );

		desired_heading = 45;
		waitUntil( abs( getGyroStable() - desired_heading ) < 10);

		iChA_filtered = 60;
		iChB_filtered = -10;
		wait1Msec(350);
		iChA_filtered = 0;
		iChB_filtered = 0;

		setMotorSpeed(clawMotor, -100);
		setMotorTarget(armMotor, ARM_CARRY, 100);
		waitUntilMotorStop(clawMotor);
		waitUntilMotorStop(armMotor);

		iChA_filtered = 0;
		iChB_filtered = 0;

		desired_heading = -135;
		waitUntil( abs( getGyroStable() - desired_heading ) < 20);

		break;
	}
	drive_override = false;
}

task main()
{
	initialize();

	setGyroStable();
	resetGyroStable();

	timestamp1 = getTimerValue(T1);
	timestamp2 = getTimerValue(T1);

	desired_heading = 0;

	startTask(drive);
	startTask(claw_preset);

	resetTimer(T2);
	//clearDebugStream();

	while(true)
	{
		if ( !drive_override )
		{
			if (getJoystickValue(BtnFUp) && getTimerValue(T1) > (500 + timestamp1))
			{
				timestamp1 = getTimerValue(T1);

				if(slow_drive)
				{
					slow_drive = false;
				}
				else
				{
					slow_drive = true;
				}
			}

			if(slow_drive)
			{
				iChC_filtered = iSlowTurnMapping[abs(getJoystickValue(ChC))]*sgn(getJoystickValue(ChC));
			}
			else
			{
				iChC_filtered = iTurnMapping[abs(getJoystickValue(ChC))]*sgn(getJoystickValue(ChC));
			}

			if ( abs(getJoystickValue(ChC)) > 5 )
			{
				clearTimer(T2);
			}

			eightDirectionalLimitedJoystick();
		}

		//writeDebugStreamLine("%f %f %f %f %f %f %f", getTimerValue(T1), getTimerValue(T2), getJoystickValue(ChC), iChC_filtered, desired_heading, getGyroStable(), getTouchLEDBlue(LED));
		//displayCenteredTextLine(3, "%d, %d, %d", getGyroStable(), desired_heading, getMotorBrakeMode(clawMotor));

		if ( getJoystickValue(BtnLUp) && !claw_working )
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

		lift_preset();

		if (getJoystickValue(BtnEDown) && getTimerValue(T1) > (500 + timestamp2) )
		{
			timestamp2 = getTimerValue(T1);
			macro(macroName);
			macroName += 1;
		}

		wait1Msec(dt);
	}
}