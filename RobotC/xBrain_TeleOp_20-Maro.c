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

#define ARM_CARRY 240
#define ARM_DELTA 15
#define CLAW_DELTA 80
#define MS_PER_ENCODER_UNIT 2

#define LIFT_LEVELS 3

bool claw_working = false;
bool claw_to_release = false;

int iArmLevel[LIFT_LEVELS] = {150, 1360, 1630};
int in_between_level = 1090;

bool drive_override = false;
bool slow_drive = false;

int iChC_filtered;
int iChA_filtered;
int iChB_filtered;
float desired_heading = 0.0;

#define GYRO_SAMPLING_SECONDS 5
#define ACCEPTABLE_DRIFT_RANGE 0.05
float fGyroDriftRate;

#define COUNT_PER_ROUND 960
#define WHEEL_TRAVEL 20.0
#define GEAR_RATIO 2
#define RATE 1

float timestamp1, timestamp2;
int macroName = 0;

float dt = 25;

typedef struct {
	float setpoint;
	float measured_value;
	float integral;
	float prev_error;
	float kp;
	float ki;
	float kd;
	float delta;
}structPID

structPID pidStrafe;
structPID pidOrientation;
structPID pidEncoder;

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

float cmToEncoderUnit(float distance)
{
	return distance * COUNT_PER_ROUND / WHEEL_TRAVEL / GEAR_RATIO / RATE;
}

void setGyroStable()
{
	fGyroDriftRate = 100;

	while (fGyroDriftRate > ACCEPTABLE_DRIFT_RANGE)
	{
		// Let the LDH flashing to remind people do not move the robot.
		setTouchLEDColor(LED,colorRed);
		//setTouchLEDBlinkTime(LED, 14,6);
		//wait1Msec(3000);
		setTouchLEDBlinkTime(LED, 0,1);
		wait1Msec(2000);

		setTouchLEDColor(LED,colorGreen);
		setTouchLEDBlinkTime(LED, 8, 12);

		resetGyro(gyro);
		clearTimer(T4);
		wait1Msec(GYRO_SAMPLING_SECONDS*1000);

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

float PIDControl (structPID &pid)
{
	float error = pid.setpoint - pid.measured_value;

	if ( abs(error) < pid.delta )
		error = 0;

	pid.integral = pid.integral + error * dt;
	float derivative = ( error - pid.prev_error ) / dt;
	float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
	pid.prev_error = error;

	return output;
}

void strafePID(int direction, int distance, int maxJoyStick, float Kp, float Ki, float Kd, int delta)
{
	int tmpJoyStick, motor_a, motor_b, ChA_selector, ChB_selector;

	pidStrafe.setpoint = cmToEncoderUnit(distance) / sqrt(2);
	pidStrafe.measured_value = 0;
	pidStrafe.kp = Kp;
	pidStrafe.ki = Ki;
	pidStrafe.kd = Kd;
	pidStrafe.delta = cmToEncoderUnit(delta);
	pidStrafe.integral = 0;
	pidStrafe.prev_error = 0;

	pidEncoder.setpoint = 0;
	pidEncoder.measured_value = 0;
	pidEncoder.kp = 0.015;
	pidEncoder.ki = 0.000002;
	pidEncoder.kd = 0.005;
	pidEncoder.delta = 30;
	pidEncoder.integral = 0;
	pidEncoder.prev_error = 0;

	resetMotorEncoder(FL);
	resetMotorEncoder(FR);
	resetMotorEncoder(BL);
	resetMotorEncoder(BR);

	switch(direction)
	{
	case 1:
		if(distance > 0)
		{
			motor_a = FL;
			motor_b = BL;
		}
		else
		{
			motor_a = FR;
			motor_b = BR;
		}
		ChA_selector = 0;
		ChB_selector = 1;
		break;

	case 2:
		motor_a = FL;
		motor_b = BR;
		ChA_selector = 1;
		ChB_selector = 1;
		break;

	case 3:
		if(distance > 0)
		{
			motor_a = BR;
			motor_b = BL;
		}
		else
		{
			motor_a = FR;
			motor_b = FL;
		}
		ChA_selector = 1;
		ChB_selector = 0;
		break;

	case 4:
		motor_a = FR;
		motor_b = BL;
		ChA_selector = 1;
		ChB_selector = -1;
		break;
	}

	while ( abs(pidStrafe.setpoint - pidStrafe.measured_value) > pidStrafe.delta )
	{
		pidStrafe.measured_value = sgn(pidStrafe.setpoint) * (abs(getMotorEncoder(motor_a)) + abs(getMotorEncoder(motor_b))) / 2;

		tmpJoyStick = PIDControl(pidStrafe);
		if (abs(tmpJoyStick) > maxJoyStick)
			tmpJoyStick = maxJoyStick * sgn(tmpJoyStick);

		iChA_filtered = tmpJoyStick * ChA_selector;
		iChB_filtered = tmpJoyStick * ChB_selector;

		pidEncoder.measured_value = abs(getMotorEncoder(motor_b))-abs(getMotorEncoder(motor_a));
		switch (direction)
		{
		case 1:
			iChA_filtered = PIDControl(pidEncoder);
			break;
		case 3:
			iChB_filtered = PIDControl(pidEncoder);
			break;
		}

		writeDebugStreamLine( "%f %f %f %f %f %f", getTimerValue(T2), iChA_filtered, iChB_filtered, abs(getMotorEncoder(motor_a)), abs(getMotorEncoder(motor_b)), pidEncoder.measured_value );

		//writeDebugStreamLine( "%f %f %f %f %f %f %f %f", getTimerValue(T2), iChA_filtered, iChC_filtered, getGyroDegrees(gyro), getGyroStable(), desired_heading, pidStrafe.setpoint, pidStrafe.measured_value );
		//writeDebugStreamLine( "%f %f %f %f %f %f %f %f %f", getTimerValue(T2), iChA_filtered, abs(getMotorEncoder(FL))-pre_FL, abs(getMotorEncoder(FR))-pre_FR, abs(getMotorEncoder(BL))-pre_BL, abs(getMotorEncoder(BR))-pre_BR, abs(getMotorEncoder(FL))+abs(getMotorEncoder(FR)), abs(getMotorEncoder(BL))+abs(getMotorEncoder(BR)), pidLight.measured_value );

		wait1Msec(dt);
	}
	iChA_filtered = 0;
	iChB_filtered = 0;
}

void turnTo(float heading, float delta)
{
	float tempDelta = pidOrientation.delta;
	pidOrientation.delta = delta;
	pidOrientation.integral = 0;
	pidOrientation.prev_error = 0;

	desired_heading = heading;
	waitUntil( abs(getGyroStable() - desired_heading) <= pidOrientation.delta );
	pidOrientation.delta = tempDelta;
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
					iChA_filtered = -90;
					wait1Msec(350);
					stopDrivetrain();
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
			pidOrientation.measured_value = getGyroStable();
			pidOrientation.setpoint = desired_heading;

			iChC_filtered = -PIDControl(pidOrientation);
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
		turnTo(90,10);
		//waitUntil(getTouchLEDValue(LED));
		strafePID(1, 42, 90, 0.18, 0, 0, 2);
		strafePID(3, 8, 90, 0.18, 0, 0, 2);

		setMotorSpeed(clawMotor, -100);
		setMotorTarget(armMotor, ARM_CARRY, 100);
		waitUntilMotorStop(armMotor);

		desired_heading = 90;
		strafePID(1, -38, 90, 0.18, 0, 0, 2);
		wait1Msec(100);

		strafePID(3, -15, 90, 0.18, 0, 0, 10);
		setMotorTarget(clawMotor, CLAW_RELEASE, 100);
		setMotorTarget(armMotor, iArmLevel[0], 100);

		strafePID(3, -65, 90, 0.18, 0, 0, 2);
		strafePID(1, 46, 90, 0.18, 0, 0, 20);
		strafePID(3, 35, 90, 0.18, 0, 0, 20);
		turnTo(-98,10);
		//desired_heading = -95;
		claw_working = false;
		claw_to_release = true;

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

	pidOrientation.delta = 0.6;
	pidOrientation.kp = 1;
	pidOrientation.ki = 0.00001;
	pidOrientation.kd = 0.0001;

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

		if ( (getJoystickValue(BtnLUp) || getJoystickValue(BtnLDown)) && !claw_working )
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
