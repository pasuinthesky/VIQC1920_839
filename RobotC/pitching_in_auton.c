#pragma config(Sensor, port8,  Gyro,           sensorVexIQ_Gyro)
#pragma config(Sensor, port9,  LED,            sensorVexIQ_LED)
#pragma config(Motor,  motor1,          leftMotor,     tmotorVexIQ, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motor7,          rightMotor,    tmotorVexIQ, PIDControl, reversed, driveRight, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
float dt = 25;
float allowed_time = 7.5;
float iChC_filtered = 0.0;
float iChB_filtered = 0.0;
float iChA_filtered = 0.0;
float desired_heading = 0.0;

bool drive_override = false;
bool task_running = false;

#define GYRO_SAMPLING_SECONDS 5
#define ACCEPTABLE_DRIFT_RANGE 0.05
float fGyroDriftRate;

#define COUNT_PER_ROUND 960
#define WHEEL_TRAVEL 20
#define GEAR_RATIO 2
#define RATE 1

float cmToEncoderUnit(float distance)
{
	return distance * COUNT_PER_ROUND / WHEEL_TRAVEL / GEAR_RATIO / RATE;
}

//47 to 56 creates a new type of struct and names it structPID
typedef struct {
	float setpoint;
	float measured_value;
	float integral;
	float prev_error;
	float kp;
	float ki;
	float kd;
	float delta;
}structPID;

structPID pidDrive
structPID pidOrientation




void setGyroStable()
{
	fGyroDriftRate = 100;

	while (fGyroDriftRate > ACCEPTABLE_DRIFT_RANGE)
	{
		// Let the LED flashing to remind people do not move the robot.
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

void strafePID(int distance, int maxJoyStick, float Kp, float Ki, float Kd, int delta)
{
	int tmpJoyStick, motor_a, motor_b, ChA_selector, ChB_selector;

	pidDrive.setpoint = cmToEncoderUnit(distance);
	pidDrive.measured_value = 0;
	pidDrive.kp = Kp;
	pidDrive.ki = Ki;
	pidDrive.kd = Kd;
	pidDrive.delta = cmToEncoderUnit(delta);
	pidDrive.integral = 0;
	pidDrive.prev_error = 0;

	resetMotorEncoder(leftMotor);
	resetMotorEncoder(rightMotor);

	motor_a = leftMotor;
	motor_b = rightMotor;
	ChA_selector = 1;
	//ChB_selector = 0;

	while ( abs(pidDrive.setpoint - pidDrive.measured_value) > pidDrive.delta )
	{
		pidDrive.measured_value = sgn(pidDrive.setpoint) * (abs(getMotorEncoder(motor_a)) + abs(getMotorEncoder(motor_b))) / 2;

		tmpJoyStick = PIDControl(pidDrive);
		if (abs(tmpJoyStick) > maxJoyStick)
			tmpJoyStick = maxJoyStick * sgn(tmpJoyStick);

		iChA_filtered = tmpJoyStick * ChA_selector;

		//writeDebugStreamLine( "%f %f %f %f %f %f %f %f", getTimerValue(T2), iChA_filtered, iChC_filtered, getGyroDegrees(gyro), getGyroStable(), desired_heading, pidDrive.setpoint, pidDrive.measured_value );
		//writeDebugStreamLine( "%f %f %f %f %f %f %f %f %f", getTimerValue(T2), iChA_filtered, abs(getMotorEncoder(FL))-pre_FL, abs(getMotorEncoder(FR))-pre_FR, abs(getMotorEncoder(BL))-pre_BL, abs(getMotorEncoder(BR))-pre_BR, abs(getMotorEncoder(FL))+abs(getMotorEncoder(FR)), abs(getMotorEncoder(BL))+abs(getMotorEncoder(BR)), pidLight.measured_value );

		wait1Msec(dt);
	}
	iChA_filtered = 0;
}

void turnTo(float heading, float delta)
{
	float tempDelta = pidOrientation.delta;
	pidOrientation.delta = delta;
	pidOrientation.integral = 0;
	pidOrientation.prev_error = 0;

	desired_heading = heading;
	waitUntil( abs(getGyroStable() - desired_heading) <= pidOrientation.delta );
	setTouchLEDRGB(LED, 255, 0, 0);
	pidOrientation.delta = tempDelta;
}

task drive()
{
	while (true)
	{
		pidOrientation.measured_value = getGyroStable();
		pidOrientation.setpoint = desired_heading;

		iChC_filtered = -PIDControl(pidOrientation);
		writeDebugStreamLine( "%f %f %f %f %f %f %f %f", getTimerValue(T2), iChA_filtered, iChC_filtered, getGyroDegrees(gyro), getGyroStable(), desired_heading );
		if(! drive_override)
		{
			setMotorSpeed(leftMotor,(iChA_filtered+iChC_filtered)/1);
			setMotorSpeed(rightMotor,(iChA_filtered-iChC_filtered)/1);
		}
		wait1Msec(dt);
	}
}

void initialize()
{
	setMotorEncoderUnits(encoderCounts);

	setMotorBrakeMode(rightMotor, motorCoast);
	setMotorBrakeMode(leftMotor, motorCoast);

	setTouchLEDColor(LED,colorYellow);
	waitUntil(getTouchLEDValue(LED));

	setMotorBrakeMode(rightMotor, motorHold);
	setMotorBrakeMode(leftMotor, motorHold);
}
/*
void LEDBusiness(int colour, int blinkTimeOn, int blinkTimeOff, int blinkColour, int blink)
{

	timeStamp = time1[T1];
	displayCenteredTextLine( 3, "%f", timeStamp );

	if (blink == 0)
	{
		setTouchLEDColor(LED, colour);
		waitUntil( getTouchLEDValue(LED) == 1 );
	}
	else if (blink ==1)
	{
		setTouchLEDColor(LED,colour);
		waitUntil(getTouchLEDValue(LED) == 1);
		setTouchLEDColor(LED,blinkColour);
		setTouchLEDBlinkTime(LED, blinkTimeOn, blinkTimeOff);
	}

	wait(0.3);
	resetGyro(Gyro);
	targetHeading = 0;
}*/


task main()
{
	initialize();

	setGyroStable();
	setTouchLEDColor(LED,colorBlue);
	waitUntil(getTouchLEDValue(LED));

	setTouchLEDColor(LED,colorGreen);
	clearDebugStream();

	resetGyroStable();
	desired_heading = 0;
	resetTimer(T2);

	pidOrientation.delta = 3;
	pidOrientation.kp = 0.75;
	pidOrientation.ki = 0;
	pidOrientation.kd = 1;

	startTask(drive);
	int i;
	for (i = 1; i<=8; i++)
	{
		strafePID(15, 90, 5, 0, 1, 2);
		turnTo(90*i, 10);
	}
	setTouchLEDRGB(LED, 0, 255, 50);
	// for speed 90 use 5, 0, 1
	// for speed 30 use 5, 2, 1
}
