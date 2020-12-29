#pragma config(Sensor, port10, LED,            sensorVexIQ_LED)
#pragma config(Sensor, port12, gyro,           sensorVexIQ_Gyro)
#pragma config(Motor,  motor1,          BR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor4,          clawMotor,     tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor5,          BL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor6,          FL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor7,          FR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         armMotor,      tmotorVexIQ, PIDControl, reversed, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

float desired_distance;
float dt = 25;
float allowed_time = 7.5;
float iChC_filtered = 0.0;
float iChB_filtered = 0.0;
float iChA_filtered = 0.0;
float progressToGoal = getMotorEncoder(BL) * 1.414;
float desired_heading = 0.0;

#define GYRO_SAMPLING_SECONDS 10000
#define ACCEPTABLE_DRIFT_RANGE 0.08
float fGyroDriftRate;

#define COUNT_PER_ROUND 960
#define WHEEL_TRAVEL 20.0
#define GEAR_RATIO 2
#define RATE 1

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

structPID pidDistance;
structPID pidOrientation;

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

task orientationPID()
{
	while (true)
	{
		pidOrientation.measured_value = getGyroStable();
		pidOrientation.setpoint = desired_heading;

		iChC_filtered = -PIDControl(pidOrientation);
		writeDebugStreamLine( "%f %f %f %f %f %f %f %f", getTimerValue(T2), iChA_filtered, iChC_filtered, getGyroDegrees(gyro), getGyroStable(), desired_heading );
		wait1Msec(dt);
	}
}

void DrivePID(int distance, int maxJoyStick, float Kp, float Ki, float Kd, int delta)
{
	pidDistance.setpoint = cmToEncoderUnit(distance) / sqrt(2);
	pidDistance.measured_value = 0;
	pidDistance.kp = Kp;
	pidDistance.ki = Ki;
	pidDistance.kd = Kd;
	pidDistance.delta = cmToEncoderUnit(delta);

	while ( abs(pidDistance.setpoint - pidDistance.measured_value) > pidDistance.delta )
	{
		pidDistance.measured_value = (abs(getMotorEncoder(BL)) + abs(getMotorEncoder(BR))) / 2;

		iChA_filtered = PIDControl(pidDistance);
		if (iChA_filtered > maxJoyStick)
			iChA_filtered = maxJoyStick;
		iChB_filtered = 0;
		writeDebugStreamLine( "%f %f %f %f %f %f %f %f", getTimerValue(T2), iChA_filtered, iChC_filtered, getGyroDegrees(gyro), getGyroStable(), desired_heading, pidDistance.setpoint, pidDistance.measured_value );
		wait1Msec(dt);
	}
	iChA_filtered = 0;
	iChB_filtered = 0;
}

task drive()
{
	while (true)
	{
		setMotorSpeed( FL, 0 + iChA_filtered + iChB_filtered - iChC_filtered );
		setMotorSpeed( BL, 0 + iChA_filtered - iChB_filtered - iChC_filtered );
		setMotorSpeed( FR, 0 - iChA_filtered + iChB_filtered - iChC_filtered );
		setMotorSpeed( BR, 0 - iChA_filtered - iChB_filtered - iChC_filtered );
		wait1Msec(dt);
	}
}

// remember these values for orientation: DriveStraightPID(20, 0.7, 0, 0, 5)
task main()
{
	setGyroStable();
	resetGyroStable();

	setMotorBrakeMode(BL, motorHold);
	setMotorBrakeMode(FL, motorHold);
	setMotorBrakeMode(BR, motorHold);
	setMotorBrakeMode(FR, motorHold);
	resetTimer(T2);
	setMotorEncoderUnits(encoderCounts);

	startTask(orientationPID);

	clearDebugStream();

	pidOrientation.delta = 0.6;
	pidOrientation.kp = 1;
	pidOrientation.ki = 0.00001;
	pidOrientation.kd = 0.0001;

	startTask(drive);
	DrivePID(28, 90, 0.18, 0, 0, 1);
	//desired_heading = 90;
	pidOrientation.delta = 1;
	waitUntil( abs(getGyroStable() - desired_heading) <= pidOrientation.delta );
	stopAllTasks();
}
