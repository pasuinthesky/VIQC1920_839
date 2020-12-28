#pragma config(Sensor, port10, LED,            sensorVexIQ_LED)
#pragma config(Sensor, port12, gyro,           sensorVexIQ_Gyro)
#pragma config(Motor,  motor1,          BR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor5,          BL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor6,          FL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor7,          FR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         liftMotorR,    tmotorVexIQ, PIDControl, encoder)
//ARE YA READY KIDS? AY AY CAPTAIN! I CAAANNNNNNNNNNNNNNNNNNNT HEAAAAAAAR YOUUUUUUUUUUUUU! AYY AYYY CAPTAIN! WHO LIVES IN A PINEAPPLE UNDER THE SEA? SPONGEBOB SQUARE PANTS! ABOSRBENT AND YELLOW AND POROUS IS HE? SPONGEBOB SQUAREPANTS! IF NAUTICAL NONSENSE BE SOMETHING YOU WISH? SPONGEBOB SQUAREPANTS! THEN DROP ON THE DECK AND FLOP LIKE A FISH! SPONGEBOB SQUAREPANTS! READY? SPONGEBOB SQUAREPANTS! SPONGEBOB SQUAREPANTS! SPONGEBOB SQUAREPANTS!
// turn this into claw later#pragma config(Motor,  motor4,          liftMotorL,    tmotorVexIQ, PIDControl, reversed, encoder)
float desired_distance;
float dt = 25;
float allowed_time = 7.5;
float iChC_filtered = 0.0;
float iChB_filtered = 0.0;
float iChA_filtered = 0.0;
float progressToGoal = getMotorEncoder(BL) * 1.414;
float desired_heading;

#define GYRO_SAMPLING_SECONDS 10000
#define ACCEPTABLE_DRIFT_RANGE 0.08
float fGyroDriftRate;

#define COUNT_PER_ROUND 960
#define WHEEL_TRAVEL 20.0
#define GEAR_RATIO 2
#define RATE 1

float cmToEncoderUnit(float distance)
{
	return distance * COUNT_PER_ROUND * 1.414 / WHEEL_TRAVEL / GEAR_RATIO / RATE;
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
	pidOrientation.delta = 5;
	pidOrientation.kp = 0.7;
	pidOrientation.ki = 0;
	pidOrientation.kd = 0;
	//int speed
	while (true)
	{
		pidOrientation.measured_value = getGyroStable();
		pidOrientation.setpoint = desired_heading;

		iChC_filtered = PIDControl(pidOrientation);
//		writeDebugStreamLine( "%f %f %f %f", iChA_filtered, cmToEncoderUnit(distance), pidOrientation.measured_value, pidOrientation.setpoint - pidOrientation.measured_value );
		wait1Msec(dt);
	}
}

void DrivePID(int distance, float Kp, float Ki, float Kd, int delta)
{
	//int speed
	while (abs(cmToEncoderUnit(distance) - getMotorEncoder(BL) * 1.414) > delta)
	{
		pidDistance.delta = cmToEncoderUnit(delta);
		pidDistance.kp = Kp;
		pidDistance.ki = Ki;
		pidDistance.kd = Kd;
		pidDistance.measured_value = getMotorEncoder(BL) * 1.414;
		pidDistance.setpoint = cmToEncoderUnit(distance);


		pidOrientation.delta = delta;
		pidOrientation.kp = Kp;
		pidOrientation.ki = Ki;
		pidOrientation.kd = Kd;
		pidOrientation.measured_value = getGyroStable();
		pidOrientation.setpoint = desired_heading;

		iChA_filtered = PIDControl(pidDistance);
		iChB_filtered = 0;
		iChC_filtered = PIDControl(pidOrientation);
		writeDebugStreamLine( "%f %f %f %f", iChA_filtered, cmToEncoderUnit(distance), pidDistance.measured_value, distance - pidDistance.measured_value );
		wait1Msec(dt);
	}
		iChA_filtered = 0;
		iChB_filtered = 0;
		iChC_filtered = 0;
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
	startTask(orientationPID)
	clearDebugStream();
	setMotorEncoderUnits(encoderCounts);
	startTask(drive);
	DrivePID(40, 0.15, 0, 0, 2);
}
