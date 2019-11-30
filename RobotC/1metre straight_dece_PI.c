#pragma config(Sensor, port4,  Gyro,           sensorVexIQ_Gyro)
#pragma config(Sensor, port8,  LED,            sensorVexIQ_LED)
#pragma config(Motor,  motor1,          leftMotor,     tmotorVexIQ, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motor6,          rightMotor,    tmotorVexIQ, PIDControl, reversed, driveRight, encoder)
#pragma config(Motor,  motor7,          clawMotor,     tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         tailMotor,     tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor12,         armMotor,      tmotorVexIQ, PIDControl, reversed, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


#define WAIT_CYCLE 20
#define MIN_SPEED 20
#define COUNT_PER_ROUND 960
#define GEAR_RATIO 1.5
#define DECEL_RATE 0.75
#define SLOW_ZONE_PERCENT 40

void Initialize()
// Prepare robot
{
	wait(1);
	clearTimer(T1);
	resetGyro(Gyro);
	setMotorEncoderUnits(encoderCounts);
	resetMotorEncoder(leftMotor);
	resetMotorEncoder(rightMotor);
	setMotorBrakeMode(leftMotor, motorHold);
	setMotorBrakeMode(rightMotor, motorHold);

}

void goStraight(int maxSpeed, float Ki, float Kp, int distance)
//For no load: Ki = 0.0026, Kp = 0.5
{
	float motorSpeed;
	float error = 0, integral = 0, output;
	int currentCount = 0;
	int targetCount = distance * COUNT_PER_ROUND / 20.0 / GEAR_RATIO;
	float slowZone = targetCount * SLOW_ZONE_PERCENT / 100;

	writeDebugStreamLine("targetCount = %d", targetCount);
	writeDebugStreamLine("slowZone = %d", slowZone);

	Initialize();

	while ( currentCount < targetCount )
	{
		error = getGyroDegreesFloat(Gyro) - 0;
		integral = integral + error;
		output = error * Kp + integral * Ki;

    if ( currentCount < ( targetCount - slowZone ) )
		{
			motorSpeed = maxSpeed;
		}
		else
		{
			motorSpeed = MIN_SPEED + DECEL_RATE * ( maxSpeed - MIN_SPEED ) * ( targetCount - currentCount ) / slowZone;
		}

		setMotorSpeed(leftMotor, motorSpeed + output);
		setMotorSpeed(rightMotor, motorSpeed - output);

		currentCount = ( getMotorEncoder(leftMotor) + getMotorEncoder(rightMotor) ) / 2;
		writeDebugStreamLine("%d:  %f,  %f,  %f,  %f,  %f", time1[T1], error, integral, output, motorSpeed, currentCount);
//		writeDebugStreamLine("%d: error %d, integral %d, output %d, motorSpeed %d, currentCount %d", time1[T1], error, integral, output, motorSpeed, currentCount);
		wait1Msec(WAIT_CYCLE);
	}
		setMotorSpeed(leftMotor, 0);
		setMotorSpeed(rightMotor, 0);
}

task main()
{
	goStraight(90, 0.0028, 0.5, 100);
}