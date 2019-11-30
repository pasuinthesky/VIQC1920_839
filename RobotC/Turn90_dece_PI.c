#pragma config(Sensor, port4,  Gyro,           sensorVexIQ_Gyro)
#pragma config(Sensor, port8,  LED,            sensorVexIQ_LED)
#pragma config(Motor,  motor1,          leftMotor,     tmotorVexIQ, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motor6,          rightMotor,    tmotorVexIQ, PIDControl, reversed, driveRight, encoder)
#pragma config(Motor,  motor7,          clawMotor,     tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         tailMotor,     tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor12,         armMotor,      tmotorVexIQ, PIDControl, reversed, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define TARGET_ANGLE 90

#define TURN_SPEED 5
#define TURN_DELTA 2

#define Kp 0.6
#define Ki 0

#define WAIT_CYCLE 20

task main()
{

	float error = 0, integral = 0, output;

	wait(1);
	clearTimer(T1);
	resetGyro(Gyro);
	setMotorEncoderUnits(encoderCounts);
	resetMotorEncoder(leftMotor);
	resetMotorEncoder(rightMotor);
	setMotorBrakeMode(leftMotor, motorHold);
	setMotorBrakeMode(rightMotor, motorHold);

	while ( getGyroDegreesFloat(Gyro) < (TARGET_ANGLE - TURN_DELTA) )
	{
		error = getGyroDegreesFloat(Gyro) - TARGET_ANGLE;
		integral = integral + error;
		output = error * Kp + integral * Ki;

		setMotorSpeed(leftMotor, sgn(error) * TURN_SPEED + output);
		setMotorSpeed(rightMotor, -sgn(error) * TURN_SPEED - output);

		//writeDebugStreamLine("%d:  %d,  %d,  %d", time1[T1], error, integral, output);
		wait1Msec(WAIT_CYCLE);
	}
		setMotorSpeed(leftMotor, 0);
		setMotorSpeed(rightMotor, 0);
}