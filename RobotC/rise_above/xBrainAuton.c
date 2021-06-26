#pragma config(Sensor, port10, LED,            sensorVexIQ_LED)
#pragma config(Sensor, port12, gyro,           sensorVexIQ_Gyro)
#pragma config(Motor,  motor1,          BR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor4,          liftMotorL,    tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor5,          BL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor6,          FL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor7,          FR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         liftMotorR,    tmotorVexIQ, PIDControl, encoder)
#define MIN_SPEED 30
#define GEAR_RATIO 2
#define DECEL_RATE 0.75
#define WHEEL_TRAVEL 20.0
int currentCount, pickUpTrigger, targetHeading;
#define COUNT_PER_ROUND 960
#define MS_PER_ENCODER_UNIT 1
#define RATE 1.05

float cmToEncoderUnit(float distance)
{
	return distance * COUNT_PER_ROUND / WHEEL_TRAVEL / GEAR_RATIO / RATE;
}

void goStraightDecel(int distance, int maxSpeed, float Ki, float Kp, int slowZonePercent)
//For no load: Ki = 0.0026, Kp = 0.5
{
	float motorSpeed;
	float error = 0, integral = 0, output;
	int targetCount = cmToEncoderUnit(distance);
	float slowZone = targetCount * slowZonePercent / 100;

	writeDebugStreamLine("targetCount = %d", targetCount);
	writeDebugStreamLine("slowZone = %d", slowZone);

	currentCount = 0;
	resetMotorEncoder(FL);
	resetMotorEncoder(FR);
	resetMotorEncoder(BL);
	resetMotorEncoder(BR);

	clearTimer( T2 );

	while ( currentCount < targetCount && time1[T2] < ( targetCount * MS_PER_ENCODER_UNIT * 100 / MIN_SPEED ) )
	{
		error = getgyroDegreesFloat(gyro) - targetHeading;
		integral = integral + error;
		output = error * Kp + integral * Ki;

    if ( currentCount < ( targetCount - slowZone ) )
		{
			motorSpeed = maxSpeed;
		}
		else
		{
			motorSpeed = sgn(maxSpeed) * MIN_SPEED + DECEL_RATE * ( maxSpeed - sgn(maxSpeed) * MIN_SPEED ) * ( targetCount - currentCount ) / slowZone;
		}

		setMotorSpeed(leftMotor, motorSpeed + output);
		setMotorSpeed(rightMotor, motorSpeed - output);
		setMotorSpeed(BL, motorSpeed )

		currentCount = abs( getMotorEncoder(leftMotor) + getMotorEncoder(rightMotor) ) / 2;
//		writeDebugStreamLine("%d:  %f,  %f,  %f,  %f,  %f", time1[T1], error, integral, output, motorSpeed, currentCount);
//		writeDebugStreamLine("%d: error %d, integral %d, output %d, motorSpeed %d, currentCount %d", time1[T1], error, integral, output, motorSpeed, currentCount);
//		displayCenteredBigTextLine(3, "%d, %d", currentCount, targetCount);
		displayCenteredBigTextLine(3, "%d, %d", getgyroDegrees(gyro), targetHeading);
		wait1Msec(WAIT_CYCLE);
	}
		setMotorSpeed(leftMotor, 0);
		setMotorSpeed(rightMotor, 0);
}

void PIDControl (float Kp, float Ki, float Kd, float delta)
{
	while (true)
	{
		float measured_value = getgyroHeadingFloat(gyro);
		int error = setpoint - measured_value;
		integral = integral + error * dt;
		float derivative = ( error - prev_error ) / dt;
		float output = Kp * error + Ki * integral + Kd * derivative;
		turn(output);
		prev_error = error;
		if ( error <= delta || time1[T1] > allowed_time )
		{
			break;
		}
		wait1Msec ( dt );
	}

}

task main()
{

}
