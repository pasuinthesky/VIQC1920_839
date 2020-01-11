#pragma config(Sensor, port4,  Gyro,           sensorVexIQ_Gyro)
#pragma config(Sensor, port8,  LED,            sensorVexIQ_LED)
#pragma config(Motor,  motor1,          leftMotor,     tmotorVexIQ, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motor6,          rightMotor,    tmotorVexIQ, PIDControl, reversed, driveRight, encoder)
#pragma config(Motor,  motor7,          clawMotor,     tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         tailMotor,     tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor12,         armMotor,      tmotorVexIQ, PIDControl, reversed, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define RATE 1.05
#define WAIT_CYCLE 20
#define MIN_SPEED 30
#define COUNT_PER_ROUND 960
#define GEAR_RATIO 1.5
#define DECEL_RATE 0.75
#define WHEEL_TRAVEL 20.0

#define ON_SPOT_TURN 0
#define LEFT_WHEEL_TURN 1
#define RIGHT_WHEEL_TURN 2
#define TWO_WHEEL_TURN 3

#define ARM_SPEED 100
#define TAIL_SPEED 60
#define CLAW_SPEED 100

#define CLAW_OPEN -120
#define CLAW_CLOSE -450
#define CLAW_DELTA 65

#define TAIL_UP 2000
#define TAIL_DOWN 960
#define TAIL_DELTA 65

#define ARM_DELTA 65
#define MS_PER_ENCODER_UNIT 1
#define MS_PER_DEGREE_ROBOT_TURN 12


int currentCount, pickUpTrigger, targetHeading, cubeLevel;
int iArmLv[4] = {105, 400, 1100, 1455};

int tailTarget;
float timeStamp;

void reset_gyro_timer_encoder()
// Prepare robot
{
	wait(1);
	resetGyro(Gyro);

	clearTimer(T1);

	resetMotorEncoder(leftMotor);
	resetMotorEncoder(rightMotor);
	setMotorBrakeMode(leftMotor, motorHold);
	setMotorBrakeMode(rightMotor, motorHold);

	targetHeading = 0;

}

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
	resetMotorEncoder(leftMotor);
	resetMotorEncoder(rightMotor);

	clearTimer( T2 );

	while ( currentCount < targetCount && time1[T2] < ( targetCount * MS_PER_ENCODER_UNIT * 100 / MIN_SPEED ) )
	{
		error = getGyroDegreesFloat(Gyro) - targetHeading;
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

		currentCount = abs( getMotorEncoder(leftMotor) + getMotorEncoder(rightMotor) ) / 2;
		displayCenteredBigTextLine(3, "%d, %d", getGyroDegrees(Gyro), targetHeading);
		wait1Msec(WAIT_CYCLE);
	}
		setMotorSpeed(leftMotor, 0);
		setMotorSpeed(rightMotor, 0);
}

task moveTail()
{
	setMotorTarget( tailMotor, tailTarget, 100 );
	wait1Msec( abs( TAIL_UP - TAIL_DOWN ) * MS_PER_ENCODER_UNIT );
	EndTimeSlice();
}

void clawAction(int clawTarget)
{
	setMotorTarget( clawMotor, clawTarget, 100 );
	wait1Msec( abs( CLAW_CLOSE - CLAW_OPEN ) * MS_PER_ENCODER_UNIT );
	setMotorSpeed( clawMotor, 0 );
}

task returnArm()
{
	setMotorTarget(armMotor,iArmLv[0],100);
	wait1Msec( abs( getMotorEncoder(armMotor) - iArmLv[0] ) * MS_PER_ENCODER_UNIT );
	EndTimeSlice();
}

void drop()
{
	clawAction( CLAW_OPEN );
	if ( getMotorEncoder(armMotor) > ( iArmLv[2] - ARM_DELTA ) )
	{
		// This block should had been done by goStraightDecel function instead.
		setMotorSpeed(leftMotor,-100);
		setMotorSpeed(rightMotor,-100);
		wait1Msec(100);
		setMotorSpeed(leftMotor,0);
		setMotorSpeed(rightMotor,0);
	}
	startTask(returnArm);
}

task pick_up_cube()
{
	writeDebugStreamLine( "%f: %f, %f", time1[T1], currentCount, cmToEncoderUnit( pickUpTrigger ) );
	currentCount = 0;
	while ( currentCount < cmToEncoderUnit( pickUpTrigger ) )
		{
			writeDebugStreamLine( "%f: %f, %f", time1[T1], currentCount, cmToEncoderUnit( pickUpTrigger ) );
			wait1Msec( WAIT_CYCLE );
	}

	clawAction( CLAW_CLOSE );
	setMotorTarget( armMotor, iArmLv[cubeLevel], 100 );
	wait1Msec( abs( iArmLv[cubeLevel] - getMotorEncoder( armMotor ) ) * MS_PER_ENCODER_UNIT );
	setMotorSpeed( armMotor, 0 );
	EndTimeSlice();
}

void turnDecel( int inputHeading, int turnStyle, float Ki, float Kp, int baseSpeed, int delta )
{
	float error = 0, integral = 0, output;
	int degreeTurning = abs( getGyroDegrees( Gyro ) - inputHeading );

	targetHeading = inputHeading;
	clearTimer( T3 );

	while ( abs( getGyroDegreesFloat(Gyro) - targetHeading ) > delta && time1[T3] < ( degreeTurning * MS_PER_DEGREE_ROBOT_TURN * 100 / MIN_SPEED ) )
	{
		error = getGyroDegreesFloat(Gyro) - targetHeading;
		integral = integral + error;
		output = error * Kp + integral * Ki;

		switch ( turnStyle )
		{
			case ON_SPOT_TURN:
				setMotorSpeed(leftMotor, sgn(error) * baseSpeed + output);
				setMotorSpeed(rightMotor, -sgn(error) * baseSpeed - output);
				break;
			case LEFT_WHEEL_TURN:
				setMotorSpeed(leftMotor, sgn(error) * baseSpeed + output);
				setMotorSpeed(rightMotor, 0);
				break;
			case RIGHT_WHEEL_TURN:
				setMotorSpeed(leftMotor, 0);
				setMotorSpeed(rightMotor, -sgn(error) * baseSpeed - output);
				break;
			case TWO_WHEEL_TURN:
				setMotorSpeed(leftMotor, baseSpeed + output);
				setMotorSpeed(rightMotor, baseSpeed - output);
				break;
		}

		//writeDebugStreamLine("%d:  %d,  %d,  %d", time1[T1], error, integral, output);
		wait1Msec(WAIT_CYCLE);
	}
		setMotorSpeed(leftMotor, 0);
		setMotorSpeed(rightMotor, 0);
}

void greenCubeLow()
{
	goStraightDecel(18, 90, 0.0026, 0.5, 20);

	pickUpTrigger = 3; cubeLevel = 2;
	startTask( pick_up_cube );
	goStraightDecel(20, 30, 0.0026, 0.5, 0);

	goStraightDecel(28, 60, 0.003, 0.6, 30);

	//goStraightDecel(43, 90, 0.003, 0.6, 40);
	drop();

}


void greenCubeLeftHighRight()
{
	greenCubeLow();

	turnDecel( -45, RIGHT_WHEEL_TURN, 0, 0.45, 20,  2);

	goStraightDecel(40, 100, 0.0026, 0.5, 5);
	turnDecel(-90, LEFT_WHEEL_TURN, 0, 0.45, 20, 2);
	goStraightDecel(20, 80, 0.0026, 0.5, 5);
	turnDecel(-180, ON_SPOT_TURN, 0, 0.6, 5, 2);
/*
	while (true)
		displayCenteredBigTextLine( 3, "%d, %d", getGyroDegrees(Gyro), targetHeading );
*/
	pickUpTrigger = 4; cubeLevel = 3;
	startTask(pick_up_cube);
	goStraightDecel(38, 30, 0.003, 0.5, 30);
	goStraightDecel(20, 70, 0.003, 0.5, 30);

	drop();
	goStraightDecel(5, -60, 0.003, 0.5, 30);

	turnDecel( -135, ON_SPOT_TURN, 0, 0.6, 5,  2);

	goStraightDecel(50, 100, 0.0026, 0.5, 30);

	turnDecel(-90, RIGHT_WHEEL_TURN, 0, 0.45, 20, 5);
	goStraightDecel(18, 80, 0.0026, 0.5, 40);
	turnDecel	(0, ON_SPOT_TURN, 0, 0.3, 5, 2);


	pickUpTrigger = 10; cubeLevel = 2;
	startTask(pick_up_cube);
	goStraightDecel(20, 30, 0.003, 0.5, 30);
	goStraightDecel(30, 100, 0.003, 0.5, 30);

	drop();

}

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
}

void cornerCubes()
{
	goStraightDecel(48, 90, 0.0026, 0.5, 20);
	turnDecel( -132, ON_SPOT_TURN, 0, 0.6, 5, 2 );

	pickUpTrigger = 23; cubeLevel = 1;
	startTask( pick_up_cube );
	goStraightDecel(24, 90, 0.0026, 0.5, 20);

	turnDecel( -200, RIGHT_WHEEL_TURN, 0, 0.3, 20, 2 );

	goStraightDecel(45, -90, 0.0026, 0.5, 20);

	tailTarget = TAIL_UP;
	startTask( moveTail );
	turnDecel( -237, ON_SPOT_TURN, 0, 0.3, 5, 2 );
	goStraightDecel(195, 100, 0.003, 0.6, 30);

	drop();
	goStraightDecel(10, -60, 0.003, 0.6, 20);

	turnDecel( -145, LEFT_WHEEL_TURN, 0, 0.3, 20, 2 );

	goStraightDecel(70, -100, 0.003, 0.6, 0);

	tailTarget = TAIL_DOWN;
	startTask( moveTail );
	turnDecel( -170, LEFT_WHEEL_TURN, 0, 0.25, 20, 2 );

	goStraightDecel(20, -100, 0.003, 0.6, 0);

	targetHeading = -190;
	goStraightDecel(10, 40, 0.0026, 0.5, 0);

	pickUpTrigger = 28.5; cubeLevel = 2;
	startTask( pick_up_cube );
	goStraightDecel( 30, 90, 0.0026, 0.5, 30 );

	goStraightDecel( 20, -90, 0.0026, 0.5, 10 );
	turnDecel( -225, RIGHT_WHEEL_TURN, 0, 0.3, 20, 2 );
	goStraightDecel( 10, -60, 0.003, 0.6, 10 );

	turnDecel( -230, RIGHT_WHEEL_TURN, 0, 0.3, 5, 2 );
	goStraightDecel( 5, -90, 0.003, 0.6, 30 );

	setMotorTarget(armMotor,iArmLv[1],100);


	wait1Msec( abs( getMotorEncoder(armMotor) - iArmLv[1] ) * MS_PER_ENCODER_UNIT );

	tailTarget = TAIL_UP;
	startTask( moveTail );
	turnDecel( -120, TWO_WHEEL_TURN, 0.001, 0.3, 30, 2 );
	goStraightDecel( 165, 100, 0.003, 0.6, 20 );

	drop();
	goStraightDecel( 10, -60, 0.003, 0.6, 20 );

	turnDecel( -220, RIGHT_WHEEL_TURN, 0, 0.3, 20, 2 );

	goStraightDecel( 50, -90, 0.003, 0.6, 20 );

	turnDecel( -200, RIGHT_WHEEL_TURN, 0, 0.3, 30, 5 );
	tailTarget = TAIL_DOWN;
	startTask( moveTail );
	goStraightDecel( 25, -50, 0.003, 0.6, 0 );

	goStraightDecel( 25, 100, 0.026, 0.5, 0 );

}

void cornerCubes2()
{
	goStraightDecel(48, 90, 0.0026, 0.5, 20);
	turnDecel( -132, ON_SPOT_TURN, 0, 0.6, 5, 2 );

	pickUpTrigger = 22.5; cubeLevel = 1;
	startTask( pick_up_cube );
	goStraightDecel(24, 90, 0.0026, 0.5, 30);

	turnDecel( -200, RIGHT_WHEEL_TURN, 0, 0.3, 20, 2 );

	goStraightDecel(45, -90, 0.0026, 0.5, 20);

	tailTarget = TAIL_UP;
	startTask( moveTail );
	turnDecel( -237, ON_SPOT_TURN, 0, 0.3, 5, 2 );
	goStraightDecel(195, 100, 0.003, 0.6, 30);

	drop();
	goStraightDecel(10, -60, 0.003, 0.6, 20);

	iArmLv[0] = 560;
	iArmLv[1] = 750;
	startTask( returnArm );

	turnDecel( -145, LEFT_WHEEL_TURN, 0, 0.3, 20, 2 );

	goStraightDecel(70, -90, 0.003, 0.6, 0);

	tailTarget = TAIL_DOWN;
	startTask( moveTail );
	turnDecel( -170, LEFT_WHEEL_TURN, 0, 0.25, 20, 2 );

	setMotorSpeed(leftMotor,-100);
	setMotorSpeed(rightMotor,-100);
	wait1Msec(800);
	setMotorSpeed(leftMotor,0);
	setMotorSpeed(rightMotor,0);

//	waitUntil( getTouchLEDValue(LED) == 1 );

	turnDecel( -180, ON_SPOT_TURN, 0, 0.3, 20, 2 );

	pickUpTrigger = 36; cubeLevel = 1;
	startTask( pick_up_cube );
	goStraightDecel( 38, 90, 0.0026, 0.5, 30 );

	turnDecel( -225, ON_SPOT_TURN, 0, 0.3, 60, 2 );
//	waitUntil( getTouchLEDValue(LED) == 1 );

	goStraightDecel( 40, -90, 0.003, 0.6, 10 );

	//waitUntil( getTouchLEDValue(LED) == 1 );
	tailTarget = TAIL_UP;
	startTask( moveTail );
	turnDecel( -120, TWO_WHEEL_TURN, 0.001, 0.3, 30, 2 );
	goStraightDecel( 165, 100, 0.003, 0.6, 20 );

	drop();
	goStraightDecel( 10, -60, 0.003, 0.6, 20 );

	turnDecel( -220, RIGHT_WHEEL_TURN, 0, 0.3, 20, 2 );

	goStraightDecel( 55, -90, 0.003, 0.6, 20 );

	turnDecel( -210, RIGHT_WHEEL_TURN, 0, 0.3, 30, 5 );
	tailTarget = TAIL_DOWN;
	startTask( moveTail );
	goStraightDecel( 18, -100, 0.003, 0.6, 0 );

	goStraightDecel( 25, 100, 0.026, 0.5, 0 );


//	while (true)
		//displayCenteredBigTextLine( 3, "%d, %d", getGyroDegrees(Gyro), targetHeading );


}

void reset_arm_tail_claw()
{
	setMotorTarget( clawMotor, CLAW_OPEN, 100 );
	setMotorTarget( tailMotor, TAIL_DOWN, 100 );
	setMotorTarget( armMotor, iArmLv[0], 100 );
	wait( 0.7 );
}

task main()
{
	setMotorEncoderUnits( encoderCounts );

	setMotorSpeed( clawMotor, 100 );
	setMotorSpeed( tailMotor, -100 );
	setMotorSpeed( armMotor, -100 );
	wait(2);
	resetMotorEncoder( clawMotor );
	resetMotorEncoder( tailMotor );
	resetMotorEncoder( armMotor );

	reset_arm_tail_claw();
	reset_gyro_timer_encoder();

	LEDBusiness( colorGreen, 0, 0, 0, 0 );
	greenCubeLeftHighRight();

	goStraightDecel(40, -100, 0.0026, 0.5, 0);

	LEDBusiness( colorGreen, 0, 0, 0, 0 );
	cornerCubes2();
	reset_arm_tail_claw();

}
