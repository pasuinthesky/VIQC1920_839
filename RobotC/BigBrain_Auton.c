#pragma config(Sensor, port7,  Gyro,           sensorVexIQ_Gyro)
#pragma config(Sensor, port1,  LED,            sensorVexIQ_LED)
#pragma config(Motor,  motor2,          rightMotor,    tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor5,          liftMotorR,    tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor6,          clawMotor,     tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor8,          leftMotor,     tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         liftMotorL,    tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor12,         scoopMotor,    tmotorVexIQ, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define CLAW_CLOSE 450
#define CLAW_OPEN 50
#define CLAW_DELTA 20
#define SCOOPER_DELTA 35
#define LIFT_DELTA 25
#define MS_PER_ENCODER_UNIT 1
#define ENCODER_UNIT_PER_SCOOP_ROUND 1920
#define FLIP_UP_LIFT 1250
#define FLIP_DOWN_LIFT 900
#define FLIP_UP_SCOOPER -40
#define FLIP_DOWN_SCOOPER 0

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

#define SCOOPER_SCOOP_BALL 0
#define SCOOPER_SCORE_BALL 1
#define SCOOPER_PICKUP_CUBE 2
#define SCOOPER_RELEASE_CUBE 3

#define MS_PER_DEGREE_ROBOT_TURN 12

#define GYRO_SAMPLEING_SECONDS 10
#define ACCEPTABLE_DRIFT_RANGE 0.10

float fGyroDriftRate;
int currentCount, pickUpTrigger, clawTarget, targetHeading, liftLevel;

float timeStamp;

#define LIFT_LEVELS 5
int iLiftLevel[LIFT_LEVELS] = {15 ,420, 1100, 1500, 1830}; //pickup_inside, pickup_outside/transport,low_inside,low_outside,,high_inside

int iScoopPos[4] = { -300, 280, 815, 1000 }; //pick_cube, keep_ball/release cube, ready_to_scoop_ball, score_ball

int turnNumber = 0;

void iCheckBattLevel(float min, float closemin){
	float nBattLevel = 0;
	float BattLevel = 0;
	setTouchLEDRGB(LED,255, 145, 0);
	setTouchLEDBlinkTime(LED, 5, 5);
	for(int i = 0; i<1000; i++){
		BattLevel = BattLevel+(nImmediateBatteryLevel/1000);
		wait1Msec(10);
	}
	setTouchLEDColor(LED, colorNone);
	setTouchLEDBlinkTime(LED, 0, 0);
	nBattLevel = BattLevel/1000;
	displayCenteredBigTextLine(3, "%f", nBattLevel);
	if(nBattLevel<min){
		setTouchLEDRGB(LED,255, 0, 0);
		waitUntil(getTouchLEDValue(LED)==1);
		setTouchLEDColor(LED, colorNone);
		}else if(nBattLevel<=closemin){
		setTouchLEDRGB(LED,255, 125, 125);
		waitUntil(getTouchLEDValue(LED)==1);
		setTouchLEDColor(LED, colorNone);
	}
}

void setGyroStable()
{
	fGyroDriftRate = 100;

	while (fGyroDriftRate > ACCEPTABLE_DRIFT_RANGE)
	{
		setTouchLEDColor(LED,colorRed);
		setTouchLEDBlinkTime(LED, 14,6);
		wait1Msec(3000);
		setTouchLEDBlinkTime(LED, 0,1);
		wait1Msec(2000);

		setTouchLEDColor(LED,colorGreen);
		setTouchLEDBlinkTime(LED, 8, 12);

		resetGyro(Gyro); targetHeading = 0;
		clearTimer(T4);
		wait1Msec(GYRO_SAMPLEING_SECONDS*1000);

		fGyroDriftRate = getGyroDegreesFloat(Gyro) / GYRO_SAMPLEING_SECONDS;

		if (fGyroDriftRate < ACCEPTABLE_DRIFT_RANGE)
			setTouchLEDColor(LED, colorGreen);
		else
			setTouchLEDColor(LED, colorRed);

		setTouchLEDBlinkTime(LED, 4, 4);
		wait1Msec(100);
		setTouchLEDColor(LED,colorNone);
		setTouchLEDBlinkTime(LED, 1, 0);
	}

}

float getGyroStable()
{
	return getGyroDegreesFloat(Gyro) - fGyroDriftRate * time1[T4] / 1000;
}

void resetGyroStable()
{
	resetGyro(Gyro);
	clearTimer(T4);
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
		error = getGyroStable() - targetHeading;
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
		//		writeDebugStreamLine("%d:  %f,  %f,  %f,  %f,  %f", time1[T1], error, integral, output, motorSpeed, currentCount);
		//		writeDebugStreamLine("%d: error %d, integral %d, output %d, motorSpeed %d, currentCount %d", time1[T1], error, integral, output, motorSpeed, currentCount);
		//		displayCenteredBigTextLine(3, "%d, %d", currentCount, targetCount);
		displayCenteredBigTextLine(3, "%d, %d", getGyroStable(), targetHeading);
		wait1Msec(WAIT_CYCLE);
	}
	setMotorSpeed(leftMotor, 0);
	setMotorSpeed(rightMotor, 0);
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

void LEDBusiness(int colour, int blinkTimeOn, int blinkTimeOff, int blinkColour, int blink)
{

	timeStamp = time1[T1];
	displayCenteredBigTextLine( 3, "%f", timeStamp );

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

	wait1Msec(200);
	resetGyroStable();
	targetHeading = 0;
}

void clawAction(int targetName)
{
	setMotorTarget( clawMotor, targetName, 100 );
	wait1Msec( abs( CLAW_CLOSE - CLAW_OPEN ) * MS_PER_ENCODER_UNIT );
	setMotorSpeed( clawMotor, 0 );
}

void scoreBalls()
{
	setMotorSpeed(leftMotor,30);
	setMotorSpeed(rightMotor,30);

	setMotorTarget(scoopMotor,turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[3],100);
	wait1Msec( abs( iScoopPos[3] - iScoopPos[1] ) * MS_PER_ENCODER_UNIT );
	wait1Msec(100);
	setMotorSpeed(leftMotor,0);
	setMotorSpeed(rightMotor,0);
	setMotorTarget(scoopMotor,turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[2],100);
	wait1Msec( abs( iScoopPos[3] - iScoopPos[2] ) * MS_PER_ENCODER_UNIT );
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

	clawAction( clawTarget );
	setMotorTarget( liftMotorL, iLiftLevel[liftLevel], 100 );
	setMotorTarget( liftMotorR, iLiftLevel[liftLevel], 100 );
	wait1Msec(abs(getMotorEncoder(liftMotorL)-iLiftLevel[liftLevel])*MS_PER_ENCODER_UNIT);
	wait1Msec(300);
	EndTimeSlice();
}

void right_low_green_two_blue()
{
	setMotorTarget(clawMotor,CLAW_OPEN,100);
	goStraightDecel(5, -60, 0.007, 0.6, 30);

	pickUpTrigger = 4; liftLevel = 3; clawTarget = CLAW_CLOSE;
	startTask( pick_up_cube );

	goStraightDecel(10, -30, 0.007, 0.6, 0);
	setMotorTarget(liftMotorL, iLiftLevel[liftLevel],100);
	setMotorTarget(liftMotorR, iLiftLevel[liftLevel],100);

	goStraightDecel(55, -40, 0.007, 0.6, 0);

	clawAction(CLAW_OPEN);

	targetHeading = 5;
	goStraightDecel(5, 34, 0.007, 0.6, 0);

	setMotorTarget(clawMotor, CLAW_CLOSE,30);
	setMotorTarget(liftMotorL, iLiftLevel[1],100);
	setMotorTarget(liftMotorR, iLiftLevel[1],100);

	turnDecel(110, ON_SPOT_TURN, 0, 0.5, 10, 10);
	wait1Msec(300);

	goStraightDecel(29, 80, 0.007, 0.6, 0);
	setMotorTarget(scoopMotor, iScoopPos[0], 100);
	goStraightDecel(13, -40, 0.007, 0.6, 0);

	turnDecel(192, ON_SPOT_TURN, 0, 0.5, 10, 10);

	goStraightDecel(65, -100, 0.007, 0.6, 0);

	clawAction(CLAW_OPEN);

	wait1Msec(2000);
	setMotorTarget(scoopMotor, iScoopPos[1], 100);
	setMotorTarget(clawMotor, CLAW_CLOSE-100, 100);
}

void left_low_green_high_green()
{
	setMotorTarget(scoopMotor, iScoopPos[1], 100);
	setMotorTarget(clawMotor, CLAW_OPEN, 100);
	goStraightDecel(5, -60, 0.007, 0.6, 30);

	pickUpTrigger = 4; liftLevel = 3; clawTarget = CLAW_CLOSE;
	startTask( pick_up_cube );

	goStraightDecel(10, -30, 0.007, 0.6, 0);
	setMotorTarget(liftMotorL, iLiftLevel[liftLevel],100);
	setMotorTarget(liftMotorR, iLiftLevel[liftLevel],100);

	goStraightDecel(55, -40, 0.007, 0.6, 0);

	clawAction(CLAW_OPEN);

	goStraightDecel(10, 30, 0.007, 0.6, 0);

	setMotorTarget(clawMotor, CLAW_CLOSE,30);
	setMotorTarget(liftMotorL, iLiftLevel[0],100);
	setMotorTarget(liftMotorR, iLiftLevel[0],100);
	turnDecel(-90, ON_SPOT_TURN, 0, 0.5, 10, 20);

	goStraightDecel(32, -60, 0.007, 0.6, 30);
	turnDecel(-140, ON_SPOT_TURN, 0, 0.5, 10, 10);

	goStraightDecel(15, -80, 0.007, 0.6, 30);
	clawAction(CLAW_OPEN);
	setMotorTarget(clawMotor, CLAW_CLOSE,100);

	wait1Msec(300);

	pickUpTrigger = 15; liftLevel = 4; clawTarget = CLAW_OPEN;
	startTask( pick_up_cube );
	goStraightDecel(20, -50, 0.007, 0.6, 0);
	goStraightDecel(43, -80, 0.007, 0.6, 40);
	turnDecel(-235, ON_SPOT_TURN, 0, 0.25, 10, 10);

	goStraightDecel(21, -40, 0.007, 0.6, 30);

	clawAction(CLAW_CLOSE);

	setMotorTarget(liftMotorL,iLiftLevel[1],50);
	setMotorTarget(liftMotorR,iLiftLevel[1],50);
	goStraightDecel(15, 100, 0.007, 0.6, 30);
}

void two_red_two_blue_on_left()
{
	setMotorTarget(scoopMotor, iScoopPos[1], 100);
	setMotorTarget(clawMotor, CLAW_CLOSE, 100);
	setMotorTarget(liftMotorL,iLiftLevel[1],100);
	setMotorTarget(liftMotorR,iLiftLevel[1],100);

	targetHeading = 10;
	goStraightDecel(4, -100, 0.007, 0.6, 0);
	turnDecel(-70, RIGHT_WHEEL_TURN, 0, 0.5, 20, 10);

	goStraightDecel(18, 100, 0.007, 0.6, 0);
	setMotorTarget(scoopMotor, iScoopPos[0]+185, 100);
	goStraightDecel(10, -100, 0.007, 0.6, 0);

	//waitUntil(getTouchLEDValue(LED)==1);

	turnDecel(-135, ON_SPOT_TURN, 0, 0.5, 10, 10);
	goStraightDecel(30, 90, 0.007, 0.6, 30);

	//waitUntil(getTouchLEDValue(LED)==1);
/*
	turnDecel(-20, ON_SPOT_TURN, 0, 0.5, 20, 10);

	//waitUntil(getTouchLEDValue(LED)==1);
	goStraightDecel(5, 100, 0.007, 0.6, 0);
*/
	turnDecel(0, ON_SPOT_TURN, 0, 0.5, 20, 10);
	goStraightDecel(35, -100, 0.007, 0.6, 0);

	clawAction(CLAW_OPEN + 150);
}

void two_red_two_blue_on_left_2()
{
	setMotorTarget(scoopMotor, iScoopPos[1], 100);
	setMotorTarget(clawMotor, CLAW_CLOSE, 50);
	setMotorTarget(liftMotorL,iLiftLevel[1],100);
	setMotorTarget(liftMotorR,iLiftLevel[1],100);
	turnDecel(-80, RIGHT_WHEEL_TURN, 0, 0.5, 10, 10);

	goStraightDecel(20, 100, 0.007, 0.6, 0);
	setMotorTarget(scoopMotor, iScoopPos[0]+170, 100);
	goStraightDecel(2, -100, 0.007, 0.6, 0);

	//waitUntil(getTouchLEDValue(LED)==1);

	turnDecel(40, ON_SPOT_TURN, 0, 0.5, 10, 10);
	goStraightDecel(38, -90, 0.007, 0.6, 30);

	turnDecel(0, ON_SPOT_TURN, 0, 0.5, 10, 10);

	//waitUntil(getTouchLEDValue(LED)==1);

	goStraightDecel(28, -90, 0.007, 0.6, 30);
	clawAction(CLAW_OPEN + 150);

	//waitUntil(getTouchLEDValue(LED)==1);

	goStraightDecel(3, 100, 0.007, 0.6, 0);
}

void two_red_right()
{
	setMotorTarget(scoopMotor, iScoopPos[3], 100);
	setMotorTarget(clawMotor, CLAW_OPEN, 100);
	setMotorTarget(liftMotorL,iLiftLevel[0],100);
	setMotorTarget(liftMotorR,iLiftLevel[0],100);
	goStraightDecel(20, -90, 0.007, 0.6, 0);

	turnDecel(85, ON_SPOT_TURN, 0, 0.5, 20, 10);

	//waitUntil(getTouchLEDValue(LED)==1);

	goStraightDecel(30, 90, 0.007, 0.6, 0);
	turnNumber++;
	setMotorTarget(scoopMotor, iScoopPos[1]+ENCODER_UNIT_PER_SCOOP_ROUND*turnNumber, 100);
	goStraightDecel(20, -60, 0.007, 0.6, 30);

	//waitUntil(getTouchLEDValue(LED)==1);

	turnDecel(45, ON_SPOT_TURN, 0, 0.5, 20, 10);

	//waitUntil(getTouchLEDValue(LED)==1);

	goStraightDecel(20, 90, 0.007, 0.6, 0);

	scoreBalls();

	goStraightDecel(15, -90, 0.007, 0.6, 0);

	turnDecel(-55, ON_SPOT_TURN, 0, 0.5, 20, 10);

	goStraightDecel(35, -100, 0.007, 0.6, 0);
	turnDecel(-10, ON_SPOT_TURN, 0, 0.5, 20, 10);

	setMotorSpeed(leftMotor, -100);
	setMotorSpeed(rightMotor, -100);
	wait1Msec(800);

	setMotorSpeed(leftMotor, 100);
	setMotorSpeed(rightMotor, 100);
	wait1Msec(300);

	setMotorSpeed(leftMotor, 0);
	setMotorSpeed(rightMotor, 0);

}

void findPointZero(){
	setMotorSpeed(liftMotorL, -100);
	setMotorSpeed(liftMotorR, -100);
	setMotorSpeed(clawMotor, -50);
	setMotorSpeed(scoopMotor, -60);
	wait1Msec(2000);
	setMotorSpeed(liftMotorL, 0);	resetMotorEncoder(liftMotorL);
	setMotorSpeed(liftMotorR, 0);	resetMotorEncoder(liftMotorR);
	setMotorSpeed(clawMotor, 0);	resetMotorEncoder(clawMotor);
	setMotorSpeed(scoopMotor, 0); resetMotorEncoder(scoopMotor);
}

void prepareMotors(){
	setMotorTarget(scoopMotor, iScoopPos[1], 100);
	setMotorTarget(clawMotor, CLAW_OPEN, 100);
	setMotorTarget(liftMotorL, iLiftLevel[1], 100);
	setMotorTarget(liftMotorR, iLiftLevel[1], 100);
}

void prepareDriveTrain(){
	resetMotorEncoder(leftMotor);
	resetMotorEncoder(rightMotor);
	setMotorBrakeMode(leftMotor, motorHold);
	setMotorBrakeMode(rightMotor, motorHold);
}

task main()
{

	setMotorEncoderUnits( encoderCounts );

	// Define 0 point for claw, scooper, and lift.
	findPointZero();
	// Get claw, scooper, and lift ready for the game
	prepareMotors();
	wait1Msec(1000);

	iCheckBattLevel(7.8, 8.0);

	setTouchLEDColor(LED,colorOrange);
	waitUntil(getTouchLEDValue(LED) == 1);

	// Collect gyro drifting rate, and reset gyro
	setGyroStable();
	targetHeading = 0;

	// Get drive train ready for the game
	prepareDriveTrain();

	// On the mark
	LEDBusiness( colorGreen, 0, 0, 0, 0 );
	clearTimer( T1 );
	right_low_green_two_blue();

	LEDBusiness( colorGreen, 0, 0, 0, 0 );
	left_low_green_high_green();
	LEDBusiness( colorGreen, 0, 0, 0, 0 );
	two_red_two_blue_on_left();

	LEDBusiness( colorGreen, 0, 0, 0, 0 );
	two_red_right();

}
