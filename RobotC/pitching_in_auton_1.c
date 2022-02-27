#pragma config(Sensor, port5,  LED,            sensorVexIQ_LED)
#pragma config(Sensor, port8,  gyro,           sensorVexIQ_Gyro)
#pragma config(Sensor, port9,  sonar,          sensorVexIQ_Distance)
#pragma config(Motor,  motor1,          leftMotor,     tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor3,          intakeMotor,   tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor4,          catapultMotor, tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor6,          rightMotor,    tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor11,         gateMotor,     tmotorVexIQ, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define GYRO_SAMPLING_SECONDS 5
#define ACCEPTABLE_DRIFT_RANGE 0.05
float fGyroDriftRate;
#define MIN_TURNING_CHC 10
#define MIN_DRIVING_CHC 0

#define COUNT_PER_ROUND 960
#define WHEEL_TRAVEL 20
#define GEAR_RATIO 2
#define RATE 1

#define SONAR_IN_RANGE 90

float dt = 25;
float allowed_time = 7.5;
float iChC_filtered = 0.0;
float iChA_filtered = 0.0;
float desired_heading = 0.0;
float min_ChC = 0.0;


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

structPID pidDrive;
structPID pidOrientation;




void setGyroStable()
{
	setMotorSpeed(gateMotor, 0);

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

	writeDebugStreamLine( "%f %f %f %f %f %f %f %f", getTimerValue(T2), output, error, pid.integral, derivative, pid.kp, pid.ki, pid.kd );

	return output;
}

void goStraight(int distance, int maxJoyStick, float Kp, float Ki, float Kd, int delta)
{

	pidOrientation.setpoint = desired_heading;
	pidOrientation.delta = 10;
	pidOrientation.kp = 0.55;
	pidOrientation.ki = 0.001;
	pidOrientation.kd = 0.1;
	pidOrientation.integral = 0;
	pidOrientation.prev_error = 0;
	min_ChC = MIN_DRIVING_CHC;

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


	while ( abs(pidDrive.setpoint - pidDrive.measured_value) > pidDrive.delta )
	{
		pidDrive.measured_value = sgn(pidDrive.setpoint) * (abs(getMotorEncoder(leftMotor)) + abs(getMotorEncoder(rightMotor))) / 2;
		iChA_filtered = PIDControl(pidDrive);
		if (abs(iChA_filtered) > maxJoyStick)
			iChA_filtered = maxJoyStick * sgn(iChA_filtered);

		pidOrientation.measured_value = getGyroStable();
		iChC_filtered = -PIDControl(pidOrientation);
		if ( abs(iChC_filtered) < min_ChC && iChC_filtered != 0 )
		{
			iChC_filtered = sgn(iChC_filtered) * min_ChC;
		}
		//writeDebugStreamLine( "%f %f %f %f %f %f %f %f", getTimerValue(T2), iChA_filtered, iChC_filtered, getGyroDegrees(gyro), getGyroStable(), desired_heading, pidDrive.setpoint, pidDrive.measured_value );


		setMotorSpeed(leftMotor,iChA_filtered+iChC_filtered);
		setMotorSpeed(rightMotor,iChA_filtered-iChC_filtered);

		wait1Msec(dt);
	}
	setMotorSpeed(leftMotor,0);
	setMotorSpeed(rightMotor,0);
}

void turnTo(float heading, float speed, float Kp, float Ki, float Kd, float delta)
{
	desired_heading = heading;
	iChA_filtered = speed;

	pidOrientation.setpoint = desired_heading;
	pidOrientation.delta = delta;
	pidOrientation.kp = Kp;
	pidOrientation.ki = Ki;
	pidOrientation.kd = Kd;
	pidOrientation.integral = 0;
	pidOrientation.prev_error = 0;

	min_ChC = MIN_TURNING_CHC;

	while ( abs(getGyroStable() - desired_heading) > pidOrientation.delta )
	{
		pidOrientation.measured_value = getGyroStable();
		iChC_filtered = -PIDControl(pidOrientation);
		if ( abs(iChC_filtered) < min_ChC && iChC_filtered != 0 )
		{
			iChC_filtered = sgn(iChC_filtered) * min_ChC;
		}

		writeDebugStreamLine( "%f %f %f %f %f %f %f %f", getTimerValue(T2), iChA_filtered, iChC_filtered, getGyroDegrees(gyro), getGyroStable(), desired_heading, pidDrive.setpoint, pidDrive.measured_value );

		setMotorSpeed(leftMotor,iChA_filtered+iChC_filtered);
		setMotorSpeed(rightMotor,iChA_filtered-iChC_filtered);

		wait1Msec(dt);
	}
	setMotorSpeed(leftMotor,0);
	setMotorSpeed(rightMotor,0);
}

void set_catapult()
{
	while (getDistanceValue(sonar) > SONAR_IN_RANGE)
	{
		setMotorSpeed(catapultMotor, -100);
		wait1Msec(dt);
	}
	setMotorSpeed(catapultMotor, 0);
}

void fire_catapult()
{
	if (getDistanceValue(sonar) < SONAR_IN_RANGE)
	{
		setMotorSpeed(gateMotor, 100);
		wait1Msec(300);
		setMotorSpeed(gateMotor, 0);

		setMotorSpeed(catapultMotor, -100);
		wait1Msec(500);
		setMotorTarget(gateMotor, -500, 100);
		wait1Msec(800);
	}
	//set_catapult();
}

void initialize()
{
	setMotorEncoderUnits(encoderCounts);

	setMotorBrakeMode(rightMotor, motorCoast);
	setMotorBrakeMode(leftMotor, motorCoast);

	setMotorSpeed(gateMotor, 100);
	wait1Msec(800);
	resetMotorEncoder(gateMotor);
	setMotorSpeed(gateMotor, 0);

	setMotorTarget(gateMotor, -500, 100);
	set_catapult();

	setMotorBrakeMode(rightMotor, motorHold);
	setMotorBrakeMode(leftMotor, motorHold);

	setTouchLEDColor(LED,colorYellow);
	waitUntil(getTouchLEDValue(LED));

	setMotorBrakeMode(gateMotor, motorHold);
}

void outtake()
{
	setMotorSpeed(intakeMotor, 0);
	wait1Msec(100);
	setMotorSpeed(intakeMotor, -100);
	wait1Msec(500);
	setMotorSpeed(intakeMotor, 0);
}

task outtake_task()
{
	outtake();
}


void eight_ball()
{
	turnTo(22, 0, 0.55, 0, 0, 8);
	setMotorSpeed(intakeMotor, 100);
	goStraight(25, 90, 5, 0, 1, 13);
	turnTo(-50, 0, 0.55, 0, 0, 10);
	goStraight(30, 90, 5, 0, 1, 13);
	turnTo(-90, 0, 0.55, 0, 0, 8);
	setMotorSpeed(intakeMotor, 0);
	//waitUntil(getTouchLEDValue(LED));

	desired_heading = -100;
	goStraight(90, 90, 5, 2, 1, 15);
	setMotorSpeed(intakeMotor, 100);
	goStraight(20, 60, 5, 2, 1, 5);
	turnTo(-40, 0, 0.55, 0, 0, 3);

	desired_heading = -10;
	goStraight(40, 60, 5, 2, 1, 15);
	turnTo(-5, 20, 0.55, 0, 0, 3);

	goStraight(70, 80, 5, 2, 1, 2);
	setMotorSpeed(intakeMotor, 0);
	//waitUntil(getTouchLEDValue(LED));

	goStraight(8, 60, 5, 2, 1, 2);
	setMotorSpeed(intakeMotor, 100);
	wait1Msec(400);
	setMotorSpeed(intakeMotor, 0);

	turnTo(120, 0, 0.55, 0, 0, 3);
	goStraight(80, 60, 5, 2, 1, 23);
	//waitUntil(getTouchLEDValue(LED));

	setMotorSpeed(intakeMotor, 100);
	goStraight(10, 40, 5, 2, 1, 3);
	wait1Msec(200);
	setMotorSpeed(intakeMotor, 0);
	//turnTo(54, 35, 0.55, 0, 0, 3);
	//goStraight(10, 90, 5, 2, 1, 3);
	//waitUntil(getTouchLEDValue(LED));


	turnTo(240, 0, 0.55, 0, 0, 3);
	//waitUntil(getTouchLEDValue(LED));

	setMotorSpeed(leftMotor, -100);
	setMotorSpeed(rightMotor, -100);
	wait1Msec(500);
	setMotorSpeed(leftMotor, 0);
	setMotorSpeed(rightMotor, 0);
	outtake();
	fire_catapult();
	setMotorSpeed(intakeMotor, 100);
	set_catapult();
	wait1Msec(500);
	outtake();
	setMotorSpeed(intakeMotor, 100);
	fire_catapult();
	set_catapult();
	wait1Msec(500);
	outtake();
	setMotorSpeed(intakeMotor, 100);
	fire_catapult();
	set_catapult();
	//wait1Msec(500);
	//outtake();
	//setMotorSpeed(intakeMotor, 100);
	fire_catapult();
	set_catapult();
}

/*
void eight_ball_tmp()
{
	turnTo(22, 0, 0.55, 0, 0, 8);
	setMotorSpeed(intakeMotor, 100);
	goStraight(15, 90, 5, 0, 1, 3);
	turnTo(-50, 0, 0.55, 0, 0, 10);
	goStraight(20, 90, 5, 0, 1, 3);
	turnTo(-90, 0, 0.55, 0, 0, 5);
	setMotorSpeed(intakeMotor, 0);
	//waitUntil(getTouchLEDValue(LED));

	desired_heading = -100;
	goStraight(80, 90, 5, 2, 1, 5);
	setMotorSpeed(intakeMotor, 100);
	goStraight(25, 60, 5, 2, 1, 5);
	turnTo(-40, 0, 0.55, 0, 0, 3);
	desired_heading = -10;
	setMotorSpeed(intakeMotor, 100);
	goStraight(40, 60, 5, 2, 1, 15);
	turnTo(-10, 20, 0.55, 0, 0, 3);
	setMotorSpeed(leftMotor, 100);
	setMotorSpeed(rightMotor, 90);
	wait1Msec(1800);
	setMotorSpeed(leftMotor, 70);
	setMotorSpeed(rightMotor, 60);
	wait1Msec(600);

	resetGyroStable();

	//setMotorSpeed(leftMotor, 50);
	//setMotorSpeed(rightMotor, 50);
	//wait1Msec(300);
	//waitUntil(getTouchLEDValue(LED));
	turnTo(70, 0, 0.55, 0, 0, 3);

	goStraight(45, 60, 5, 2, 1, 2);
	setMotorSpeed(intakeMotor, 100);
	goStraight(-10, 30, 5, 0, 1, 2);
	setMotorSpeed(intakeMotor, 30);
	turnTo(180, 0, 0.55, 0, 0, 5);
	setMotorSpeed(intakeMotor, 0);

	displayTextLine(3, "%f", getGyroStable());
	//waitUntil(getTouchLEDValue(LED));

	goStraight(55, 60, 5, 2, 1, 2);
	displayTextLine(3, "%f", getGyroStable());
	waitUntil(getTouchLEDValue(LED));

	turnTo(250, 0, 0.55, 0, 0, 5);
	displayTextLine(3, "%f", getGyroStable());

	setMotorSpeed(leftMotor, -100);
	setMotorSpeed(rightMotor, -100);
	wait1Msec(500);
	setMotorSpeed(leftMotor, 0);
	setMotorSpeed(rightMotor, 0);
	outtake();
	fire_catapult();
	setMotorSpeed(intakeMotor, 100);
	set_catapult();
	wait1Msec(500);
	setMotorSpeed(intakeMotor, 0);
	outtake();
	fire_catapult();
	setMotorSpeed(intakeMotor, 100);
	set_catapult();
	wait1Msec(500);
	setMotorSpeed(intakeMotor, 0);
	fire_catapult();
	set_catapult();
	fire_catapult();
}
*/

/*
void six_ball()
{
	turnTo(22, 0, 0.55, 0, 0, 8);
	setMotorSpeed(intakeMotor, 100);
	goStraight(15, 90, 5, 0, 1, 3);
	turnTo(-50, 0, 0.55, 0, 0, 10);
	goStraight(20, 90, 5, 0, 1, 3);
	turnTo(-90, 0, 0.55, 0, 0, 5);
	setMotorSpeed(intakeMotor, 0);
	//waitUntil(getTouchLEDValue(LED));

	desired_heading = -100;
	goStraight(80, 90, 5, 2, 1, 5);
	setMotorSpeed(intakeMotor, 100);
	goStraight(25, 60, 5, 2, 1, 5);
	turnTo(-40, 0, 0.55, 0, 0, 3);
	desired_heading = -10;
	setMotorSpeed(intakeMotor, 100);
	goStraight(40, 60, 5, 2, 1, 15);
	turnTo(-10, 20, 0.55, 0, 0, 3);
	//goStraight(20, 30, 5, 2, 1, 15);
	//waitUntil(getTouchLEDValue(LED));

	setMotorSpeed(intakeMotor, 100);
	turnTo(57, 0, 0.55, 0, 0, 3);
	//waitUntil(getTouchLEDValue(LED));

	setMotorSpeed(intakeMotor, 100);
	goStraight(30, 90, 5, 2, 1, 3);
	//turnTo(54, 35, 0.55, 0, 0, 3);
	//goStraight(10, 90, 5, 2, 1, 3);
	//waitUntil(getTouchLEDValue(LED));

	turnTo(40, 0, 0.55, 0, 0, 3);
	//waitUntil(getTouchLEDValue(LED));

	setMotorSpeed(intakeMotor, 100);
	goStraight(13, 90, 5, 2, 1, 3);

	turnTo(-90, 0, 0.55, 0, 0, 3);
	//waitUntil(getTouchLEDValue(LED));

	setMotorSpeed(leftMotor, -100);
	setMotorSpeed(rightMotor, -100);
	wait1Msec(500);
	setMotorSpeed(leftMotor, 0);
	setMotorSpeed(rightMotor, 0);
	outtake();
	fire_catapult();
	setMotorSpeed(intakeMotor, 100);
	set_catapult();
	wait1Msec(500);
	setMotorSpeed(intakeMotor, 0);
	outtake();
	fire_catapult();
	setMotorSpeed(intakeMotor, 100);
	set_catapult();
	wait1Msec(500);
	setMotorSpeed(intakeMotor, 0);
	fire_catapult();
	set_catapult();
}
*/

/*
void four_ball()
{
	turnTo(22, 0, 0.55, 0, 0, 8);
	setMotorSpeed(intakeMotor, 100);
	goStraight(15, 90, 5, 0, 1, 3);
	turnTo(-50, 0, 0.55, 0, 0, 10);
	goStraight(20, 90, 5, 0, 1, 3);
	turnTo(-90, 0, 0.55, 0, 0, 5);
	setMotorSpeed(intakeMotor, 0);
	//waitUntil(getTouchLEDValue(LED));

	desired_heading = -100;
	goStraight(80, 90, 5, 2, 1, 5);
	setMotorSpeed(intakeMotor, 100);
	goStraight(25, 60, 5, 2, 1, 5);
	turnTo(-40, 0, 0.55, 0, 0, 3);
	desired_heading = -10;
	setMotorSpeed(intakeMotor, 100);
	goStraight(40, 60, 5, 2, 1, 15);
	turnTo(-10, 20, 0.55, 0, 0, 3);
	goStraight(20, 30, 5, 2, 1, 15);
	setMotorSpeed(intakeMotor, 100);
	waitUntil(getTouchLEDValue(LED));
	goStraight(-5, 60, 5, 2, 1, 2);
	waitUntil(getTouchLEDValue(LED));
	turnTo(20, 0, 0.55, 0, 0, 3);
	waitUntil(getTouchLEDValue(LED));
	goStraight(20, 60, 5, 2, 1, 3);
	//waitUntil(getTouchLEDValue(LED));
	turnTo(-90, 0, 0.55, 0, 0, 3);
	goStraight(-30, 60, 5, 2, 1, 2);
	setMotorSpeed(intakeMotor, 0);
	setMotorSpeed(leftMotor, -40);
	setMotorSpeed(rightMotor, -40);
	wait1Msec(200);
	setMotorSpeed(intakeMotor, -50);
	wait1Msec(300);
	setMotorSpeed(intakeMotor, 0);
	setMotorSpeed(leftMotor, 0);
	setMotorSpeed(rightMotor, 0);
	fire_catapult();
	set_catapult();
	setMotorSpeed(intakeMotor, 100);
	wait1Msec(1000);
	setMotorSpeed(intakeMotor, 0);
	fire_catapult();
	set_catapult();
}
*/

/*
void next_four()
{
	turnTo(-120, 0, 0.55, 0, 0, 3);
	//waitUntil(getTouchLEDValue(LED));
	setMotorSpeed(intakeMotor, 60);
	goStraight(50, 90, 5, 2, 1, 2);
	turnTo(-40, 0, 0.55, 0, 0, 3);
	setMotorSpeed(intakeMotor, 80);
	setMotorSpeed(leftMotor, 100);
	setMotorSpeed(rightMotor, 90);
	wait1Msec(1800);
	setMotorSpeed(leftMotor, 70);
	setMotorSpeed(rightMotor, 60);
	wait1Msec(600);

	resetGyroStable();

	//setMotorSpeed(leftMotor, 50);
	//setMotorSpeed(rightMotor, 50);
	//wait1Msec(300);
	//waitUntil(getTouchLEDValue(LED));
	turnTo(70, 0, 0.55, 0, 0, 3);

	goStraight(45, 60, 5, 2, 1, 2);
	setMotorSpeed(intakeMotor, 100);
	goStraight(-10, 30, 5, 0, 1, 2);
	setMotorSpeed(intakeMotor, 30);
	turnTo(180, 0, 0.55, 0, 0, 5);
	setMotorSpeed(intakeMotor, 0);

	displayTextLine(3, "%f", getGyroStable());
	//waitUntil(getTouchLEDValue(LED));

	goStraight(63, 60, 5, 2, 1, 2);
	displayTextLine(3, "%f", getGyroStable());
	//waitUntil(getTouchLEDValue(LED));

	turnTo(260, 0, 0.55, 0, 0, 5);
	displayTextLine(3, "%f", getGyroStable());
	//waitUntil(getTouchLEDValue(LED));

	setMotorSpeed(intakeMotor, 30);
	//goStraight(-20, 90, 5, 2, 1, 2);
	setMotorSpeed(leftMotor, -100);
	setMotorSpeed(rightMotor, -100);
	wait1Msec(200);
	setMotorSpeed(intakeMotor, -50);
	wait1Msec(300);
	setMotorSpeed(intakeMotor, 0);
	setMotorSpeed(leftMotor, 0);
	setMotorSpeed(rightMotor, 0);
	fire_catapult();
	set_catapult();
	setMotorSpeed(intakeMotor, 100);
	wait1Msec(1000);
	setMotorSpeed(intakeMotor, 0);
	fire_catapult();
	set_catapult();
}
*/

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

/*
	// Test Block - Drivechain
	goStraight(100, 60, 5, 2, 1, 3);
	waitUntil(getTouchLEDValue(LED));
	turnTo(-90, 40, 0.2, 0, 0, 5);
	waitUntil(getTouchLEDValue(LED));
*/

/*
	// Test Block - Catapult
	waitUntil(getTouchLEDValue(LED));
	fire_catapult();
	waitUntil(getTouchLEDValue(LED));
	fire_catapult();
	waitUntil(getTouchLEDValue(LED));
	fire_catapult();
*/

/*
	six_ball();
	displayTextLine(3, "%f", getTimerValue(T2));
	//waitUntil(getTouchLEDValue(LED));
	next_four();
*/
	eight_ball();
	displayTextLine(3, "%f", getTimerValue(T2));
	waitUntil(getTouchLEDValue(LED));
	/*
	set_catapult();

	setTouchLEDColor(LED,colorBlue);
	waitUntil(getTouchLEDValue(LED));
	resetGyroStable();

	four_ball();
	next_four();
	*/
}