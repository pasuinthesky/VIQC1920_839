#pragma config(Sensor, port3,  colorLeft,      sensorVexIQ_ColorGrayscale)
#pragma config(Sensor, port10, LED,            sensorVexIQ_LED)
#pragma config(Sensor, port12, gyro,           sensorVexIQ_Gyro)
#pragma config(Motor,  motor1,          BR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor4,          clawMotor,     tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor5,          BL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor6,          FL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor7,          FR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         armMotor,      tmotorVexIQ, PIDControl, reversed, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

float dt = 25;
float allowed_time = 7.5;
float iChC_filtered = 0.0;
float iChB_filtered = 0.0;
float iChA_filtered = 0.0;
float desired_heading = 0.0;

bool drive_override = false;

#define GYRO_SAMPLING_SECONDS 10000
#define ACCEPTABLE_DRIFT_RANGE 0.08
float fGyroDriftRate;

#define COUNT_PER_ROUND 960
#define WHEEL_TRAVEL 20.0
#define GEAR_RATIO 2
#define RATE 1

#define LIFT_LEVELS 3
int iArmLevel[LIFT_LEVELS] = {125, 1300, 1425};
#define ARM_LAND 1100
#define ARM_CARRY 200

bool claw_grab = false;
#define CLAW_CLOSE 100
#define CLAW_OPEN 735
#define CLAW_PUSH 540

float cmToEncoderUnit(float distance)
{
	return distance * COUNT_PER_ROUND / WHEEL_TRAVEL / GEAR_RATIO / RATE;
}

float tempDelta;

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

structPID pidStrafe;
structPID pidOrientation;
structPID pidEncoder;


void setGyroStable()
{
	fGyroDriftRate = 100;

	while (fGyroDriftRate > ACCEPTABLE_DRIFT_RANGE)
	{
		// Let the LDH flashing to remind people do not move the robot.
		setTouchLEDColor(LED,colorRed);
		setTouchLEDBlinkTime(LED, 14,6);
		wait1Msec(3000);
		setTouchLEDBlinkTime(LED, 0,1);
		wait1Msec(2000);

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

void strafePID(float direction, int distance, int maxJoyStick, float Kp, float Ki, float Kd, int delta)
{
	int tmpJoyStick, motor_a, motor_b, ChA_selector, ChB_selector;

	pidStrafe.setpoint = cmToEncoderUnit(distance) / sqrt(2);
	pidStrafe.measured_value = 0;
	pidStrafe.kp = Kp;
	pidStrafe.ki = Ki;
	pidStrafe.kd = Kd;
	pidStrafe.delta = cmToEncoderUnit(delta);

	pidEncoder.setpoint = 0;
	pidEncoder.measured_value = 0;
	pidEncoder.kp = 0.015;
	pidEncoder.ki = 0.000002;
	pidEncoder.kd = 0.005;
	pidEncoder.delta = 30;

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
	tempDelta = pidOrientation.delta;
	pidOrientation.delta = delta;
	desired_heading = heading;
	waitUntil( abs(getGyroStable() - desired_heading) <= pidOrientation.delta );
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
			setMotorSpeed( FL, 0 + iChA_filtered + iChB_filtered - iChC_filtered );
			setMotorSpeed( BL, 0 + iChA_filtered - iChB_filtered - iChC_filtered );
			setMotorSpeed( FR, 0 - iChA_filtered + iChB_filtered - iChC_filtered );
			setMotorSpeed( BR, 0 - iChA_filtered - iChB_filtered - iChC_filtered );
			wait1Msec(dt);
		}
	}
}

task claw_move()
{
	int claw_position, prev_claw = -1;

	setMotorBrakeMode(clawMotor, motorHold);
	while (true)
	{
		if (claw_grab)
		{
			claw_position = getMotorEncoder(clawMotor);
			if (prev_claw == claw_position)
			{
				setMotorSpeed(clawMotor, 0);
				prev_claw = 0;
				claw_grab = false;
			}
			else
			{
				prev_claw = claw_position;
				setMotorSpeed(clawMotor, -100);
				wait1Msec(100);
			}
		}
		wait1Msec(dt);
	}
}

void initialize()
{
	setMotorEncoderUnits(encoderCounts);

	setMotorBrakeMode(FL, motorHold);
	setMotorBrakeMode(FR, motorHold);
	setMotorBrakeMode(BR, motorHold);
	setMotorBrakeMode(BL, motorHold);
	setMotorBrakeMode(clawMotor, motorHold);
	setMotorBrakeMode( armMotor, motorHold );

	setMotorSpeed(clawMotor, -100);
	setMotorSpeed(armMotor, -50);
	wait1Msec(3000);

	resetMotorEncoder(clawMotor);
	resetMotorEncoder(armMotor);

	setMotorSpeed(clawMotor, 0);
	setMotorSpeed(armMotor, 0);

	setMotorTarget(clawMotor, CLAW_OPEN, 100);
	setMotorTarget(armMotor, iArmLevel[0], 100);
	wait1Msec(100);
	waitUntilMotorStop(clawMotor);
	waitUntilMotorStop(armMotor);
	setMotorSpeed(clawMotor, 0);
}

void land_riser()
{
	setMotorTarget(armMotor,ARM_LAND,100);
	waitUntilMotorStop(armMotor);
	setMotorTarget(clawMotor, CLAW_OPEN, 100);
	waitUntilMotorStop(clawMotor);
}

void right_side_3_1_3()
{
	strafePID(1, 55, 90, 0.18, 0, 0, 1);
	turnTo(-90, 10);
	strafePID(1, -32, 90, 0.18, 0, 0, 1);
	strafePID(3, 8, 90, 0.18, 0, 0, 1);
	claw_grab = true;
	waitUntil( !claw_grab );

	setMotorTarget(armMotor, iArmLevel[1], 100);
	wait1Msec(300);
	waitUntilMotorStop(armMotor);

	turnTo(-125, 10);

	strafePID(3, 5, 40, 0.18, 0, 0, 1);

	land_riser();

	strafePID(3, -65, 90, 0.18, 0, 0, 1);

	setMotorTarget(armMotor, iArmLevel[0], 100);
	//strafePID(3, -17, 40, 0.18, 0, 0, 1); this comment pairs with the fancy drift thing

	desired_heading = -90;
	strafePID(1, -40, 40, 0.18, 0, 0, 1);

	//strafePID(3, 5, 60, 0.18, 0, 0, 1);
	setMotorTarget(clawMotor, CLAW_PUSH, 100);

	while (getColorGrayscale(colorLeft)>100)
	{
		iChB_filtered = -20;
	}
	iChB_filtered = 0;
	wait1Msec(100);

	strafePID(3, 38, 90, 0.18, 0, 0, 1);

	strafePID(3, -30, 60, 0.18, 0, 0, 1);

	strafePID(1, -90, 60, 0.18, 0, 0, 1);
	//waitUntil(getTouchLEDValue(LED));
	//strafePID(1, 5, 40, 0.18, 0, 0, 1);

	strafePID(3, 30, 90, 0.18, 0, 0, 1);
	setMotorTarget(clawMotor, CLAW_OPEN, 100);
	//strafePID(3, -3, 40, 0.18, 0, 0, 1);

	strafePID(1, 20, 40, 0.18, 0, 0, 1);
	//waitUntil(getTouchLEDValue(LED));

	strafePID(3, 8, 40, 0.18, 0, 0, 1);
	claw_grab = true;
	waitUntil( !claw_grab );

	setMotorTarget(armMotor, iArmLevel[1], 100);
	wait1Msec(300);
	waitUntilMotorStop(armMotor);

	turnTo(-55, 10);

	strafePID(3, 4, 40, 0.18, 0, 0, 1);

	land_riser();

	strafePID(3, -60, 90, 0.18, 0, 0, 1);
	turnTo(0, 10);
	//waitUntil(getTouchLEDValue(LED));
	strafePID(3, -10, 40, 0.18, 0, 0, 1);

	strafePID(3, -120, 90, 0.18, 0, 0, 1);
}

void right_side_3_3_3_NOT_DONE()
{
	strafePID(1, 55, 90, 0.18, 0, 0, 1);
	turnTo(-90, 10);
	strafePID(1, -32, 90, 0.18, 0, 0, 1);
	strafePID(3, 8, 90, 0.18, 0, 0, 1);
	claw_grab = true;
	waitUntil( !claw_grab );

	setMotorTarget(armMotor, iArmLevel[1], 100);
	wait1Msec(300);
	waitUntilMotorStop(armMotor);

	turnTo(-125, 10);

	strafePID(3, 5, 40, 0.18, 0, 0, 1);

	land_riser();

	strafePID(3, -62, 90, 0.18, 0, 0, 1);

	setMotorTarget(armMotor, iArmLevel[0], 100);
	//strafePID(3, -17, 40, 0.18, 0, 0, 1); this comment pairs with the fancy drift thing

	desired_heading = -90;
	strafePID(1, -20, 40, 0.18, 0, 0, 1);

	//strafePID(3, 5, 60, 0.18, 0, 0, 1);
	setMotorTarget(clawMotor, CLAW_PUSH, 100);

	while (getColorGrayscale(colorLeft)>100)
	{
		iChB_filtered = -20;
	}
	iChB_filtered = 0;
	wait1Msec(100);

	strafePID(3, 38, 90, 0.18, 0, 0, 1);

	//////
	//////
	////// begin of middle 3 code
	//////
	//////
	setMotorTarget(clawMotor, CLAW_OPEN, 100);
	strafePID(1, -20, 60, 0.18, 0, 0, 1);
	strafePID(3, 8, 40, 0.18, 0, 0, 1);
	claw_grab = true;
	waitUntil( !claw_grab );

	setMotorTarget(armMotor, ARM_CARRY, 100);
	strafePID(3, 38, 40, 0.18, 0, 0, 1);
	strafePID(1, 40, 40, 0.18, 0, 0, 1);
	while (getColorGrayscale(colorLeft)>100)
	{
		iChB_filtered = 20;
	}
	iChB_filtered = 0;

	waitUntil(getTouchLEDValue(LED));

	setMotorTarget(armMotor, iArmLevel[1], 100);
	strafePID(1, 10, 40, 0.18, 0, 0, 1);
	strafePID(3, 38, 40, 0.18, 0, 0, 1);
	waitUntil(getTouchLEDValue(LED));

	land_riser();
	strafePID(3, -10, 60, 0.18, 0, 0, 1);
	setMotorTarget(armMotor, iArmLevel[0], 100);
	strafePID(3, 12, 40, 0.18, 0, 0, 1);
	claw_grab = true;
	waitUntil( !claw_grab );

	setMotorTarget(armMotor, iArmLevel[1], 100);
	strafePID(3, -10, 40, 0.18, 0, 0, 1);
	strafePID(1, -40, 40, 0.18, 0, 0, 1);
	land_riser();
	strafePID(3, -10, 60, 0.18, 0, 0, 1);
	setMotorTarget(armMotor, iArmLevel[0], 100);

	strafePID(3, -20, 60, 0.18, 0, 0, 1);
	waitUntil(getTouchLEDValue(LED));
	//////
	//////
	////// end of middle 3 code
	//////
	//////

	strafePID(1, -90, 60, 0.18, 0, 0, 1);
	//waitUntil(getTouchLEDValue(LED));
	//strafePID(1, 5, 40, 0.18, 0, 0, 1);

	strafePID(3, 30, 90, 0.18, 0, 0, 1);
	setMotorTarget(clawMotor, CLAW_OPEN, 100);
	//strafePID(3, -3, 40, 0.18, 0, 0, 1);

	strafePID(1, 20, 40, 0.18, 0, 0, 1);
	//waitUntil(getTouchLEDValue(LED));

	strafePID(3, 8, 40, 0.18, 0, 0, 1);
	claw_grab = true;
	waitUntil( !claw_grab );

	setMotorTarget(armMotor, iArmLevel[1], 100);
	wait1Msec(300);
	waitUntilMotorStop(armMotor);

	turnTo(-55, 10);

	strafePID(3, 4, 40, 0.18, 0, 0, 1);

	land_riser();

	strafePID(3, -60, 90, 0.18, 0, 0, 1);
	turnTo(0, 10);
	//waitUntil(getTouchLEDValue(LED));
	strafePID(3, -10, 40, 0.18, 0, 0, 1);

	strafePID(3, -120, 90, 0.18, 0, 0, 1);
}

void right_side_3_3_3_FANCY()
{
	strafePID(1, 55, 90, 0.18, 0, 0, 1);
	turnTo(-90, 10);
	strafePID(1, -32, 90, 0.18, 0, 0, 1);
	strafePID(3, 8, 90, 0.18, 0, 0, 1);
	claw_grab = true;
	waitUntil( !claw_grab );

	setMotorTarget(armMotor, iArmLevel[1], 100);
	wait1Msec(300);
	waitUntilMotorStop(armMotor);

	turnTo(-125, 10);

	strafePID(3, 5, 40, 0.18, 0, 0, 1);

	land_riser();

	strafePID(3, -10, 40, 0.18, 0, 0, 1);
	setMotorTarget(armMotor, iArmLevel[0], 100);
	strafePID(3, -10, 40, 0.18, 0, 0, 1);
	waitUntilMotorStop(armMotor);

	desired_heading = -90;
	//strafePID(1, -21, 40, 0.18, 0, 0, 1); // GREEN batt
	strafePID(1, -22.5, 40, 0.18, 0, 0, 1); // RED batt

	strafePID(3, 12, 40, 0.18, 0, 0, 1);
	claw_grab = true;
	waitUntil( !claw_grab );

	setMotorTarget(armMotor, ARM_CARRY, 100);

	while (getColorGrayscale(colorLeft)>100)
	{
		iChB_filtered = -20;
	}
	iChB_filtered = 0;
	wait1Msec(100);

	strafePID(3, -5, 40, 0.18, 0, 0, 1); // GREEN batt
	//strafePID(3, -3, 40, 0.18, 0, 0, 1); // RED batt
	wait1Msec(100);
	turnTo(-360, 3);

	wait1Msec(100);

	desired_heading = -360;
	setMotorTarget(armMotor, iArmLevel[1], 100);
	strafePID(3, 53, 40, 0.18, 0, 0, 1);
	turnTo(-415, 10);

	//waitUntil(getTouchLEDValue(LED));

	strafePID(3, 5, 40, 0.18, 0, 0, 1);
	land_riser();

	strafePID(3, -10, 40, 0.18, 0, 0, 1);
	setMotorTarget(armMotor, iArmLevel[0], 100);
	waitUntilMotorStop(armMotor);

	strafePID(3, 14, 40, 0.18, 0, 0, 1);
	//waitUntil(getTouchLEDValue(LED));

	claw_grab = true;
	waitUntil( !claw_grab );

	strafePID(3, -14, 40, 0.18, 0, 0, 1);
	setMotorTarget(armMotor, iArmLevel[1], 100);
	waitUntilMotorStop(armMotor);

	turnTo(-470, 10);

	land_riser();
	strafePID(3, -30, 40, 0.18, 0, 0, 1);
	setMotorTarget(armMotor, iArmLevel[0], 100);

	turnTo(-450, 10);

	setMotorTarget(clawMotor, CLAW_PUSH, 100);
	strafePID(1, -70, 60, 0.18, 0, 0, 1);
	//waitUntil(getTouchLEDValue(LED));
	//strafePID(1, 5, 40, 0.18, 0, 0, 1);

	strafePID(3, 30, 90, 0.18, 0, 0, 1);
	setMotorTarget(clawMotor, CLAW_OPEN, 100);
	//strafePID(3, -3, 40, 0.18, 0, 0, 1);

	strafePID(1, 19, 40, 0.18, 0, 0, 1);
	//waitUntil(getTouchLEDValue(LED));

	strafePID(3, 10, 40, 0.18, 0, 0, 1);
	claw_grab = true;
	waitUntil( !claw_grab );

	setMotorTarget(armMotor, iArmLevel[1], 100);
	wait1Msec(300);
	waitUntilMotorStop(armMotor);

	turnTo(-415, 10);

	strafePID(3, 4, 40, 0.18, 0, 0, 1);

	land_riser();

	strafePID(3, -60, 90, 0.18, 0, 0, 1);
	turnTo(-360, 10);
	//waitUntil(getTouchLEDValue(LED));
	strafePID(3, -10, 40, 0.18, 0, 0, 1);

	strafePID(3, -120, 90, 0.18, 0, 0, 1);
}

task main()
{
	initialize();
	setTouchLEDColor(LED,colorYellow);
	waitUntil(getTouchLEDValue(LED));

	resetGyro(gyro);
	setGyroStable();
	setTouchLEDColor(LED,colorBlue);
	waitUntil(getTouchLEDValue(LED));

	setTouchLEDColor(LED,colorGreen);
	clearDebugStream();

	resetGyroStable();
	desired_heading = 0;
	resetTimer(T2);

	pidOrientation.delta = 0.6;
	pidOrientation.kp = 1;
	pidOrientation.ki = 0.00001;
	pidOrientation.kd = 0.0001;

	startTask(drive);
	startTask(claw_move);

	//right_side_3_1_3();

	right_side_3_3_3_FANCY();

	displayTextLine(3, "%f", getTimerValue(T2));
	waitUntil(getTouchLEDValue(LED));
}
