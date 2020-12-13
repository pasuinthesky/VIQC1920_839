#pragma config(Sensor, port10, LED,            sensorVexIQ_LED)
#pragma config(Sensor, port12, gyro,           sensorVexIQ_Gyro)
#pragma config(Motor,  motor1,          BR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor4,          clawMotor,     tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor5,          BL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor6,          FL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor7,          FR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         liftMotor,     tmotorVexIQ, PIDControl, reversed, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

int error;
float integral = 0;
float dt = 25;
float prev_error = 0;
float allowed_time = 7.5;
float output;
float delta = 10;
int artificial_ChC_Reading = 0;
float desired_heading;
bool claw_working = false;
#define ARM_DELTA 10
#define CLAW_DELTA 15
#define MS_PER_ENCODER_UNIT 3

#define LIFT_LEVELS 2
int iLiftLevel[LIFT_LEVELS] = {30, 450};
int in_between_level = 405

#define CLAW_CLOSED 15
#define CLAW_OPEN 145

/*
int iDriveMapping[101] = {
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
10,10,10,10,10,10,10,10,10,10,
10,10,10,10,10,10,10,10,10,10,
20,20,20,20,20,20,20,20,20,20,
30,30,30,30,30,30,30,30,30,30,
40,40,40,40,40,40,40,40,40,40,
50,50,50,50,50,50,50,50,50,50,
60,62,64,66,68,70,72,74,76,78,
84,88,92,96,100,100,100,100,100,100,100};

int iStrafeMapping[101] = {
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
10,10,10,10,10,10,10,10,10,10,
10,10,10,10,10,10,10,10,10,10,
20,20,20,20,20,20,20,20,20,20,
25,25,25,25,25,25,25,25,25,25,
30,30,30,30,30,30,30,40,40,40,
50,50,50,50,50,60,60,60,60,60,
80,80,80,80,80,80,80,80,80,80,
100,100,100,100,100,100,100,100,100,100,100};
*/
int iDriveMapping[101] = {
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	10,10,10,10,10,10,10,10,10,10,
	10,10,10,10,10,10,10,10,10,10,
	20,20,20,20,20,20,20,20,20,20,
	20,20,20,20,20,20,20,20,20,20,
	30,30,30,30,30,30,30,30,30,30,
	30,30,30,30,30,30,30,30,30,30,
	30,30,30,30,30,30,30,30,30,30,
	40,40,40,60,60,80,80,100,100,100,100};

int iStrafeMapping[101] = {
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	10,10,10,10,10,10,10,10,10,10,
	10,10,10,10,10,10,10,10,10,10,
	20,20,20,20,20,20,20,20,20,20,
	20,20,20,20,20,20,20,20,20,20,
	30,30,30,30,30,30,30,30,30,30,
	30,30,30,30,30,30,30,30,30,30,
	30,30,30,30,30,30,30,30,30,30,
	40,40,40,60,60,80,80,100,100,100,100};

int iTurnMapping[101] = {
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	10,10,10,10,10,10,10,10,10,10,
	10,10,10,10,10,10,10,10,10,10,
	20,20,20,20,20,20,20,20,20,20,
	20,20,20,20,20,20,20,20,20,20,
	30,30,30,30,30,30,30,30,30,30,
	30,30,30,30,30,30,30,30,30,30,
	30,30,30,30,30,30,30,30,30,30,
	40,40,40,40,40,40,40,60,60,60,60};

task claw_preset()
{
	int claw_position, prev_claw=CLAW_CLOSED;

	while (true)
	{
		// checks the level before moving
		if (getJoystickValue(BtnLUp)==1)
		{
			if (! claw_working)
			{
				claw_working = true;
				if ( abs(getMotorEncoder(clawMotor) - CLAW_OPEN) < CLAW_DELTA )
				{
					setMotorBrakeMode(clawMotor, motorCoast);
					setMotorTarget(clawMotor,CLAW_CLOSED,100);
					wait1Msec( abs( CLAW_CLOSED - CLAW_OPEN ) * MS_PER_ENCODER_UNIT );
					setMotorSpeed(clawMotor, 0);
					if ( abs(getMotorEncoder(liftMotor) - iLiftLevel[1]) < ARM_DELTA  )
					{
						setMotorTarget(liftMotor, in_between_level, 100);
						setMotorSpeed(BL, -50 );
						setMotorSpeed(BR, 50 );
						setMotorSpeed(FL, -50 );
						setMotorSpeed(FR, 50 );
						wait1Msec(500);
						setMotorSpeed(BL, 0 );
						setMotorSpeed(BR, 0 );
						setMotorSpeed(FL, 0 );
						setMotorSpeed(FR, 0 );
					}
				}
				else
				{
					setMotorBrakeMode(clawMotor, motorHold);
					claw_position = getMotorEncoder(clawMotor);
					if (prev_claw == claw_position)
					{
						setMotorSpeed(clawMotor, 0);
					}
					else
					{
						setMotorSpeed(clawMotor,100);
					}
					prev_claw = claw_position;
				}
			}
		}
		else
		{
			claw_working = false;
			setMotorSpeed(clawMotor, 0);
		}
		wait1Msec(50);
	}
}

float PIDControl (float setpoint, float measured_value, float Kp, float Ki, float Kd, int delta)
{
	error = measured_value - setpoint;

	if ( abs(error) < delta )
		error = 0;

	integral = integral + error * dt;
	float derivative = ( error - prev_error ) / dt;
	output = Kp * error + Ki * integral + Kd * derivative;
	prev_error = error;

	return output;
}

void lift_preset()
{
	if (getJoystickValue(BtnRUp)== 1)
	{
		setMotorSpeed(BL, 0 );
		setMotorSpeed(BR, 0 );
		setMotorSpeed(FL, 0 );
		setMotorSpeed(FR, 0 );

		for (int i=0;i<LIFT_LEVELS-1;i=i+1)
		{
			if ( getMotorEncoder(liftMotor) < ( iLiftLevel[i+1] - ARM_DELTA ) )
			{
				setMotorTarget(liftMotor, iLiftLevel[i+1], 100);
				wait1Msec( ( iLiftLevel[i+1] - iLiftLevel[i] ) * MS_PER_ENCODER_UNIT );
				break;
			}
		}
	}
	else if (getJoystickValue(BtnRDown)==1)
	{
		for (int i=LIFT_LEVELS-1;i>0;i=i-1)
		{
			if ( getMotorEncoder(liftMotor) > ( iLiftLevel[i-1] + ARM_DELTA ) )
			{
				setMotorTarget(liftMotor, iLiftLevel[i-1],100);
				wait1Msec( ( iLiftLevel[i] - iLiftLevel[i-1] ) * MS_PER_ENCODER_UNIT );
				break;
			}
		}
	}
}
/*
task flashLED()
{
while( true )
{
setTouchLEDRGB(LED, 0, 255, 0);
sleep( 500 );
setTouchLEDRGB(LED, 0, 255, 255);
sleep( 500 );
}
}*/
task flashLED ()
{
	while (true)
	{
		setTouchLEDColor(LED, colorRed);
		wait1Msec(10);
		setTouchLEDColor(LED, colorBlue);
		wait1Msec(10);
		setTouchLEDColor(LED, colorYellow);
		wait1Msec(10);
		setTouchLEDColor(LED, colorOrange);
		wait1Msec(10);
		setTouchLEDColor(LED, colorGreen);
		wait1Msec(10);
		setTouchLEDColor(LED, colorViolet);
	}
}




task main()
{
	int iChC_filtered;
	int iChA_filtered;
	int iChB_filtered;

	setMotorBrakeMode(FL, motorHold);
	setMotorBrakeMode(FR, motorHold);
	setMotorBrakeMode(BR, motorHold);
	setMotorBrakeMode(BL, motorHold);
	setMotorBrakeMode(clawMotor, motorCoast);
	setMotorBrakeMode( liftMotor, motorHold );

	setMotorSpeed(clawMotor, -50);
	setMotorSpeed(liftMotor, -30);
	wait1Msec(3000);

	resetMotorEncoder(clawMotor);
	resetMotorEncoder(liftMotor);

	setMotorSpeed(clawMotor, 0);
	setMotorSpeed(liftMotor, 0);

	setMotorTarget(clawMotor, CLAW_CLOSED, 60);
	setMotorTarget(liftMotor, iLiftLevel[0], 100);
	wait1Msec(1000);
	setMotorSpeed(clawMotor, 0);

	resetGyro(gyro);
	desired_heading = 0;

	//startTask( flashLED );
	startTask(claw_preset);

	while(true)
	{

		iChA_filtered = iDriveMapping[abs(getJoystickValue(ChA))]*sgn(getJoystickValue(ChA));
		iChB_filtered = iStrafeMapping[abs(getJoystickValue(ChB))]*sgn(getJoystickValue(ChB));
		iChC_filtered = iTurnMapping[abs(getJoystickValue(ChC))]*sgn(getJoystickValue(ChC));

		if ( abs(getJoystickValue(ChC)) <= 5 )
		{
			iChC_filtered = PIDControl( desired_heading, getGyroDegrees(gyro), 0.7, 0, 0, 5 );
			setTouchLEDColor(LED, colorBlue);
		}
		else
		{
			desired_heading = getGyroDegrees(gyro);
			setTouchLEDColor(LED, colorYellow);
		}

		displayCenteredTextLine(3, "%d, %d, %d", getGyroDegrees(gyro), desired_heading, getMotorBrakeMode(clawMotor));

		lift_preset();

		setMotorSpeed(BL, 0 + iChA_filtered - iChB_filtered - iChC_filtered );
		setMotorSpeed(BR, 0 - iChA_filtered - iChB_filtered - iChC_filtered );
		setMotorSpeed(FL, 0 + iChA_filtered + iChB_filtered - iChC_filtered );
		setMotorSpeed(FR, 0 - iChA_filtered + iChB_filtered - iChC_filtered );

		wait1Msec(dt);
	}
}
