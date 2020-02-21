#pragma config(Motor,  motor8,           leftMotor,     tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor2,           rightMotor,    tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor12,          scoopMotor,    tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,          liftMotorL,    tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor5,           liftMotorR,    tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor6,           clawMotor,     tmotorVexIQ, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define CLAW_CLOSE 450
#define CLAW_OPEN 50
#define CLAW_DELTA 20
#define SCOOPER_DELTA 35
#define ARM_DELTA 25
#define MS_PER_ENCODER_UNIT 1
#define ENCODER_UNIT_PER_SCOOP_ROUND 1920
#define FLIP_UP 1250
#define FLIP_DOWN 900

bool overwriteDrive = false;
bool overwriteClaw = false;
bool overwriteLift = false;
bool overwriteScooper = false;


#define LIFT_LEVELS 5
int iLiftLevel[LIFT_LEVELS] = {15 ,400, 1100, 1435, 1830}; //pickup_inside, pickup_outside/transport,low_inside,low_outside,,high_inside

int iScoopPos[4] = { -150, 250, 815, 1000 }; //pick_cube, keep_ball/release cube, ready_to_scoop_ball, score_ball

int iChA_filtered=0, iChC_filtered=0, turnNumber = 0;

int iDriveDirection = -1;

int iDriveMapping[101] = {
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
10,10,10,10,10,20,20,20,20,20,
20,20,20,20,20,30,30,30,30,30,
30,30,30,30,30,30,30,30,30,30,
30,30,30,30,30,30,30,30,30,30,
40,40,40,40,40,40,40,40,40,40,
50,50,50,50,50,50,50,50,50,50,
60,62,64,66,68,70,72,74,76,78,
84,88,92,96,100,100,100,100,100,100,100};

int iTurnMapping[101] = {
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
10,10,10,10,10,20,20,20,20,20,
20,20,20,20,20,20,20,20,20,20,
20,20,20,20,20,20,20,20,20,20,
30,30,30,30,30,30,30,30,30,30,
30,30,30,30,30,31,31,32,34,36,
38,39,39,40,40,40,40,40,40,40,
40,40,40,40,40,41,41,42,44,46,
50,54,58,60,62,64,64,64,64,64,64};


void lift_manual()
{
	if (getJoystickValue(BtnRUp)== 1)
	{
		setMotorSpeed(liftMotorL, 100);
		setMotorSpeed(liftMotorR, 100);
	}
	else if (getJoystickValue(BtnRDown) == 1)
	{
		setMotorSpeed(liftMotorL, -100);
		setMotorSpeed(liftMotorR, -100);
	}
	else
	{
		setMotorSpeed(liftMotorL, 0);
		setMotorSpeed(liftMotorR, 0);
	}
}

void lift_preset()
{
	if (getJoystickValue(BtnRUp)== 1)
	{
		for (int i=0;i<LIFT_LEVELS-1;i=i+1)
		{
			if ( getMotorEncoder(liftMotorL) < ( iLiftLevel[i+1] - ARM_DELTA ) )
			{
				setMotorTarget(liftMotorL,iLiftLevel[i+1],100);
				setMotorTarget(liftMotorR, iLiftLevel[i+1], 100);
				wait1Msec( ( iLiftLevel[i+1] - iLiftLevel[i] ) * MS_PER_ENCODER_UNIT );
				break;
			}
		}
	}
	else if (getJoystickValue(BtnRDown)==1)
	{
		for (int i=LIFT_LEVELS-1;i>0;i=i-1)
		{
			if ( getMotorEncoder(liftMotorL) > ( iLiftLevel[i-1] + ARM_DELTA ) )
			{
				setMotorTarget(liftMotorL,iLiftLevel[i-1],100);
				setMotorTarget(liftMotorR, iLiftLevel[i-1],100);
				wait1Msec( ( iLiftLevel[i] - iLiftLevel[i-1] ) * MS_PER_ENCODER_UNIT );
				break;
			}
		}
	}
}

void scooper_manual()
{
	if (getJoystickValue(BtnFUp)== 1)
	{
		setMotorSpeed(scoopMotor, 100);
	}
	else if (getJoystickValue(BtnFDown) == 1)
	{
		setMotorSpeed(scoopMotor, -100);
	}
	else
	{
		setMotorSpeed(scoopMotor, 0);
	}
}

void scooper_preset()
{
	if (getJoystickValue (BtnFUp) == 1)
	{
		if (abs( getMotorEncoder(scoopMotor) - (turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[2])) < SCOOPER_DELTA )
		{//Scoop Balls
			turnNumber++;
			setMotorTarget(scoopMotor,turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[1],100);
			wait1Msec( abs( iScoopPos[2] - iScoopPos[1] + ENCODER_UNIT_PER_SCOOP_ROUND ) * MS_PER_ENCODER_UNIT );
			if ( abs(getMotorEncoder(scoopMotor) - (turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[1])) > SCOOPER_DELTA ) // bounce back if not get to the scooped position
			{
				turnNumber--;
				setMotorTarget(scoopMotor,turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[2],100);
				wait1Msec( abs( iScoopPos[2] - iScoopPos[1] + ENCODER_UNIT_PER_SCOOP_ROUND ) * MS_PER_ENCODER_UNIT );
			}
		}
		else
		{//Score Balls
			overwriteDrive = true;
			setMotorSpeed(leftMotor,0);
			setMotorSpeed(rightMotor,0);

			setMotorTarget(scoopMotor,turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[3],100);
			wait1Msec( abs( iScoopPos[3] - iScoopPos[1] ) * MS_PER_ENCODER_UNIT );
			wait1Msec(100);
			setMotorTarget(scoopMotor,turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[2],100);
			wait1Msec( abs( iScoopPos[3] - iScoopPos[2] ) * MS_PER_ENCODER_UNIT );

			overwriteDrive = false;
		}
	}
	else if ( ( getJoystickValue (BtnFDown) == 1 ) )
	{
		if (abs(getMotorEncoder(scoopMotor) - (turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[0])) < SCOOPER_DELTA )
		{//Release Cube
			setMotorTarget(scoopMotor,(turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND)+iScoopPos[1],100);
			wait1Msec( abs( iScoopPos[1] - iScoopPos[0] ) * MS_PER_ENCODER_UNIT );
		}
		else
		{//Pickup Cube
			setMotorTarget(scoopMotor,(turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND)+iScoopPos[0],100);
			wait1Msec( abs( getMotorEncoder(scoopMotor) - (turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[0]) ) * MS_PER_ENCODER_UNIT );
		}
	}
	else
	{
		setMotorSpeed(scoopMotor, 0);
	}
}


task task_lift()
{
	repeat(forever)
	{
		if (! overwriteLift)
		{
			//lift_preset();
			lift_manual();
		}

		wait1Msec(20);
	}
}

task task_scooper()
{
	repeat(forever)
	{
		if (! overwriteScooper)
		{
			scooper_preset();
		//scooper_manual();
		}

		wait1Msec(20);
	}
}

task task_claw()
{
	int clawTarget = CLAW_CLOSE;

	repeat(forever)
	{
		if (getJoystickValue(BtnLUp)==1 && time1[T2] > 1000 )
		{
			if ( abs(getMotorEncoder(clawMotor) - CLAW_CLOSE) < CLAW_DELTA*5 )
			{
				clawTarget = CLAW_OPEN;
				clearTimer(T2);
			}
			else
			{
				clawTarget = CLAW_CLOSE;
				clearTimer(T2);
			}
		}

		if (! overwriteClaw)
		{
			if ( abs(getMotorEncoder(clawMotor) - clawTarget) > CLAW_DELTA )
				setMotorTarget(clawMotor,clawTarget,100);
			else
				setMotorSpeed(clawMotor,0);
		}

		wait1Msec(20);
	}
}

void flip()
{
	overwriteDrive = true;
	overwriteLift = true;
	overwriteClaw = true;

	setMotorSpeed(leftMotor,-40);
	setMotorSpeed(rightMotor,-40);
	setMotorTarget(liftMotorL,FLIP_UP,100);
	setMotorTarget(liftMotorR,FLIP_UP,100);
	setMotorTarget(clawMotor,CLAW_OPEN,100);
	wait1Msec(700);

	setMotorTarget(liftMotorL,FLIP_DOWN,100);
	setMotorTarget(liftMotorR,FLIP_DOWN,100);
	wait1Msec(200);

	setMotorSpeed(leftMotor,60);
	setMotorSpeed(rightMotor,60);
	setMotorTarget(liftMotorL,FLIP_DOWN,100);
	setMotorTarget(liftMotorR,FLIP_DOWN,100);
	setMotorTarget(clawMotor,CLAW_CLOSE,100);
	wait1Msec(600);

	overwriteDrive = false;
	overwriteLift = false;
	overwriteClaw = false;
}

void trick1()
{
	overwriteDrive = true;
	overwriteScooper = true;

	setMotorSpeed(leftMotor, -40);
	setMotorSpeed(rightMotor, 40);
	wait1Msec(300);

	setMotorSpeed(leftMotor, 60);
	setMotorSpeed(rightMotor, 60);
	wait1Msec(800);

	turnNumber++;
	setMotorTarget(scoopMotor,turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[0],100);

	setMotorSpeed(leftMotor, 60);
	setMotorSpeed(rightMotor, 60);
	wait1Msec(500);

	setMotorSpeed(leftMotor, -60);
	setMotorSpeed(rightMotor, 60);
	wait1Msec(300);

	setMotorSpeed(leftMotor, 0);
	setMotorSpeed(rightMotor, 0);

	setMotorTarget(scoopMotor,turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[1],100);
	wait1Msec( abs( iScoopPos[1] - iScoopPos[0] ) * MS_PER_ENCODER_UNIT );

	overwriteDrive = false;
	overwriteScooper = false;

}

task main()
{
	setMotorEncoderUnits(encoderCounts);
	setMotorSpeed(liftMotorL, -100);
	setMotorSpeed(liftMotorR, -100);
	setMotorSpeed(clawMotor, -50);
	setMotorSpeed(scoopMotor, -60);
	wait1Msec(2000);
	setMotorSpeed(liftMotorL, 0);	resetMotorEncoder(liftMotorL);
	setMotorSpeed(liftMotorR, 0);	resetMotorEncoder(liftMotorR);
	setMotorSpeed(clawMotor, 0);	resetMotorEncoder(clawMotor);
	setMotorSpeed(scoopMotor, 0); resetMotorEncoder(scoopMotor);

	setMotorTarget(scoopMotor, iScoopPos[2], 100);
	setMotorTarget(clawMotor, CLAW_CLOSE, 100);
	setMotorTarget(liftMotorL, FLIP_UP, 100);
	setMotorTarget(liftMotorR, FLIP_UP, 100);
	wait1Msec(1000);

	startTask(task_scooper);
	startTask(task_lift);
	startTask(task_claw);
	clearTimer(T1);

	waitUntil(getJoystickValue(BtnLDown) == 1);
	trick1();

 	repeat (forever)
	{
/*
		if (getJoystickValue(BtnEDown) == 1 && time1[T1] > 1000 )
		{
				iDriveDirection = iDriveDirection * (-1);
				clearTimer(T1);
		}
*/
		if (getJoystickValue(BtnLDown) == 1)
			flip();

		iChA_filtered=iDriveMapping[abs(getJoystickValue(ChA))]*sgn(getJoystickValue(ChA))*iDriveDirection;
		iChC_filtered=iTurnMapping[abs(getJoystickValue(ChC))]*sgn(getJoystickValue(ChC));

		if (! overwriteDrive)
		{
			setMotorSpeed(leftMotor,(iChA_filtered+iChC_filtered)/1);
			setMotorSpeed(rightMotor,(iChA_filtered-iChC_filtered)/1);
		}

		wait1Msec(20);
	}

}
