#pragma config(Motor,  motor8,           leftMotor,     tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor2,           rightMotor,    tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor12,          scoopMotor,    tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,          liftMotorL,    tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor5,           liftMotorR,    tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor6,           clawMotor,     tmotorVexIQ, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define CLAW_CLOSE 480
#define CLAW_OPEN 40
#define CLAW_DELTA 100
#define SCOOPER_DELTA 35
#define ARM_DELTA 25
#define MS_PER_ENCODER_UNIT 1
#define ENCODER_UNIT_PER_SCOOP_ROUND 1920


#define LIFT_LEVELS 5
int iLiftLevel[LIFT_LEVELS] = {15 ,400, 1100, 1435, 1830}; //pickup_inside, pickup_outside/transport,low_inside,low_outside,,high_inside

int iScoopPos[4] = { -150, 250, 815, 1000 }; //pick_cube, keep_ball, ready_ball, score_ball

int iChA_filtered=0, iChC_filtered=0, turnNumber = 0;

int iDriveDirection = 1;

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

int iTurnMapping[101] = {
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
10,10,10,10,10,11,11,12,14,16,
18,19,19,20,20,20,20,20,20,20,
20,20,20,20,20,21,21,22,24,26,
28,29,29,30,30,30,30,30,30,30,
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
	else
	{
		setMotorSpeed( liftMotorL, 0 );
		setMotorSpeed( liftMotorR, 0 );
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
		{
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
		{
			setMotorTarget(scoopMotor,turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[3],100);
			wait1Msec( abs( iScoopPos[3] - iScoopPos[1] ) * MS_PER_ENCODER_UNIT );
			wait1Msec(100);
			setMotorTarget(scoopMotor,turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[2],100);
			wait1Msec( abs( iScoopPos[3] - iScoopPos[2] ) * MS_PER_ENCODER_UNIT );
		}
	}
	else if ( ( getJoystickValue (BtnFDown) == 1 ) )
	{
		if (abs(getMotorEncoder(scoopMotor) - (turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND+iScoopPos[0])) < SCOOPER_DELTA )
		{
			setMotorTarget(scoopMotor,(turnNumber*ENCODER_UNIT_PER_SCOOP_ROUND)+iScoopPos[1],100);
			wait1Msec( abs( iScoopPos[1] - iScoopPos[0] ) * MS_PER_ENCODER_UNIT );
		}
		else
		{
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
		//lift_preset();
		lift_manual();

		wait1Msec(20);
	}
}

task task_scooper()
{
	repeat(forever)
	{
		scooper_preset();
	//scooper_manual();

		wait1Msec(20);
	}
}

task main()
{
	setMotorEncoderUnits(encoderCounts);
	setMotorSpeed(liftMotorL, -100);
	setMotorSpeed(liftMotorR, -100);
	setMotorSpeed(clawMotor, -50);
	setMotorSpeed(scoopMotor, -60);
	wait(2);
	setMotorSpeed(liftMotorL, 0);	resetMotorEncoder(liftMotorL);
	setMotorSpeed(liftMotorR, 0);	resetMotorEncoder(liftMotorR);
	setMotorSpeed(clawMotor, 0);	resetMotorEncoder(clawMotor);
	setMotorSpeed(scoopMotor, 0); resetMotorEncoder(scoopMotor);

	setMotorTarget(scoopMotor, iScoopPos[2], 100);
	setMotorTarget(clawMotor, CLAW_OPEN, 100);
	setMotorTarget(liftMotorL, iLiftLevel[1], 100);
	setMotorTarget(liftMotorR, iLiftLevel[1], 100);
	wait(1);

	startTask(task_scooper);
	startTask(task_lift);
	clearTimer(T1);

 	repeat (forever)
	{
		if (getJoystickValue(BtnEDown) == 1)
		{
			if (time1[T1] > 1000)
			{
				iDriveDirection = iDriveDirection * (-1);
				clearTimer(T1);
			}
		}

		iChA_filtered=iDriveMapping[abs(getJoystickValue(ChA))]*sgn(getJoystickValue(ChA))*iDriveDirection;
		iChC_filtered=iTurnMapping[abs(getJoystickValue(ChC))]*sgn(getJoystickValue(ChC));

		setMotorSpeed(leftMotor,(iChA_filtered+iChC_filtered)/1);
		setMotorSpeed(rightMotor,(iChA_filtered-iChC_filtered)/1);

		if (getJoystickValue(BtnLUp)==1)
		{
			if ( abs(getMotorEncoder(clawMotor) - CLAW_OPEN) < CLAW_DELTA)
			{
				setMotorTarget(clawMotor,CLAW_CLOSE,100);
				wait1Msec( abs(CLAW_CLOSE - CLAW_OPEN) * MS_PER_ENCODER_UNIT );
			}
			else
			{
				setMotorTarget(clawMotor, CLAW_OPEN, 100);
				wait1Msec( abs(CLAW_CLOSE - CLAW_OPEN) * MS_PER_ENCODER_UNIT );
			}
		}
		else
		{
			setMotorSpeed(clawMotor, 0);
		}
		wait1Msec(20);
	}

}
