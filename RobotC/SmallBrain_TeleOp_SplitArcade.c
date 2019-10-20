#pragma config(Motor,  motor1,          leftMotor,     tmotorVexIQ, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motor6,          rightMotor,    tmotorVexIQ, PIDControl, reversed, driveRight, encoder)
#pragma config(Motor,  motor7,          clawMotor,     tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         tailMotor,     tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor12,         armMotor,      tmotorVexIQ, PIDControl, reversed, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define GEAR_RATIO 1.5

#define ARM_SPEED 100
#define TAIL_SPEED 60
#define CLAW_SPEED 100

#define CLAW_OPEN -20
#define CLAW_CLOSE -170
#define CLAW_DELTA 25

#define TAIL_UP 150
#define TAIL_DOWN 10
#define TAIL_DELTA 25

#define ARM_DELTA 25
#define ARM_WAIT_RATIO 2.5


int iChA_filtered=0, iChC_filtered=0;
//
int iArmLv[4] = {0, 150, 375, 545};

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

// each number is a channel c reading, for example, the 20th 0 is actually a reading of 20
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

/*
int iTurnMapping[101] = {
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
30,30,30,30,30,30,30,30,30,30,
30,30,30,30,30,31,32,33,34,36,
38,40,40,40,40,40,40,42,46,48,
50,50,50,50,50,50,50,52,54,56,
58,58,59,59,60,60,60,60,60,60,
60,60,60,60,60,60,60,60,60,60,
60,60,60,62,62,63,63,63,63,63,
64,64,64,64,64,64,64,64,64,64,64};
*/

task main()
{
// move every thing to ready position
	setMotorSpeed(clawMotor, 100);
	setMotorSpeed(tailMotor, -100);
	setMotorSpeed(armMotor, -100);
	wait(2);
	resetMotorEncoder(clawMotor);
	resetMotorEncoder(tailMotor);
	resetMotorEncoder(armMotor);

	setMotorTarget(clawMotor, -10, 50);
	setMotorTarget(tailMotor, 10, 50);
	setMotorTarget(armMotor, 10, 50);

	repeat (forever)
	{
		//this issplit arcade with idrivemapping as going forward and iturnmapping as chc, or turning
		iChA_filtered=iDriveMapping[abs(getJoystickValue(ChA))]*sgn(getJoystickValue(ChA));
		iChC_filtered=iTurnMapping[abs(getJoystickValue(ChC))]*sgn(getJoystickValue(ChC));
		//iChC_filtered=iDriveMapping[abs(getJoystickValue(ChC))]*sgn(getJoystickValue(ChC))/GEAR_RATIO

		setMotorSpeed(leftMotor,(iChA_filtered+iChC_filtered)/1);
		setMotorSpeed(rightMotor,(iChA_filtered-iChC_filtered)/1);


		//this sees if it is up or down, and then goes to the opposite direction(one button)
		if (getJoystickValue(BtnLDown)==1)
		{
			if ( abs(getMotorEncoder(tailMotor) - TAIL_DOWN) < TAIL_DELTA )
			{
				setMotorTarget(tailMotor,TAIL_UP,100);
				wait1Msec(500);
			}
			else
			{
				setMotorTarget(tailMotor,TAIL_DOWN,100);
				wait1Msec(500);
			}
		}

//
    if (getJoystickValue(BtnRUp)== 1)
		{
			for (int i=0;i<3;i=i+1)
			{
				if ( getMotorEncoder(armMotor) < ( iArmLv[i+1] - ARM_DELTA ) )
				{
					setMotorTarget(armMotor,iArmLv[i+1],100);
					wait1Msec( ( iArmLv[i+1] - iArmLv[i] ) * ARM_WAIT_RATIO );
					break;
				}
			}
		}
		else if (getJoystickValue(BtnRDown)==1)
		{
			for (int i=3;i>0;i=i-1)
			{
				if ( getMotorEncoder(armMotor) > ( iArmLv[i-1] + ARM_DELTA ) )
				{
					setMotorTarget(armMotor,iArmLv[i-1],100);
					wait1Msec( ( iArmLv[i] - iArmLv[i-1] ) * ARM_WAIT_RATIO );
					break;
				}
			}
		}
		else
		{
			setMotorSpeed(armMotor, 0);
		}

		if (getJoystickValue(BtnLUp)==1)
// checks the level before opening if on low r high platform then back up after
		{
			if ( abs(getMotorEncoder(clawMotor) - CLAW_OPEN) < CLAW_DELTA )
			{
				setMotorTarget(clawMotor,CLAW_CLOSE,100);
				wait1Msec( abs( CLAW_CLOSE - CLAW_OPEN ) * ARM_WAIT_RATIO );
				if ( getMotorEncoder(armMotor) < ( iArmLv[1] - ARM_DELTA ) )
				{
					setMotorTarget(armMotor,iArmLv[1],100);
					setMotorSpeed(leftMotor,0);
					setMotorSpeed(rightMotor,0);
					wait1Msec( abs( iArmLv[1] - getMotorEncoder(armMotor) ) * ARM_WAIT_RATIO );
				}
			}
			else
			{
				setMotorTarget(clawMotor,CLAW_OPEN,100);
				wait1Msec( abs( CLAW_CLOSE - CLAW_OPEN ) * ARM_WAIT_RATIO );
				if ( getMotorEncoder(armMotor) > ( iArmLv[2] - ARM_DELTA ) )
				{
					setMotorSpeed(leftMotor,-100);
					setMotorSpeed(rightMotor,-100);
					wait1Msec(400);
					setMotorSpeed(leftMotor,0);
					setMotorSpeed(rightMotor,0);
				}
				setMotorTarget(armMotor,iArmLv[0],100);
				wait1Msec( abs( getMotorEncoder(armMotor) - iArmLv[0] ) * ARM_WAIT_RATIO );
			}
		}
	}
}
