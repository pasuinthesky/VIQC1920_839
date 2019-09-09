#pragma config(Motor,	motor7,			leftMotor,	tmotorVexIQ, PIDControl, reversed, driveLeft, encoder)
#pragma config(Motor,	motor12,		rightMotor,	tmotorVexIQ, PIDControl, driveRight, encoder)

int iChA_filtered=0, iChC_filtered=0;
int iDeadzone = 10;

task main()
{
	repeat (forever)
	{
		if ( abs(getJoystickValue(ChA)) < iDeadzone)
		{
			iChA_filtered = 0;
		}
		else
		{
			iChA_filtered = getJoystickValue(ChA);
		}

		if ( abs(getJoystickValue(ChC)) < iDeadzone)
		{
			iChC_filtered = 0;
		}
		else
		{
			iChC_filtered = getJoystickValue(ChC);
		}

		setMotorSpeed(leftMotor,(iChA_filtered+iChC_filtered)/2);
		setMotorSpeed(rightMotor,(iChA_filtered-iChC_filtered)/2);
	}
}
