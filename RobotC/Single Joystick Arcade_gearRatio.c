
#pragma config(Motor,  motor1,          rightMotor,     tmotorVexIQ, openLoop, driveLeft, encoder)
#pragma config(Motor,  motor10,          lift,          tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor6,          leftMotor,    tmotorVexIQ, openLoop, reversed, driveRight, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
	int iPower, iTurn, iTurnPrevious=0;
	int iLowerThreshold=30, iUpperThreshold=95;
	int iGearRatio = 3;

	repeat (forever)
	{
		iPower=getJoystickValue(ChA);
		iTurn=getJoystickValue(ChB);

		if ( abs(iPower) > iUpperThreshold )
		{
			iPower=100*sgn(iPower);
		}
		else if ( abs(iPower) < iLowerThreshold )
		{
			iPower=0;
		}
		else
		{
			iPower=iPower;
		}

		if ( abs(iTurn) > iUpperThreshold )
		{
			iTurnPrevious=iTurn;
			iTurn=100*sgn(iTurn)/iGearRatio;
		}
		else if ( abs(iTurn) < iLowerThreshold )
		{
			iTurnPrevious=iTurn;
			iTurn=0;
		}
		else
		{
			iTurn=abs(iPower)*sgn(iTurn)/iGearRatio;
		}

		setMotorSpeed(leftMotor, (iPower+iTurn)/(1.0+1.0/iGearRatio));
		setMotorSpeed(rightMotor, (iPower-iTurn)/(1.0+1.0/iGearRatio));

		if (getJoystickValue(BtnRUp)== 1)
		{
			setMotorSpeed(lift, -100);
		}
		else if (getJoystickValue(BtnRDown)==1)
		{
			setMotorSpeed(lift, 100);
		}
		else
		{
			setMotorSpeed(lift, 0);
		}

		wait10Msec(2);
	}
}
