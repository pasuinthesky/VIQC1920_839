#pragma config(Motor,	motor6,		leftMotor,	tmotorVexIQ, PIDControl, reversed, driveLeft, encoder)
#pragma config(Motor,	motor1,		rightMotor,	tmotorVexIQ, PIDControl, driveRight, encoder)
#pragma config(Motor,	motor10,	armMotor, tmotorVexIQ, PIDControl, encoder)

task main()
{
	repeat (forever)
	{
		int leftValue, rightValue;

		leftValue = getJoystickValue(ChA);
		rightValue = getJoystickValue(ChD);
		if (abs(leftValue) > 10 || abs(rightValue) > 10)
		{
			setMotorSpeed(leftMotor, leftValue);
			setMotorSpeed(rightMotor, rightValue);
		}
		else
		{
			setMotorSpeed(leftMotor, 0);
			setMotorSpeed(rightMotor, 0);
		}

		if (getJoystickValue(BtnRUp)== 1)
		{
			setMotorSpeed(armMotor, -100);
		}
		else if (getJoystickValue(BtnRDown)==1)
		{
			setMotorSpeed(armMotor, 100);
		}
		else
		{
			setMotorSpeed(armMotor, 0);
		}
	}
}
