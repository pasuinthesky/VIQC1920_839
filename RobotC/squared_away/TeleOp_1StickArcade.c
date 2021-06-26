#pragma config(Motor,	motor7,			leftMotor,	tmotorVexIQ, PIDControl, reversed, driveLeft, encoder)
#pragma config(Motor,	motor12,		rightMotor,	tmotorVexIQ, PIDControl, driveRight, encoder)

task main()
{
	repeat (forever)
	{
		setMotorSpeed(leftMotor,getJoystickValue(ChA)+getJoystickValue(ChB)/2);
		setMotorSpeed(rightMotor,getJoystickValue(ChA)-getJoystickValue(ChB)/2);
	}
}
