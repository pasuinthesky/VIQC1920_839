#pragma config(Motor,motor6,				leftMotor,				tmotorVexIQ,PIDControl,reversed,driveLeft,encoder)
#pragma config(Motor,motor1,				rightMotor,				tmotorVexIQ,PIDControl,driveRight,encoder)
#pragma config(Motor,motor10,			armMotor,					tmotorVexIQ,PIDControl,encoder)

task main()
{
	repeat(forever)
	{
		setMotorSpeed(leftMotor,(getJoystickValue(ChA)+getJoystickValue(ChC))/2);
		setMotorSpeed(rightMotor,(getJoystickValue(ChA)-getJoystickValue(ChC))/2);

		if(getJoystickValue(BtnRUp)==1)
		{
			setMotorSpeed(armMotor,-100);
		}
		else if(getJoystickValue(BtnRDown)==1)
		{
			setMotorSpeed(armMotor,100);
		}
		else
		{
			setMotorSpeed(armMotor,0);
		}
	}
}
