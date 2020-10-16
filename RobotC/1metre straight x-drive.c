#pragma config(Sensor, port10, LED,            sensorVexIQ_LED)
#pragma config(Sensor, port12, gyro,           sensorVexIQ_Gyro)
#pragma config(Motor,  motor1,          BR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor4,          liftMotorR,    tmotorNone, openLoop)
#pragma config(Motor,  motor5,          BL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor6,          FL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor7,          FR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         liftMotorL,    tmotorVexIQ, PIDControl, reversed, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
	float TargetDegree;
	int motorSpeed = 10;

	TargetDegree = 100/20/3*360;

	resetMotorEncoder(FL);
	setMotorBrakeMode(FL, motorHold);
	setMotorBrakeMode(FR, motorHold);
	setMotorBrakeMode(BL, motorHold);
	setMotorBrakeMode(BR, motorHold);

	while ( getMotorEncoder(FL) < TargetDegree )
	{
    if ( getMotorEncoder(FL) < TargetDegree*0.6 )
		{
			setMotorSpeed(FL, 60);
			setMotorSpeed(FR, 60 );
			setMotorSpeed(BL, -1 * 60);
			setMotorSpeed(BL, -1 * 60);
		}
		else
		{
			setMotorSpeed(FL, motorSpeed);
			setMotorSpeed(BL, motorSpeed);
			setMotorSpeed(FR, -1 * motorSpeed);
			setMotorSpeed(BR, -1 * motorSpeed);
		}
	}
		setMotorSpeed(FL, 0);
		setMotorSpeed(FR, 0);
		setMotorSpeed(BL, 0);
		setMotorSpeed(BR, 0);
}
