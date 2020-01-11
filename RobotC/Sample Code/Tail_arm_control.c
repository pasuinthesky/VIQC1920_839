#define TAIL_UP 750
#define TAIL_DOWN 360
#define TAIL_DELTA 25

#define ARM_WAIT_RATIO 2.5


		//this sees if it is up or down, and then goes to the opposite direction(one button)
		if (getJoystickValue(BtnFUp)==1)
		{
			if ( abs(getMotorEncoder(tailMotor) - TAIL_DOWN) < TAIL_DELTA )
			{
				setMotorTarget(tailMotor,TAIL_UP,100);
				wait1Msec( ( TAIL_UP - TAIL_DOWN) * ARM_WAIT_RATIO );
			}
			else
			{
				setMotorTarget(tailMotor,TAIL_DOWN,100);
				wait1Msec( ( TAIL_UP - TAIL_DOWN) * ARM_WAIT_RATIO );
			}
		}
