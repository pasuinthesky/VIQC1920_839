#pragma config(Sensor, port10, LED,            sensorVexIQ_LED)
#pragma config(Sensor, port12, gyro,           sensorVexIQ_Gyro)
#pragma config(Motor,  motor1,          BR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor4,          liftMotorL,    tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor5,          BL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor6,          FL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor7,          FR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         liftMotorR,    tmotorVexIQ, PIDControl, encoder)
//ARE YA READY KIDS? AY AY CAPTAIN! I CAAANNNNNNNNNNNNNNNNNNNT HEAAAAAAAR YOUUUUUUUUUUUUU! AYY AYYY CAPTAIN! WHO LIVES IN A PINEAPPLE UNDER THE SEA? SPONGEBOB SQUARE PANTS! ABOSRBENT AND YELLOW AND POROUS IS HE? SPONGEBOB SQUAREPANTS! IF NAUTICAL NONSENSE BE SOMETHING YOU WISH? SPONGEBOB SQUAREPANTS! THEN DROP ON THE DECK AND FLOP LIKE A FISH! SPONGEBOB SQUAREPANTS! READY? SPONGEBOB SQUAREPANTS! SPONGEBOB SQUAREPANTS! SPONGEBOB SQUAREPANTS!
float desired_distance;
float dt = 25;
float allowed_time = 7.5;
float iChC_filtered = 0.0;
float iChB_filtered = 0.0;
float iChA_filtered = 0.0;
float desired_heading;
float progressToGoal = getMotorEncoder(BL) * 1.414;

typedef struct {
	float setpoint;
	float measured_value;
	float integral;
	float prev_error;
	float kp;
	float ki;
	float kd;
	float delta;
}structPID;

structPID pidDistance;
structPID pidOrientation;

float stayOnTrack()
{

}

float PIDControl (structPID &pid)
{
	float error = pid.measured_value - pid.setpoint;

	if ( abs(error) < pid.delta )
		error = 0;

	pid.integral = pid.integral + error * dt;
	float derivative = ( error - pid.prev_error ) / dt;
	float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
	pid.prev_error = error;

	return output;
}

void DriveStraightPID(int distance, int speed, float Kp, float Ki, float Kd, int delta)
{
	pidDistance.delta = delta;
	pidDistance.kp = Kp;
	pidDistance.ki = Ki;
	pidDistance.kd = Kd;
	pidDistance.measured_value = getMotorEncoder(BL) * 1.414;
	pidDistance.setpoint = distance;

	iChA_filtered = PIDControl(pidDistance);
	iChB_filtered = 0;
	iChC_filtered = 1;
	setMotorEncoderUnits(encoderCounts);

	setMotorSpeed( FL, 0 + iChA_filtered + iChB_filtered - iChC_filtered);
	setMotorSpeed( BL, 0 + iChA_filtered - iChB_filtered - iChC_filtered );
	setMotorSpeed( FR, 0 - iChA_filtered + iChB_filtered - iChC_filtered );
	setMotorSpeed( BR, 0 - iChA_filtered - iChB_filtered - iChC_filtered );
}

// remember these values: DriveStraightPID(20, 0.7, 0, 0, 5)
task main()
{
	DriveStraightPID(20, 30, 0.7, 0, 0, 5);

}
