#pragma config(Sensor, port10, LED,            sensorVexIQ_LED)
#pragma config(Sensor, port12, gyro,           sensorVexIQ_Gyro)
#pragma config(Motor,  motor1,          BR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor4,          liftMotorL,    tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor5,          BL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor6,          FL,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor7,          FR,            tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor11,         liftMotorR,    tmotorVexIQ, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define GYRO_SAMPLING_MILLISECONDS 10000

float fGyroDriftRate = 0;

float setGyroStable()
{
	setTouchLEDColor(LED,colorRed);
	setTouchLEDBlinkTime(LED, 14,6);

	wait1Msec(3000);
	setTouchLEDBlinkTime(LED, 0,1);
	wait1Msec(2000);

	setTouchLEDColor(LED,colorGreen);
	setTouchLEDBlinkTime(LED, 8, 12);

	resetGyro(Gyro);
	clearTimer(T4);
	wait1Msec(GYRO_SAMPLING_MILLISECONDS);
	setTouchLEDBlinkTime(LED, 4, 4);
	wait1Msec(1000);
	setTouchLEDColor(LED,colorNone);
	setTouchLEDBlinkTime(LED, 1, 0);

	fGyroDriftRate = getGyroDegreesFloat(Gyro) / GYRO_SAMPLING_MILLISECONDS;
	return fGyroDriftRate;
}

float getGyroStable()
{
	return getGyroDegreesFloat(Gyro) - fGyroDriftRate * time1[T4] / 1000;
}

//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
int GyroNull(){
	int Drift = 0;
	float DriftPerSecond = 0.0;
	resetGyro(Gyro);
	clearTimer(T1);
	while(true){
		if(timer1==600000){
			Drift = getGyroDegrees(Gyro);
			break;
		}
	}
	DriftPerSecond = Drift/60.0;
	return(DriftPerSecond);
}

float testBatt(){
	int nBatteryLevel = nImmediateBatteryLevel;
	return(nBatteryLevel*1000);
}
task main()
{
	clearTimer(T4);
	writeDebugStream("%f: Gyro and Timer Resetted\n", time1[T4]/1000);
	writeDebugStream("%f: Sampling Time = %d Seconds\n", time1[T4]/1000, GYRO_SAMPLING_MILLISECONDS);

	setGyroStable();
	writeDebugStream("%f: Sampled Drifting Rate = %f\n", time1[T4]/1000, fGyroDriftRate);

	resetGyro(Gyro);
	clearTimer(T4);

	while(true)
	{
		displayCenteredBigTextLine( 3, "%d, %d", getGyroStable(), getGyroDegrees(Gyro) );
		//writeDebugStream("%f:  RawGyro: %f, 839Gyro: %f\n",time1[T4]/1000, getGyroDegreesFloat(Gyro), gyroStablized());
		//sleep(1000);
	}
}
