#pragma config(Sensor, port1,  Gyro,           sensorVexIQ_Gyro)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
	resetGyro(Gyro);
	clearTimer(T1);

	while(true)
	{
		writeDebugStream("%f: Gyro: %f\n",time1[T1]/1000, getGyroDegreesFloat(Gyro));
		sleep(1000);
	}

}