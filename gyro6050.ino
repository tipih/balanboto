void setup_gyro() {
#ifdef DEBUG
	Serial.println("Starting Gyro up");
	Serial.println("This could be a MPU-6050");
	Wire.beginTransmission(IMUAddress);
	Wire.write(0x75);
	Wire.endTransmission();
	Serial.println("Send Who am I request...");
	Wire.requestFrom(0x68, 1);
	while (Wire.available() < 1);
	int lowByte = Wire.read();
	if (lowByte == IMUAddress) {
		Serial.print("Who Am I responce is ok: 0x");
		Serial.println(lowByte, HEX);
	}

#endif // DEBUG

	//By default the MPU-6050 sleeps. So we have to wake it up.
	Wire.beginTransmission(IMUAddress);                                     //Start communication with the address found during search.
	Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
	Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
	Wire.endTransmission();                                                   //End the transmission with the gyro.
																			  //Set the full scale of the gyro to +/- 250 degrees per second
	Wire.beginTransmission(IMUAddress);                                     //Start communication with the address found during search.
	Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
	Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
	Wire.endTransmission();                                                   //End the transmission with the gyro
																			  //Set the full scale of the accelerometer to +/- 4g.
	Wire.beginTransmission(IMUAddress);                                     //Start communication with the address found during search.
	Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
	Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
	Wire.endTransmission();                                                   //End the transmission with the gyro
																			  //Set some filtering to improve the raw data.
	Wire.beginTransmission(IMUAddress);                                     //Start communication with the address found during search
	Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
	Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
	Wire.endTransmission();                                                   //End the transmission with the gyro 
#ifdef DEBUG
	Serial.println("Gyto setup ok");
#endif // DEBUG
}

void calibrate_gyro() {
#ifdef DEBUG
	Serial.println("Calibrate the Gyro");
#endif // DEBUG
	gyro_yaw_calibration_value = gyro_pitch_calibration_value = 0;
	for (receive_counter = 0; receive_counter < 500; receive_counter++) {       //Create 500 loops
		if (receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));        //Change the state of the LED every 15 loops to make the LED blink fast
		Wire.beginTransmission(IMUAddress);                                   //Start communication with the gyro
		Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
		Wire.endTransmission();                                                 //End the transmission
		Wire.requestFrom(0x68, 4);                                      //Request 2 bytes from the gyro
		gyro_yaw_calibration_value += Wire.read() << 8 | Wire.read();               //Combine the two bytes to make one integer
		gyro_pitch_calibration_value += Wire.read() << 8 | Wire.read();             //Combine the two bytes to make one integer
		delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
	}
	gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
	gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset
#ifdef DEBUG
	Serial.println("Calibration Done");
	Serial.print(gyro_pitch_calibration_value);
	Serial.print(" ");
	Serial.println(gyro_yaw_calibration_value);

#endif // DEBUG

	while (i2cRead(0x3B, i2cData, 6));
	accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
	accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
	accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	double roll = atan2(accY, accZ) * RAD_TO_DEG;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);


}
/*Read gyro, should be called once pr loop*/
void read_gyro() {

	while (i2cRead(0x3B, i2cData, 14));
	accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
	accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
	accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
	tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
	gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
	gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
	gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

	gyroY - gyro_pitch_calibration_value;

	double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
	timer = micros();

	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	double roll = atan2(accY, accZ) * RAD_TO_DEG;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

	double gyroXrate = gyroX / 131.0; // Convert to deg/s
	double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
									  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
		kalmanX.setAngle(roll);
		kalAngleX = roll;

	}
	else
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleX) > 90)
		gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
	
	kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
									  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
		kalmanY.setAngle(pitch);
		compAngleY = pitch;
		kalAngleY = pitch;
		gyroYangle = pitch;
	}
	else
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleY) > 90)
		gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
	kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

	//gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
	//gyroYangle += gyroYrate * dt;
	//gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
	//gyroYangle += kalmanY.getRate() * dt;
	//compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
	//compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

	// Reset the gyro angle when it has drifted too much
	//if (gyroXangle < -180 || gyroXangle > 180)
	//	gyroXangle = kalAngleX;
	//if (gyroYangle < -180 || gyroYangle > 180)
	//	gyroYangle = kalAngleY;

	//Serial.print(kalAngleY); Serial.print("\t");
	//Serial.print(compAngleY); Serial.print("\t");
	//Serial.print("\r\n");
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
	uint32_t timeOutTimer;
	//Serial.println("Reading IMU");
	Wire.beginTransmission(IMUAddress);
	Wire.write(registerAddress);
	uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
	if (rcode) {
		Serial.print(F("i2cRead failed: "));
		Serial.println(rcode);
		return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
	}
	Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
	for (uint8_t i = 0; i < nbytes; i++) {
		if (Wire.available())
			data[i] = Wire.read();
		else {
			timeOutTimer = micros();
			while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
			if (Wire.available())
				data[i] = Wire.read();
			else {
				Serial.println(F("i2cRead timeout"));
				return 5; // This error value is not already taken by endTransmission
			}
		}
	}
	//Serial.println("End reading");
	return 0; // Success
}
