void setup_radio(String boudrate, int cmdPin,int serial_speed)
{

	pinMode(cmdPin, OUTPUT);	//Set command pin to output
	digitalWrite(cmdPin, LOW);	//Set the command line to LOW, it will ensure that the IC will be in the command mode
	delay(200);
	radio_serial.println(boudrate);		//Send the AT command 
	delay(200);							//Wait 100 mS for Data to arrive
	
	int size = read_radio_input(1,5);
	radio_read_buffer[size++] = '\0';	//Terminate the buffer

#ifdef DEBUG
	Serial.println("Radio buffer");
	Serial.print(radio_read_buffer[0]); //Lets check what is inside the buffer
	Serial.println(size);
#endif // DEBUG	

	if (radio_read_buffer[0] != 'Y')	//Check if we manage to set the speed
		error_handler(1);				//If not call the error handler
		

#ifdef DEBUG
	//TODO fix error
	//Serial.print(radio_read_buffer); //Lets check what is inside the buffer
#endif // DEBUG					
	
	radio_serial.println("AT+INF");		//Send the AT command


	delay(50);							//Wait 100 mS for Data to arrive
	
	size = read_radio_input(88,55);
	Serial.print("Buffer size from AT+INF ");
	Serial.println(size, DEC);
	radio_read_buffer[size++] = '\0';
	

		/*TODO create function to check we have communication up running*/

#ifdef DEBUG
		//Serial.println(radio_read_buffer); //Lets check what is inside the buffer
#endif // DEBUG
		radio_serial.end();
		digitalWrite(cmdPin, HIGH);	//Set the command line to LOW, it will ensure that the IC will be in the command mode
		
		delay(200);
		radio_serial.begin(serial_speed);	//Setup Radio to Baud 6 = 57600
		
		delay(1500);
		
		di.AngleGyro = kP;
		di.SelfBalancePidSetpoint = kI;
		di.PidSetpoint = kD;
		
		di.id = 0x01;
		di.end = 0xff;


			
			radio_write((char *)&di);
			delay(500);

		di.id = 0x02;
		di.AngleGyro = balance_point;
		di.SelfBalancePidSetpoint = 0.0;
		di.PidSetpoint = 0.0;
		radio_write((char *)&di);
		delay(500);


}

void read_radio()
{
	/*If there is no radio data, then just skib everything*/
	if (radio_serial.available() == 0)
	{
		return;
	}
	/*Section to read the radio, we expect 5 byte, with a header and 2 end byte*/
	//TODO figure out what to return
	clear_radio_buffer();
	int size = read_radio_input(14,200);
	
	Serial.println(size);
	
	if (size == 14) {

		/*If we got 14 byte then we should have everything*/
		/*Now we convert the data to float*/
		radio_analyze_data();


#ifdef Radio_data
		Serial.println();
		Serial.print("Buffer size ");
		Serial.print(size, DEC);
		Serial.print(" KP=");
		Serial.print(kP, 2);
		Serial.print(" KI=");
		Serial.print(kI, 2);
		Serial.print(" KD=");
		Serial.println(kD, 2);
#endif // Radio_data


	}


}



void clear_radio_buffer() {
	for (int a = 0; a<sizeof(radio_read_buffer); a++)
		radio_read_buffer[a] = '/0';
}

int read_radio_input(int num_of_bytes,int timeout) {
	
	int radio_input_local = 0;
	
	long current_time = millis();
	
	

	while ((radio_input_local < num_of_bytes) && (millis() - current_time < timeout)) {
		
		while (radio_serial.available() > 0) //Lets read in all the bytes
		{
			radio_read_buffer[radio_input_local++] = radio_serial.read();	//Read into buffer
			if (radio_input_local >= sizeof(radio_read_buffer) - 1) break;	//only accept buffer size

		}
		
		
	}
	
	return radio_input_local;
}

void radio_write(char* data) {
	radio_serial.write((uint8_t*)data, sizeof(debug_info));
	//Serial.println(sizeof(debug_info));
}
void radio_analyze_data(){


	//Datastruct for receiving data
	//1 byte id;
	//4 byte
	//4 byte  
	//4 byte 
	//1 byte end;
	Serial.println(radio_read_buffer[0],HEX);
	
	if (radio_read_buffer[0] == 0x00) //if message ID is 0 then this is a PID update
	{

		for (int a = 0; a < 4; a++) {
			b.c[a] = radio_read_buffer[1 + a];
		}

		if ((b.f > 0.0) && (b.f < 200.0))
			kP = b.f;

		for (int a = 0; a < 4; a++) {
			b.c[a] = radio_read_buffer[1 + 4 + a];
		}
		if ((b.f > 0.0) && (b.f < 200.0))
			kI = b.f;

		for (int a = 0; a < 4; a++) {
			b.c[a] = radio_read_buffer[1 + 8 + a];
		}
		if ((b.f > 0.0) && (b.f < 200.0))
			kD = b.f;
		configuration.kP = kP;
		configuration.kI = kI;
		configuration.kP = kD;


		Serial.print(" KP=");
		Serial.print(kP, 2);
		Serial.print(" KI=");
		Serial.print(kI, 2);
		Serial.print(" KD=");
		Serial.println(kD, 2);
		
	}
	else if (radio_read_buffer[0] == 0x01) {
	//If this is a cmd 1, we will save the PID values to EEPROM
	//This approce is to ensure that we do not write to many time to EEPROM
	//as it has a limitation of 100000 writings
		EEPROM_writeAnything(0, configuration);
	}
	else if (radio_read_buffer[0] == 0x02) {
		if (radio_read_buffer[1] == 0x01)
		 analyze_data = true; //
		else
			analyze_data = false; //
	}
	else if (radio_read_buffer[0] == 0x03) {
		for (int a = 0; a < 4; a++) {
			b.c[a] = radio_read_buffer[1 + a];
		}

		if ((b.f > 0.0) && (b.f < 5.0))
			balance_point = b.f;

		Serial.println(balance_point);
		targetAngle = balance_point;

	}
}



	