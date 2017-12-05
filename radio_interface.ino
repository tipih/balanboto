void setup_radio(String boudrate, int cmdPin, int serial_speed)
{

	pinMode(cmdPin, OUTPUT);	//Set command pin to output
	digitalWrite(cmdPin, LOW);	//Set the command line to LOW, it will ensure that the IC will be in the command mode
	delay(200);
	radio_serial.println(boudrate);		//Send the AT command 
	delay(200);							//Wait 100 mS for Data to arrive

	int size = read_radio_input(1, 5);
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

	size = read_radio_input(88, 55);
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
	//radio_serial.println("If you read this, then Radio is configures ok");
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
	int size = read_radio_input(14, 200);



	if (size == 14) {

		/*If we got 14 byte then we should have everything*/
		/*Now we convert the data to float*/
		radio_analyze_data();

	}


}

void read_radio_control() {
	if (radio_serial.available() == 0)
	{
		return;
	}

	clear_radio_buffer();
	int size = read_radio_input(2, 10);



	if (size == 2) {

		/*If we got 2 byte then we should have everything*/
		/*Now we convert the data to float*/
		get_control();

	}
}



void clear_radio_buffer() {
	for (int a = 0; a < sizeof(radio_read_buffer); a++)
		radio_read_buffer[a] = '/0';
}

int read_radio_input(int num_of_bytes, int timeout) {

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



/*2 byte for remote control*/
void get_control() {
	msgCommandStruct = (struct _msgCommand *)&radio_read_buffer;
	
	byte speed = msgCommandStruct->speed;
	byte turn = msgCommandStruct->turn;

	targetOffset = map(speed, 0, 0xff, -8, 8);
	turningOffset = map(turn, 0, 0xff, -30, 30);

	if ((speed > 100) && (speed < 130)) targetOffset = 0;
	if ((turn > 100) && (turn < 130)) turningOffset = 0;

	Serial.println("Remote "); Serial.println(" "); Serial.println(targetOffset); Serial.println(" "); Serial.println(turningOffset);
}





/*Main accespoint for analyzing data comming from PC, everything must be send as Little Endian form PC, so if PC is Big Endian a conversion must be done*/
/*We have the following message type for  14 bytes 1 byte id 12 byte for different types 1 byte for end*/
/*MSG1 1 byte ID 3*4 byte float 1 byte end*/
/*MSG2 1 byte ID 4 byte unsigned long 2*4 byte long 1 byte end*/
/*MSG3 1 byte ID 14*1 byte 1 byte end*/
/*MSG4 1 byte ID 1*4 byte unsigned long 1*4 byte long 1*4 byte float 1 byte end*/
void radio_analyze_data() {


	//Datastruct for receiving data
	//1 byte id;
	//4 byte
	//4 byte  
	//4 byte 
	//1 byte end;


#ifdef Radio_data

	Serial.print(radio_read_buffer[0], HEX);
	Serial.print("---------");

	Serial.print(radio_read_buffer[1], HEX);
	Serial.print(radio_read_buffer[2], HEX);
	Serial.print(radio_read_buffer[3], HEX);
	Serial.print(radio_read_buffer[4], HEX);
	Serial.print("---------");
	Serial.print(radio_read_buffer[5], HEX);
	Serial.print(radio_read_buffer[6], HEX);
	Serial.print(radio_read_buffer[7], HEX);
	Serial.print(radio_read_buffer[8], HEX);
	Serial.print("---------");
	Serial.print(radio_read_buffer[9], HEX);
	Serial.print(radio_read_buffer[10], HEX);
	Serial.print(radio_read_buffer[11], HEX);
	Serial.print(radio_read_buffer[12], HEX);
	Serial.print("---------");
	Serial.print(radio_read_buffer[13], HEX);
	Serial.println("---------");
#endif // Radio_data


	if (radio_read_buffer[13] != 0xFF) return; //Check of end byte is 0xff if not something when wrong




	byte msgID = radio_read_buffer[0];

	switch (msgID)
	{
	case 0:
		dataStruct = (struct _msgtype1 *)&radio_read_buffer;
		getPID();
		break;
	case 1:
		dataStruct1 = (struct _msgtype2 *)&radio_read_buffer;
		Serial.println(configuration.kP);
		Serial.println(configuration.kI);
		Serial.println(configuration.kD);

		EEPROM_writeAnything(0, configuration);
		Serial.println("Write to EEprom");
		break;
	case 2:
		dataStruct2 = (struct _msgtype3 *)&radio_read_buffer;
		break;
	case 3:
		dataStruct3 = (struct _msgtype4 *)&radio_read_buffer;
		configuration.bP = balance_point = dataStruct3->data3;
		targetAngle = balance_point;
		break;
	case 4:
		if (radio_read_buffer[1] == 0x01)
			analyze_data = true; //
		else
			analyze_data = false; //
		break;
	case 5:
		dataStruct = (struct _msgtype1 *)&radio_read_buffer;
		engien_deadband = dataStruct->data1;
		engien_offsetR = dataStruct->data2;


		break;
	case 6:
		dataStruct2 = (struct _msgtype3 *)&radio_read_buffer;

		if (dataStruct2->data3 == 0x01)
			buttonFlag = true;
		else if (dataStruct2->data3 == 0x02)
			buttonFlag = false;

		switch (dataStruct2->data1) {
		case 0: Serial.println("stop"); targetOffset = 0; turningOffset = 0; break;//Forward
		case 1: Serial.println("forward"); {
			byte speed = dataStruct2->data2;
			speed = map(speed, 0xa0, 0xfa, 1, 8);
			targetOffset = -speed;
			Serial.println(targetOffset);
			break;//Forward
		}
		case 2: Serial.println("backward"); {
			byte speed = dataStruct2->data2;
			speed = map(speed, 0xa0, 0xfa, 1, 8);
			targetOffset = speed;
			Serial.println(targetOffset);
			break; //Backward			
		}
		case 3: Serial.println("left"); {
			byte speed = dataStruct2->data2;
			speed = map(speed, 0xa0, 0xfa, 1, 30);
			turningOffset = speed;
			Serial.println(turningOffset);
			break; //Left
		}
		case 4: Serial.println("right"); {
			byte speed = dataStruct2->data2;
			speed = map(speed, 0xa0, 0xfa, 1, 30);
			turningOffset = -speed;
			Serial.println(turningOffset);
			break; //Right
		}



		}
		break;
	case 7:
		if (radio_read_buffer[1] == 0x01)
			enableSensor = true; //
		else
			enableSensor = false; //
		break;

	default:
		break;
	}






}



void getPID() {

	kP = dataStruct->data1;
	kI = dataStruct->data2;
	kD = dataStruct->data3;
	configuration.kP = kP;
	configuration.kI = kI;
	configuration.kD = kD;


	Serial.print(" KP=");
	Serial.print(configuration.kP, 2);
	Serial.print(" KI=");
	Serial.print(configuration.kI, 2);
	Serial.print(" KD=");
	Serial.println(configuration.kD, 2);
}



