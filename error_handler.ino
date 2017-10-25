void error_handler(int error) {

	while (true) {
		delay(500);
		switch (error)
		{
		case 1: Serial.println("Error in setting radio speed"); break;
		case 2: {digitalWrite(13, HIGH); break; }                                             //Turn on the led if battery voltage is to low
		default:
			Serial.println("Unknown error");
			break;
		}
	}
}
