#include "radio_interface.h"
void setup_PWM()
{
/*Setup the PWM to 20kHz this will be timer 1 pins will be 9 and 10
OCR1A will be pin 9
OCR1B will be pin 10
Value between 0 - 799
*/
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);
	TCCR1A =
		(1 << COM1A1) | (0 << COM1A0) |   // COM1A1, COM1A0 = 1, 0
		(1 << COM1B1) | (0 << COM1B0) |
		(1 << WGM11) | (0 << WGM10);      // WGM11, WGM10 = 1, 0

	ICR1 = 799;
	TCNT1 = 0;

	OCR1A = 0;
	OCR1B = 0;
	TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (1 << TOIE1); //Enable overflow interrupt, this will be use for generating a pulse to the ultrasonic trigger

	DDRB |= (1 << DDB1);
	DDRB |= (1 << DDB2);

	TCCR1B =
		(0 << ICNC1) | (0 << ICES1) |
		(1 << WGM13) | (1 << WGM12) |              // WGM13, WGM12 = 1, 1
		(0 << CS12) | (0 << CS11) | (1 << CS10);
}

void moveMotor(Command motor, Command direction, float speedRaw) { // Speed is a value in percentage 0-100%
	if (speedRaw > 100.0f)
		speedRaw = 100.0f;
	setPWM(motor, speedRaw * (float)PWMVALUE / 100.0f); // Scale from 0-100 to 0-PWMVALUE
	if (motor == left) {
		if (direction == forward) {
			LeftA = LOW;
			LeftB = HIGH;

			//leftA::Clear();
			//leftB::Set();
		}
		else {
			LeftA = HIGH;
			LeftB = LOW;

			
			//leftA::Set();
			//leftB::Clear();
		}
	}
	else {
		if (direction == forward) {
			RightA = HIGH;
			RightB = LOW;
			//rightA::Set();
			//rightB::Clear();
		}
		else {
			RightA = LOW;
			RightB = HIGH;
			//rightA::Clear();
			//rightB::Set();
		}
	}
}


void stopMotor(Command motor) {
	
	setPWM(motor, PWMVALUE); // Set high
	if (motor == left) {
		LeftA = HIGH;
		LeftB = HIGH;

	}
	else {
		RightA = HIGH;
		RightB = HIGH;

	}
}

void setPWM(Command motor, uint16_t dutyCycle) { // dutyCycle is a value between 0-ICR1


	if (dutyCycle >= 800) dutyCycle = 799;

	//#ifdef DEBUG
	//Serial.println(dutyCycle);
	//#endif // DEBUG

	if (motor == left)
		OCR1A = dutyCycle;
	else
		OCR1B = dutyCycle;
}