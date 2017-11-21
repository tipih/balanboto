#include "radio_interface.h"

void updatePID(float restAngle, float offset, float turning, float dt) {

	
	//Only break until speed is low
	if ((abs(wheelVelocity)<1) && (moveing == true))
	{
		moveing = false;
		//radio_serial.println("Stop Moving");
		digitalWrite(A3, LOW);
		
	}


	//If somebody push the robot it will start to move, now it is time to do some magic
	if (((abs(wheelVelocity) > 12) || (moveing == true)) && (offset==0.00) ) {
		digitalWrite(A3, HIGH);
		moveing = true;
		restAngle -= (float)wheelVelocity / 4.0;
		restAngle = constrain(restAngle, balance_point - 10, balance_point + 10); // Limit rest Angle
		

	}
	else
	{
		if ((offset > 0.00 && wheelVelocity < 0) || (offset < 0 && wheelVelocity > 0.00) || offset == 0.00) // Scale down offset at high speed - wheel velocity is negative when driving forward and positive when driving backward
			offset += (float)wheelVelocity / 4; // We will always compensate if the offset is 0, but steerStop is not set
		
		
		//Serial.println(offset);
		restAngle -= offset;
	}





	restAngle = constrain(restAngle, lastRestAngle - 0.15, lastRestAngle + 0.15); // Don't change restAngle with more than 1 degree in each loop
	lastRestAngle = restAngle;
	
	//radio_serial.println(restAngle);
	
	//Serial.println(restAngle); //Serial.print(" "); Serial.print(wheelVelocity); Serial.print(" "); Serial.println(balance_point - kalAngleY);
	













	/* Update PID values */
	float error = restAngle - kalAngleY;
	//radio_serial.println(error);
	float pTerm = kP * error;
	iTerm += kI * 100.0f * error * dt; // Multiplication with Ki is done before integration limit, to make it independent from integration limit value
	iTerm = constrain(iTerm, -100.0f, 100.0f); // Limit the integrated error - prevents windup
	float dTerm = (kD / 100.0f) * (error - lastError) / dt;
	lastError = error;
	float PIDValue = pTerm + iTerm + dTerm;

	//Serial.println(kalAngleY);


	/* Steer robot sideways */
	if (turning < 0) { // Left
		turning += abs((float)wheelVelocity / 4); // Scale down at high speed
		if (turning > 0)
			turning = 0;
	}
	else if (turning > 0) { // Right
		turning -= abs((float)wheelVelocity / 4); // Scale down at high speed
		if (turning < 0)
			turning = 0;
	}



	float PIDLeft = PIDValue + turning+ engien_deadband;
	float PIDRight = PIDValue - turning+ engien_deadband;

	PIDLeft *= engien_offsetL; // Compensate for difference in some of the motors
	PIDRight *= engien_offsetR;
	/* Set PWM Values */

#ifdef DEBUG_ENGIEN
	Serial.print("PID L =");
	Serial.println(PIDLeft);

	//Serial.print("PID R =");
	//Serial.println(PIDRight);
#endif


	if (PIDLeft >= 0)
		moveMotor(left, backward, PIDLeft);
	else
		moveMotor(left, forward, -PIDLeft);
	if (PIDRight >= 0)
		moveMotor(right, backward, PIDRight);
	else
		moveMotor(right, forward, -PIDRight);


}