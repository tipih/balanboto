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
	if ((abs(wheelVelocity) > 12) || (moveing == true)) {
		digitalWrite(A3, HIGH);
		moveing = true;
		restAngle -= (float)wheelVelocity / 4.0;
		restAngle = constrain(restAngle, balance_point - 10, balance_point + 10); // Limit rest Angle
		

	}





	restAngle = constrain(restAngle, lastRestAngle - 0.12, lastRestAngle + 0.12); // Don't change restAngle with more than 1 degree in each loop
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