#include <SoftwareSerial.h>
#include <Encoder.h>
#include <DirectIO.h>
#include "robot.h"
#include "RunningAverage.h"
#include <Wire.h>
#include "writeToEeprom.h"
#include <EEPROM.h>

#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter





//Frequency Setup
static const uint16_t PWM_FREQUENCY = 20000; // The motor driver can handle a PWM frequency up to 20kHz
static const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 1; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2


const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

/*Gyro const and VARS*/
const uint8_t IMUAddress = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)
static const int acc_calibration_value = 1000;                            //Enter the accelerometer calibration value
static float balance_point = 2.1;
long gyro_yaw_calibration_value, gyro_pitch_calibration_value;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
int16_t tempRaw;
uint8_t i2cData[14]; // Buffer for I2C data


//Timers
static uint32_t pidTimer; // Timer used for the PID loop
uint32_t timer;
unsigned long loop_timer;
unsigned long test_timer;
unsigned long bat_timer;
unsigned long encoderTimer;
int loop_time = loop_timing;
int current_time;

//Kalman instance, for filtering the IMU
Kalman kalmanX; 
Kalman kalmanY;
float Qangle = 0.001f;
float Qbias = 0.003f;
float Rmeasure = 0.03f;

//Filtering for battery
RunningAverage myRA(50);

/*General VARS*/

int battery_voltage;
int receive_counter;				//Byte received via the wireless
byte start, low_bat;				//State vars, this will handle if the robot should balancing

//Vars for controlling the Robot's movement, also via the remote control
static bool steerStop = true;		// Stop by default
static bool stopped=false;				// This is used to set a new target position after braking
static float targetOffset = 0.0f;	// Offset for going forward and backward
static float turningOffset = 0.0f;	// Offset for turning left and right
static float targetAngle; // Resting angle of the robot
static float lastError; // Store last angle error


//General PID VARS
static float iTerm; // Store iTerm
float kP, kI, kD; // PID variables


//General engien vars
static float engien_deadband = 1.0f;
static float engien_offsetL = 1.00f;
static float engien_offsetR = 0.95f;

//Position vars
static int32_t lastWheelPosition; // Used to calculate the wheel velocity
static int32_t wheelVelocity; // Wheel velocity based on encoder readings
static int32_t targetPosition; // The encoder position the robot should be at
static float lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one
static bool moveing = false;


//Flag for start sending data from Robot to PC, reason for having this was that radio can't send a receive at the same time, so if sending
//data at a high speed from robot to PC, ajusting PID value from PC to Robot could course wrong data i both directions, this could be fixed 
//with retransmission, and checsum, but there is no need for that solution
bool analyze_data = false;

//input buffer for radio interface
unsigned char radio_read_buffer[64];
char radio_input;








struct debug_info di;




//Class init of radio and wheel encoders
SoftwareSerial radio_serial(radio_serial_rx, radio_serial_tx);
Encoder leftEncode(leftEncoderPin1, leftEncoderPin2);		//Setup left the encoder for pin 1-2
//Encoder rightEncoder(rightEncoderPin1, rightEncoderPin2);	//Setup right the encoder for pin 1-2


void setup()
{
 /*Setup Serial for debugging*/
	Wire.begin();                                                             //Start the I2C bus as master
	TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

	Serial.begin(19200);
	Serial.println("Starting up project Robot V1");
	
	myRA.clear();																//Clear the running avaged filter
	int speed = radio_serial_speed;
	radio_serial.begin(9600);
	read_from_eeprom();															//Read the PID and more value from eeprom
	setup_radio("AT+BAUD=4", radio_serial_cmd_pin, speed);						//Setup the Radio for communication
	setup_PWM();																//Init the PWM to 20kHz
	setup_gyro();																//Look for the Gyro at address 0x68
	calibrate_gyro();															//Calibrate the Gyro

	pinMode(A3, OUTPUT);
	digitalWrite(A3, LOW);

	lastRestAngle=targetAngle = balance_point;												//Set target angle to the balance point, this can be ajusted from the PC program 
	targetOffset = 0;															//Init to 0, this will be use as remote control offset
	

	kalmanY.setQangle(Qangle);
	kalmanY.setQbias(Qbias);
	kalmanY.setRmeasure(Rmeasure);


	low_bat = 0;
  /* add setup code here */
	loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time
	timer = micros();
	pidTimer = timer;
	bat_timer = millis() + 1000;
}

void loop()
{

	/*Do all the loop magic after this point*/
	read_radio();
	PID_calculation();
	readEncoders();	//Not in use
#ifdef TEST_PWM
	test_pwm();		//For testing the PWM and engien controller
#endif // TEST_PWM

						
						

	float bat_Val = readBattery();

	if (bat_Val < 1050.0 && bat_Val > 800.0) {                      //If batteryvoltage is below 10.5V and higher than 8.0V
		//error_handler(2);
#ifdef battery_debug
		Serial.print("Battery V=");
		Serial.println(battery_voltage);
#endif // battery_debug
		low_bat = 1;												//Set low bat flag for later use
	}

	
	
																					//More than +-35 degree will make the robot stop, then it has to be within +-5 degree to start again
	if (kalAngleY > 35 || kalAngleY < -35 || start == 0) {							//If the robot tips over or the start variable is zero or the battery is empty
		//pid_output = 0;															//Set the PID controller output to 0 so the motors stop moving
		//pid_i_mem = 0;															//Reset the I-controller memory
		start = 0;																	//Set the start variable to 0
		targetAngle = balance_point;												//Reset the self_balance_pid_setpoint variable
																					//TODO reset the kalman filtering
		stopMotor(left);
		stopMotor(right);															//Serial.print("Reset State ");
	}

																					//Restart the Robot when within the +-5 degree
	if ((kalAngleY > -5 && kalAngleY < 5) && start==0) {
		Serial.println("ok to go");
		start = 1;
	}
	
	test_timer = millis();
	
	if (test_timer - encoderTimer >= 100) { // Update encoder values every 100ms
		encoderTimer = test_timer;
		readEncoders();
		}

	if (low_bat == true) {
		/*TODO Turn on digital pin, what pin is free */
	}
	


	while (loop_timer > micros()) {		
																					//Stay in this loop untill next loop												
	};
	
	loop_timer += 4000;


}


/********************************************************************************************************/
/*Battery function with avage*/
float readBattery() {
	//Load the battery voltage to the battery_voltage variable.
	//85 is the voltage compensation for the diode.
	//Resistor voltage divider => (3.3k + 3.3k)/2.2k = 2.5
	//12.5V equals ~5V @ Analog 0.
	//12.5V equals 1023 analogRead(0).
	//1250 / 1023 = 1.222.
	//The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
	battery_voltage = (analogRead(battery) * 1.222) + 85;
	myRA.addValue(battery_voltage);

	if (bat_timer < millis()) {

		Serial.println(myRA.getAverage(), 3);
		bat_timer += 5000;
	}

	return myRA.getAverage();
}


/********************************************************************************************************/
/*Section for sensor calculation*/
/*Section will Have PID calculation, Sensor data and Kalman filtering*/
/*It will be called in the following main loop calls PID->sensor->kalman -> return roll and pich to main*/
void PID_calculation()
{
	read_gyro();

	//Do PID calculation, We need to find max output
	timer = micros();
	if (start == 1)
	{
		




		updatePID(targetAngle, targetOffset, turningOffset, (float)(timer - pidTimer) / 1000000.0f);
	}
	
	pidTimer = timer;

	static long write_time = 0;
	
	//TODO create a better structure
	di.AngleGyro = kalAngleY;
	di.PidSetpoint = targetOffset;
	di.SelfBalancePidSetpoint = targetAngle;
	di.id = 0x10;
	di.end = 0xff;

	//Only send data struct if requested from PC
	if (analyze_data == true) {
		/*Send debug info each half secound*/
		if ((millis() - write_time) > 50) {
			radio_write((char *)&di);
			write_time = millis();
		}
	}


#ifdef DEBUG_GYRO

#ifdef normal_serial_output
		Serial.println(kalAngleY);
	
#else
		Serial.println(kalAngleY, 2);
#endif // normal_serial_output
#endif // DEBUG_GYRO


}


/*end of PID section*/
/********************************************************************************************************/




void serial_write(char* data) {
	Serial.write((uint8_t*)data, sizeof(debug_info));
}



void readEncoders() {
	//Read encoders
	int32_t wheelPosition = leftEncode.read();
	wheelVelocity = wheelPosition - lastWheelPosition;
	lastWheelPosition = wheelPosition;
	//Serial.print(wheelPosition); Serial.print('\t'); Serial.print(targetPosition); Serial.print('\t'); Serial.print(stopped); Serial.print('\t'); Serial.println(wheelVelocity);
	if (abs(wheelVelocity) <= 20 && !stopped) { // Set new targetPosition if braking
		targetPosition = wheelPosition;
		Serial.println("test test test test");
		stopped = true;
	}
}



#ifdef TEST_PWM
void test_pwm(){
	while (1) {
		moveMotor(Command::left, Command::forward, 20.0);
		moveMotor(Command::right, Command::forward, 20.0);
	
		delay(2000);
		moveMotor(Command::left, Command::backward, 20.0);
		moveMotor(Command::right, Command::backward, 20.0);

		delay(2000);
		moveMotor(Command::left, Command::forward, 75.0);
		moveMotor(Command::right, Command::forward, 75.0);
		delay(2000);
		moveMotor(Command::left, Command::backward, 75.0);
		moveMotor(Command::right, Command::backward, 75.0);
		delay(2000);
		for (int a = 0; a < 100; a++)
		{
			moveMotor(Command::left, Command::backward, a);
			moveMotor(Command::right, Command::backward, a);
			delay(100);
		}
		delay(2000);
 }
}
#endif // TEST_PWM

void read_from_eeprom() {
	EEPROM_readAnything(0, configuration);
#ifdef DEBUG_EEPROM
	Serial.print("Configuration ");
	Serial.println(configuration.kP);
	Serial.println(configuration.kI);
	Serial.println(configuration.kD);
#endif // DEBUG_EEPROM	
	if ((configuration.kD < 200) || (configuration.kI < 200) || (configuration.kP<200)) {
		kP = configuration.kP;
		kI = configuration.kI;
		kD = configuration.kD;
		balance_point = configuration.bP;

	}
	else {
		/*Set default values for the PID*/
		Serial.println("Set default values for PID ");
		
		configuration.kP=kP = 8.9;
		configuration.kI = kI = 0.40;
		configuration.kD = kD = 0.13;
		configuration.bP = balance_point;

		//kP = 17, kI = 180, kD = 1.0;
	}
}

void save_to_eeprom() {
	EEPROM_writeAnything(0, configuration);
}

