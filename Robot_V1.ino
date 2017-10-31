#include <SoftwareSerial.h>
#include <Encoder.h>
#include <DirectIO.h>
#include "robot.h"
#include "RunningAverage.h"
#include <Wire.h>
#include "writeToEeprom.h"
#include <EEPROM.h>

#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

#define loop_timing 10 /*main loop should be having a loop time of 10mS = 100 loop pr. sec.*/
#define start_radio_serial_speed 9600;
#define radio_serial_speed 19200;
//#define normal_serial_output;
#define battery_debug

/*Define all the pin*/
#define leftEncoderPin1 3
#define leftEncoderPin2 4
//#define rightEncoderPin1 7
//#define rightEncoderPin2 

/*Define the moter controller pin*/
#define ENA 9
#define ENB 10
Output<5> LeftA;
Output<6> LeftB;
Output<7> RightA;
Output<8> RightB;

#define radio_serial_rx 13
#define radio_serial_tx 12

#define radio_serial_cmd_pin 11
#define battery 0

/*End of pin configuration*/
#define DEBUG
//#define DEBUG_ENGIEN
//#define DEBUG_GYRO
//define TEST_PWM
//#define Radio_data



static const uint16_t PWM_FREQUENCY = 20000; // The motor driver can handle a PWM frequency up to 20kHz
static const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 1; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

/*Gyro const and VARS*/
static const int gyro_address = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)
static const int acc_calibration_value = 1000;                            //Enter the accelerometer calibration value
static float balance_point = 2.0;
long gyro_yaw_calibration_value, gyro_pitch_calibration_value;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double compAngleX, compAngleY; // Calculated angle using a complementary filter
//float self_balance_pid_setpoint;	//ajusted setpoint should be around 0
float pid_setpoint;					//Offset ajustment, used by remote to gain speed and by break to break by using gravity break
static uint32_t pidTimer; // Timer used for the PID loop
int16_t tempRaw;
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
RunningAverage myRA(50);

/*General VARS*/
unsigned long loop_timer;
unsigned long bat_timer;
int battery_voltage;
int receive_counter;
byte start, low_bat;
static bool steerStop = true; // Stop by default
static bool stopped; // This is used to set a new target position after braking
static float targetOffset = 0.0f; // Offset for going forward and backward
static float turningOffset = 0.0f; // Offset for turning left and right
static float targetAngle; // Resting angle of the robot
static float lastError; // Store last angle error
static float iTerm; // Store iTerm
float kP, kI, kD; // PID variables
static float engien_deadband = 14.0f;
static float engien_offsetL = 1.00f;
static float engien_offsetR = 0.90f;

bool analyze_data = false;

//input buffer for radio interface
unsigned char radio_read_buffer[64];
char radio_input;
float Qangle = 0.001f;
float Qbias = 0.003f;
float Rmeasure = 0.03f;

struct __attribute__((packed)) debug_info {
	byte id;
	double AngleGyro;
	float SelfBalancePidSetpoint;
	float PidSetpoint;
	byte end;
};

union UStuff
{
	float f;
	unsigned char c[0];
};
UStuff b;

struct config_t
{
	float kP;
	float kI;
	float kD;
} configuration;



int loop_time = loop_timing;
int current_time;

struct debug_info di;




//Class init
SoftwareSerial radio_serial(radio_serial_rx, radio_serial_tx);
Encoder leftEncode(leftEncoderPin1, leftEncoderPin2);		//Setup left the encoder for pin 1-2
/*TODO define right encoder pins*/
//Encoder rightEncoder(rightEncoderPin1, rightEncoderPin2);	//Setup right the encoder for pin 1-2
void setup()
{
 /*Setup Serial for debugging*/
	Wire.begin();                                                             //Start the I2C bus as master
	TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

	Serial.begin(19200);
	Serial.println("Starting up project Robot V1");
	
	myRA.clear();
	int speed = radio_serial_speed;
	radio_serial.begin(9600);
	read_from_eeprom();
	setup_radio("AT+BAUD=4", radio_serial_cmd_pin, speed);



	
	
	setup_PWM();
	setup_gyro();
	calibrate_gyro();

	targetAngle = balance_point;
	pid_setpoint = 0;
	
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
	Engien_control();	//Not in use
#ifdef TEST_PWM
	test_pwm();		//For testing the PWM and engien controller
#endif // TEST_PWM

						
						



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


	if (battery_voltage < 1050 && battery_voltage > 800) {                      //If batteryvoltage is below 10.5V and higher than 8.0V
		//error_handler(2);
#ifdef battery_debug
		Serial.print("Battery V=");
		Serial.println(battery_voltage);
#endif // battery_debug

		
		low_bat = 1;
	}

	

	if (kalAngleY > 30 || kalAngleY < -30 || start == 0) {    //If the robot tips over or the start variable is zero or the battery is empty
		//pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
		//pid_i_mem = 0;                                                          //Reset the I-controller memory
		start = 0;                                                              //Set the start variable to 0
		targetAngle = balance_point;                                          //Reset the self_balance_pid_setpoint variable
		stopMotor(left);
		stopMotor(right);																	  //Serial.print("Reset State ");
		//Serial.print(start);
		//Serial.print(" low Bat=");
	    //Serial.println(low_bat);
	}
	if ((kalAngleY > -5 && kalAngleY < 5) && start==0) {
		Serial.println("ok to go");
		start = 1;
	}
	


	if (low_bat == true) {
		/*TODO Turn on digital pin, what pin is free */

	}
	
	while (loop_timer > micros()) {
		
	};
	
	loop_timer += 4000;


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
	if (start==1)
	 updatePID(targetAngle, targetOffset, turningOffset, (float)(timer - pidTimer) / 1000000.0f);
	pidTimer = timer;

	static long write_time = 0;
	di.AngleGyro = kalAngleY;
	di.PidSetpoint = pid_setpoint;
	di.SelfBalancePidSetpoint = targetAngle;
	di.id = 0x10;
	di.end = 0xff;

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
	//Serial.println(sizeof(debug_info));
}



void Engien_control() {
//read encoder and calculate position and speed.
//This should be to ajust and offset, we should take into account of we have
//received a command.

//Truning should be direct on the engien, should be scaled down if the re

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
	Serial.print("Configuration ");
	Serial.println(configuration.kP);
	
	if ((configuration.kD < 200) || (configuration.kI < 200) || (configuration.kP<200)) {
		kP = configuration.kP;
		kI = configuration.kI;
		kD = configuration.kD;

	}
	else {
		/*Set default values for the PID*/
		Serial.println("Set default values for PID ");
		
		kP = 4.7;
		kI = 1.9;
		kD = 3.4;
		//kP = 17, kI = 180, kD = 1.0;
	}
}
void save_to_eeprom() {
	EEPROM_writeAnything(0, configuration);
}

