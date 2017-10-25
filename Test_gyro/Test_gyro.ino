#include <Wire.h>  
byte lowByte, highByte, type, error, clockspeed_ok;
unsigned long timer, loop_timer;
int address;

struct __attribute__((packed)) debug_info {
	byte id;
	double AngleGyro;
	float SelfBalancePidSetpoint;
	float PidSetpoint;
	byte end;
};

//****************************************************************************************************************
//GYROS (6050 IMU) vars
int cal_int, start, gyro_address;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
int gyro_x, gyro_y, gyro_z;
int temperature;
int acc_axis[4], gyro_axis[4];
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
struct debug_info di;



byte  gyro_check_byte;
byte roll_axis, pitch_axis, yaw_axis;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  //Serial.println("Test of Gyro");
  //Serial.println("This test will ensure the the gyro has been mounted correct and that it works");
  pinMode(A3,OUTPUT);
  
  Wire.begin();             //Start the I2C as master
  error=0;
  delay(250);               //Give the gyro time to start

  systemSetup();



	  //What gyro is connected
#ifdef output_enabled
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("Gyro search"));
  Serial.println(F("==================================================="));
#endif // output_enabled


	  delay(2000);
#ifdef output_enabled
	  Serial.println(F("Searching for MPU-6050 on address 0x68/104"));
#endif

	  delay(1000);
	  if (search_gyro(0x68, 0x75) == 0x68) {
#ifdef output_enabled
		  Serial.println(F("MPU-6050 found on address 0x68"));
#endif // output_enabled

		  
		  type = 1;
		  gyro_address = 0x68;
	  }
	  else {
		  error = 1;
		  Serial.println("Did not find the MPU6050), please check connections");
		  while (1);
	  }



	 
		  delay(3000);
#ifdef output_enabled
		  Serial.println(F(""));
		  Serial.println(F("==================================================="));
		  Serial.println(F("Gyro register settings"));
		  Serial.println(F("==================================================="));
#endif // output_enabled


		  start_gyro(); //Setup the gyro for further use

		  digitalWrite(A3, HIGH);
		  delay(3000);
#ifdef output_enabled
		  Serial.println(F(""));
		  Serial.println(F("==================================================="));
		  Serial.println(F("Gyro calibration"));
		  Serial.println(F("==================================================="));
		  Serial.println(F("Don't move the quadcopter!! Calibration starts in 3 seconds"));
#endif // output_enabled


		  delay(1000);
		 
		  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
		  calibrating_gyro();

		  delay(500);
	 

  
}


//Search for the gyro and check the Who_am_I register
byte search_gyro(int gyro_address, int who_am_i){
  Wire.beginTransmission(gyro_address);
  Wire.write(who_am_i);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 1);
  timer = millis() + 100;
  while(Wire.available() < 1 && timer > millis());
  lowByte = Wire.read();
  address = gyro_address;
  return lowByte;
}


void start_gyro(){
 
  //Setup the MPU-6050
  if(type == 1){
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //PWR_MGMT_1 register
    Wire.write(0x00);                                            //Set to zero to turn on the gyro
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x6B is set to:"));
    Serial.println(Wire.read(),BIN);
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x1B);                                            //GYRO_CONFIG register
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the gyro (adress 1101001)
    Wire.write(0x1B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x1B is set to:"));
    Serial.println(Wire.read(),BIN);

  }
  else
  {
    Serial.print("Wrong Gyro should be type 1, got type=");
    Serial.println(type);
  }
}

void read_mpu_6050_data() {                                             //Subroutine for reading the raw gyro and accelerometer data
	Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
	Wire.write(0x3B);                                                    //Send the requested starting register
	Wire.endTransmission();                                              //End the transmission
	Wire.requestFrom(0x68, 14);                                           //Request 14 bytes from the MPU-6050
	while (Wire.available() < 14);                                        //Wait until all the bytes are received
	acc_x = Wire.read() << 8 | Wire.read();                                  //Add the low and high byte to the acc_x variable
	acc_y = Wire.read() << 8 | Wire.read();                                  //Add the low and high byte to the acc_y variable
	acc_z = Wire.read() << 8 | Wire.read();                                  //Add the low and high byte to the acc_z variable
	temperature = Wire.read() << 8 | Wire.read();                            //Add the low and high byte to the temperature variable
	gyro_x = Wire.read() << 8 | Wire.read();                                 //Add the low and high byte to the gyro_x variable
	gyro_y = Wire.read() << 8 | Wire.read();                                 //Add the low and high byte to the gyro_y variable
	gyro_z = Wire.read() << 8 | Wire.read();                                 //Add the low and high byte to the gyro_z variable

}




void systemSetup(){


  delay(1000);
  
  TWBR = 12;                      //Set the I2C clock speed to 400kHz.
  
  #if F_CPU == 16000000L          //If the clock speed is 16MHz include the next code line when compiling
    clockspeed_ok = 1;            //Set clockspeed_ok to 1
  #endif                          //End of if statement

  if(TWBR == 12 && clockspeed_ok){

    
  }
  else{
    Serial.println(F("I2C clock speed is not set to 400kHz. (ERROR 8)"));
    error = 1;
  }
}


//****************************************************************************//
void loop() {
  // put your main code here, to run repeatedly:
	read_mpu_6050_data();
	get_gyro_calculation();
	gyro_angle_calculations();
	 
	
	static long write_time = 0;
	di.AngleGyro = angle_roll;
	di.PidSetpoint = 0;
	di.SelfBalancePidSetpoint = 0;
	di.id = 0x10;
	di.end = 0xff;


	/*Send debug info each half secound*/
	if ((millis() - write_time) > 10) {
	//	Serial.write((char *)&di);
		Serial.println(angle_roll);
	}


	while (micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
	loop_timer = micros();

}


void get_gyro_calculation() {
	gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
	gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
	gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

	gyro_roll = gyro_y;                                                    //Set the gyro_roll to the calibrated value
	gyro_pitch = gyro_x;                                                   //Set the gyro_pitch to the calibrated value
	gyro_yaw = gyro_z;                                                     //Set the gyro_yaw to the calibrated value





}

void gyro_angle_calculations() {

	angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
	angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

																			  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
	angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
	angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

																			  //Accelerometer angle calculations
	acc_total_vector = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));       //Calculate the total accelerometer vector.

	if (abs(acc_y) < acc_total_vector) {                                        //Prevent the asin function to produce a NaN
		angle_pitch_acc = asin((float)acc_y / acc_total_vector)* 57.296;          //Calculate the pitch angle.
	}
	if (abs(acc_x) < acc_total_vector) {                                        //Prevent the asin function to produce a NaN
		angle_roll_acc = asin((float)acc_x / acc_total_vector)* -57.296;          //Calculate the roll angle.
	}

	//Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
	angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
	angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.

	angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
	angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.


	

}

void calibrating_gyro() {

	Serial.println(" Calibrating gyro");

	for (int cal_int = 0; cal_int < 500; cal_int++) {                  //Run this code 2000 times
		if (cal_int % 125 == 0)
		{

			//Serial.print(".");                              //Print a dot on the LCD every 125 readings
 


		}
		read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
		gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
		gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
		gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
		delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
	}
	gyro_x_cal /= 500;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
	gyro_y_cal /= 500;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
	gyro_z_cal /= 500;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

	

}

