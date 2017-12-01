#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

#define loop_timing 10 /*main loop should be having a loop time of 10mS = 100 loop pr. sec.*/
#define start_radio_serial_speed 9600;
#define radio_serial_speed 19200;
//#define normal_serial_output;


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
//#define DEBUG
//#define battery_debug
//#define DEBUG_ENGIEN
//#define DEBUG_GYRO
//define TEST_PWM
//#define Radio_data





enum Command {
	pid,
	save_config,
	send_data,
	ajust_balance_point,
	stop,
	forward,
	backward,
	left,
	right,
	imu,
	joystick,
} lastCommand; // This is used set a new targetPosition

//Struct for sending data to the PC program
struct __attribute__((packed)) debug_info {
	byte id;
	double AngleGyro;
	float SelfBalancePidSetpoint;
	float PidSetpoint;
	byte end;
};

/*Packed data struct, this is used for conversion from byte array to a data struct
this is very compiler dependent and can't be move to other board with good testing
We always should receive 14 bytes no less no more*/





struct __attribute__((packed)) _msgtype1 {
	byte id;
	float data1;
	float data2;
	float data3;
	byte end;
}; static _msgtype1 *dataStruct;


struct __attribute__((packed)) _msgtype2 {
	byte id;
	unsigned long data1;
	long data2;
	long data3;
	byte end;
}; static _msgtype2 *dataStruct1;

struct __attribute__((packed)) _msgtype3 {
	byte id;
	byte data1;
	byte data2;
	byte data3;
	byte data4;
	byte data5;
	byte data6;
	byte data7;
	byte data8;
	byte data9;
	byte data10;
	byte data11;
	byte data12;
	byte end;
}; static _msgtype3 *dataStruct2;

struct __attribute__((packed)) _msgtype4 {
	byte id;
	unsigned long data1;
	long data2;
	float data3;
	byte end;
}; static _msgtype4 *dataStruct3;



//Union used convert from byte to float, and the other way around
union UStuff
{
	float f;
	unsigned char c[0];
};
UStuff b;

//Struct for storing to EEprom
struct config_t
{
	float kP;
	float kI;
	float kD;
	float bP;
}; config_t configuration;
