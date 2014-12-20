#pragma config(Motor,  port1,           Drive_LF,      tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           Drive_LB,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           Drive_RF,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           Drive_RB,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           Lift_LF,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           Lift_LB,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           Lift_RF,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           Lift_RB,       tmotorVex393_MC29, openLoop, reversed)

//#pragma config(Motor,  port9,           <motor_name>,  tmotorVex393_MC29, openLoop)
//#pragma config(Motor,  port10,          <motor_name>,  tmotorVex393_HBridge, openLoop)

//#pragma config(Sensor, dgtl1, <sensor_name>, sensorQuaEncoder)
//#pragma config(Sensor, dgtl2, <sensor_name>, sensorQuadEncoder)
//#pragma config(Sensor, dgtl3, <sensor_name>, sensorQuadEncoder)
//#pragma config(Sensor, dgtl4, <sensor_name>, sensorQuadEncoder)

//+ "Xmtr2" to second controller channels

/*	MOTORS & SENSORS

	I/O Port			Name				Type				Desc

	Motor  - Port 1		Drive_LF			VEX 3-wire module	Front left drive motor
	Motor  - Port 2		Drive_LB			VEX 3-wire module	Rear left drive motor
	Motor  - Port 3		Drive_RF			VEX 3-wire module	Front right drive motor
	Motor  - Port 4		Drive_RB			VEX 3-wire module	Rear right drive motor
	Motor  - Port 5		Lift_LF				VEX 3-wire module	Front left lift motor
	Motor  - Port 6		Lift_LB				VEX 3-wire module	Rear left lift motor, geared in reverse
	Motor  - Port 7		Lift_RF				VEX 3-wire module	Front right lift motor
	Motor  - Port 8		Lift_RB				VEX 3-wire module	Rear left lift motor, geared in reverse
	Sensor - Port 1		leftDriveEncoder	VEX Shaft Encoder	Left-side drive encoder, on front motor
	Sensor - Port 2		rightDriveEncoder	VEX Shaft Encoder	Right-side drive encoder, on front motor
	Sensor - Port 3		leftLiftEncoder		VEX Shaft Encoder	Left-side lift encoder
	Sensor - Port 4		rightLiftEncoder	VEX Shaft Encoder	Right-side lift encoder
*/

#pragma platform(VEX)

#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

/*	VEX TEAM 4393S "All Girl Timedroids" 2014-2015 "Skyrise"
	Written by Ethan Eymontt

		Rev 	Date		Desc

	1	1.0		11.19.14	Initial version
	2	1.1		11.20.14	Modified motor power calculation function; added macros to define controller channels
	3	1.2		11.25.14	Kill switch enabled
	4	1.3		11.26.14	Assumed configuration pragmas; powered lift and drive mechanisms, prganized teleoperated task
	5	1.4		11.26.14	Added autonomous drive functions
*/

#define L_U_BTN Btn7U
#define L_D_BTN Btn7D
#define L_L_BTN Btn7L
#define L_R_BTN Btn7R

#define R_U_BTN Btn8U
#define R_D_BTN Btn8D
#define R_L_BTN Btn8L
#define R_R_BTN Btn8R

#define L1 Btn5U
#define L2 Btn5D
#define R1 Btn6U
#define R2 Btn6D

#define L_X_JOY Ch4
#define L_Y_JOY Ch3
#define R_X_JOY Ch1
#define R_Y_JOY Ch2

#define DEADBAND 10
#define abs(X) ((X < 0) ? -1 * X : X)

#include "Vex_Competition_Includes.c"

bool disabled = false;

//for autonomous task concurrency
int global degrees1
int global degrees2
int global degrees3
int global degrees4

int global power1
int global power2
int global power3
int global power4

int get_motor_power(int joyValue, int maxMotorSpeed){

	if(abs(joyValue) < DEADBAND){
		return 0;
	}

	float ratio = pow(joyValue, 2) / pow(127., 2);
	int motorSpeed = (ratio * maxMotorSpeed) * (joyValue / abs(joyValue));
	return motorSpeed;
}

void power_drive(int leftMotorSpeed, int rightMotorSpeed){

	motor[Drive_LF] = leftMotorSpeed;
	motor[Drive_LB] = leftMotorSpeed;
	motor[Drive_RF] = rightMotorSpeed;
	motor[Drive_RB] = rightMotorSpeed;
}

void power_lift(int motorSpeed){

	motor[Lift_LF] = motorSpeed;
	motor[Lift_LB] = motorSpeed;
	motor[Lift_RF] = motorSpeed;
	motor[Lift_RB] = motorSpeed;
}

void drive_forward(int time, int power){

	power_drive(power);
	wait1Msec(time);
	power_drive(0);
}

void drive_turn(int time, int power, bool clockise){

	if(clockise){
		motor[Drive_LF] = leftMotorSpeed;
		motor[Drive_LB] = leftMotorSpeed;
		motor[Drive_RF] = -rightMotorSpeed;
		motor[Drive_RB] = -rightMotorSpeed;

	}else{
		motor[Drive_LF] = -leftMotorSpeed;
		motor[Drive_LB] = -leftMotorSpeed;
		motor[Drive_RF] = rightMotorSpeed;
		motor[Drive_RB] = rightMotorSpeed;
	}
	wait1Msec(time);
	power_drive(0);
}

void raise_lift(int time, int power){

	power_lift(power);
	wait1Msec(time);
	power_lift(0);
}

task drive_left(){

	SensorValue[leftEncoder] = 0;
	degrees = degrees1;
	power = power1;

	while(abs(SensorValue[leftEncoder]) < abs(degrees)){
		motor[Drive_LF] = power;
		motor[Drive_LB] = power;
		//waitMsec(10)
	}
	motor[Drive_LF] = 0;
	motor[Drive_LB] = 0;

	AbortTimeSlice()
}

task drive_right(){

	SensorValue[leftEncoder] = 0;
	degrees = degrees2;
	power = power2;

	while(abs(SensorValue[leftEncoder]) < abs(degrees)){
		motor[Drive_RF] = power;
		motor[Drive_RB] = power;
		//waitMsec(10)
	}
	motor[Drive_RF] = 0;
	motor[Drive_RB] = 0;

	AbortTimeSlice()
}

task lift_left(){

	SensorValue[leftLiftEncoder] = 0;
	degrees = degrees3;
	power = power3;

	while(abs(SensorValue[leftLiftEncoder]) < abs(degrees)){
		motor[Lift_LF] = power;
		motor[Lift_LB] = power;
		//waitMsec(10)
	}
	motor[Lift_LF] = 0;
	motor[Lift_LB] = 0;

	AbortTimeSlice()
}

task lift_right(){

	SensorValue[rightLiftEncoder] = 0;
	degrees = degrees4;
	power = power4;

	while(abs(SensorValue[rightLiftEncoder]) < abs(degrees)){
		motor[Lift_RF] = power;
		motor[Lift_RB] = power;
		//waitMsec(10)
	}
	motor[Lift_RF] = 0;
	motor[Lift_RB] = 0;

	AbortTimeSlice()
}

void adjustDrive(int total_ticks, int power){

	int ticks = 0;
	int left_power = power;
	int right_power = power; 
	int diff = 0;
	
	SensorValue[leftDriveEncoder] = 0;
	SensorValue[rightDriveEncoder] = 0;
	
	while(abs(ticks) < total_ticks){
		
		power_drive(left_power, right_power);
		
		diff = SensorValue[leftDriveEncoder] - SensorValue[rightDriveEncoder];
		right_power += diff / 4;
		if(right_power > 100){
			right_power = 100;
		}

		SensorValue[leftDriveEncoder] = 0;
		SensorValue[rightDriveEncoder] = 0;

		waitMsec(10);

		ticks += SensorValue[leftDriveEncoder];
	}
	power_drive(0,0);
}

void encoder_drive_forward(int degrees, int power){

	degrees1 = degrees;
	power1 = power;

	degrees2 = degrees;
	power2 = power;

	StartTask(drive_left);
	StartTask(drive_right);
}

void encoder_drive_turn(int degrees, int power, bool clockise){

	degrees1 = degrees;
	degrees2 = degrees;

	if(clockise){

		power1 = power;
		power2 = -power;

		StartTask(drive_left);
		StartTask(drive_right);
	}else{

		power1 = -power;
		power2 = power;

		StartTask(drive_left);
		StartTask(drive_right);
	}
}

void encoder_raise_lift(int degrees, int power){

	degrees3 = degrees;
	power3 = power;

	degrees4 = degrees;
	power4 = power;

	StartTask(lift_left);
	StartTask(lift_right);
}

void pre_auton(){

	motor[Drive_LF] = 0;
	motor[Drive_LB] = 0;
	motor[Drive_RF] = 0;
	motor[Drive_RB] = 0;
	motor[Lift_LF]  = 0;
	motor[Lift_LB]  = 0;
	motor[Lift_RF]  = 0;
	motor[Lift_RB]  = 0;
	//SensorValue[leftEncoder]      = 0;
	//SensorValue[rightEncoder]     = 0;
	//SensorValue[leftLiftEncoder]  = 0;
	//SensorValue[rightLiftEncoder] = 0;
}

task autonomous(){

	drive_forward(1000, 50);
}

task usercontrol(){

	while(true){
		if(disabled != false){ //while functioning correctly

			power_drive(get_motor_power(vexRT[L_Y_JOY], 127), get_motor_power(vexRT[R_Y_JOY], 127));

			if(vexRT[R_U_BTN]){
				power_lift(50);
			}
			else if(vexRT[R_D_BTN]){
				power_lift(-50);
			}
			else power_lift(0);
		}

		if(vexRT[L1] && vexRT[R1]){disabled = true;}	//disable robot
		if(vexRT[L2] && vexRT[R2]){disabled = false;} //enable robot
	}
}