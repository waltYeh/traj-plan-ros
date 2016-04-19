#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
//#include <geometry_msgs/PoseStamped.h>
#include <r2d2/commands.h>
#include <r2d2/output.h>
#include <r2d2/states.h>
#include <r2d2/control_sp.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include "commons.h"
//#include <fcntl.h>
//#include <sys/signal.h>
//#include <sys/types.h>
//#include <termios.h>
#include <unistd.h>
#include <math.h>
#define USB_IN_INTERRUPT_PIN 16
#define USB_OUT_PIN 1
#define USB_OUT_FREQ 125
#define PROGRAM_FREQ 500
#define DSCRT_F 32768.0f
#define DSCRT_I 32768
struct _ctrl
{
	float pos_sp[3];
	float vel_sp[3];// a local given and used value
	float vel_ff[3];
	float acc_ff[3];

	float pitch_sp;
	float roll_sp;
	float yaw_sp;

	float throttle;
};
struct _ctrl ctrl = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},0,0,0,0};
unsigned char usb_in_flag = 0;
unsigned char rcv[64];
float q_sp[4];
float thrust_force;


void usb_in_interrupt(void)
{
	int status = digitalRead(USB_IN_INTERRUPT_PIN);
	if (status == 1) {//up edge
		usb_in_flag = 1;
	} else { //down edge

	}
}
void outputCallback(const r2d2::output msg)
{
	for(int i = 0; i < 4; i++)
		q_sp[i] = msg.q_sp[i];
	thrust_force = msg.thrust_force;
}
void control_spCallback(const r2d2::control_sp msg)
{
	for(int i = 0; i < 3; i++){
		ctrl.pos_sp[i] = msg.pos_sp[i];
		ctrl.vel_sp[i] = msg.vel_sp[i];
		ctrl.vel_ff[i] = msg.vel_ff[i];
		ctrl.acc_ff[i] = msg.acc_ff[i];
	}
	ctrl.pitch_sp = msg.pitch_sp;
	ctrl.roll_sp = msg.roll_sp;
	ctrl.yaw_sp = msg.yaw_sp;
	ctrl.throttle = msg.throttle;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "linker");
	ros::NodeHandle n;
	int aux_send = 0;
	n.getParam("/auxSend", aux_send);
	ros::Publisher states_pub = n.advertise<r2d2::states>("states",1000);
	ros::Publisher commands_pub = n.advertise<r2d2::commands>("commands",1000);
	ros::Subscriber output_sub = n.subscribe("output",1000,outputCallback);
	ros::Subscriber control_sp_sub = n.subscribe("control_sp",1000,control_spCallback);
	ros::Rate loop_rate(PROGRAM_FREQ);
	int count = 0;
	int port_desc;
	int freq_cnt = 0;
	printf("start\n");
	if (wiringPiSetup () == -1){
		fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
		return 1 ;
	}
	printf("wiringPi started\n");

	if ((port_desc = mySerialOpen ("/dev/ttyACM0", 115200)) >= 0){
		printf ("ACM0 opened");
  	}
	else if ((port_desc = mySerialOpen ("/dev/ttyACM1", 115200)) >= 0){
		printf ("ACM1 opened");
	}
	else if ((port_desc = mySerialOpen ("/dev/ttyACM2", 115200)) >= 0){
		printf ("ACM2 opened");
	}
	else{
		printf ("open failed");
		return 1 ;
	}


	pinMode(USB_OUT_PIN, OUTPUT);
	pinMode(USB_IN_INTERRUPT_PIN, INPUT);
	wiringPiISR (USB_IN_INTERRUPT_PIN, INT_EDGE_BOTH, &usb_in_interrupt) ;

	while (ros::ok()){
		unsigned short rcv_cnt = 0;
		freq_cnt ++;
	/*********************    USB OUT SEND    **********************/
		if (freq_cnt >= PROGRAM_FREQ / USB_OUT_FREQ){
			freq_cnt = 0;
		//	char out_buf[32];
			char out_buf[64];
			const unsigned char out_head[3] = {'<','#','<'};
			unsigned char check_length;
			unsigned short check_sum = 0;
			unsigned char descriptor = 'm';
			unsigned int time_stamp = 2048;
			switch(descriptor){
				case 'i':
				{
					memcpy(out_buf, out_head, 3);
					memcpy(out_buf+3, &descriptor, 1);
					memcpy(out_buf+4, &time_stamp, 4);
					break;
				}
				case 'm':
				{
					int _q0_sp = q_sp[0] * DSCRT_I;
					int _q1_sp = q_sp[1] * DSCRT_I;
					int _q2_sp = q_sp[2] * DSCRT_I;
					int _q3_sp = q_sp[3] * DSCRT_I;
					int _thrust_force = thrust_force;
					check_length = 29;
					memcpy(out_buf, out_head, 3);
					memcpy(out_buf+3, &descriptor, 1);
					memcpy(out_buf+4, &time_stamp, 4);
					memcpy(out_buf+8, &check_length, 1);
					memcpy(out_buf+9, &_q0_sp, 4);
					memcpy(out_buf+13, &_q1_sp, 4);
					memcpy(out_buf+17, &_q2_sp, 4);
					memcpy(out_buf+21, &_q3_sp, 4);
					memcpy(out_buf+25, &_thrust_force, 4);
					break;
				}
				default:
				break;
			}//end of switch descriptor
			for(int i=0; i<check_length; i++) {
				check_sum += out_buf[i];
			}
			memcpy(out_buf+29, &check_sum, 2);
			out_buf[31] = 0;
			switch(aux_send){
				case 0://none
				break;
				case 1://states
				break;
				case 2://commands
				break;
				case 3://control sp
				{	
					int pos_sp_i[3], vel_sp_i[3];
					for(int i=0;i<3;i++){
						pos_sp_i[i] = pos_sp[i];
						vel_sp_i[i] = vel_sp[i];
					}
					memcpy(out_buf+32, pos_sp_i, 12);
					memcpy(out_buf+44, vel_sp_i, 12);
					break;
				}
				case 4://control ff
				{
					int vel_ff_i[3], acc_ff_i[3];
					for(int i=0;i<3;i++){
						vel_ff_i[i] = vel_ff[i];
						acc_ff_i[i] = acc_ff[i];
					}
					memcpy(out_buf+32, vel_ff_i, 12);
					memcpy(out_buf+44, acc_ff_i, 12);
					break;
				}
				default:
				break;
			}//end of switch aux send
			digitalWrite(USB_OUT_PIN, HIGH);
			write(port_desc, out_buf, 64);
			
		}
		if (freq_cnt == 1) {
			digitalWrite(USB_OUT_PIN, LOW);
		}
	/*************************    USB IN GET    *************************/
		while(serialDataAvail(port_desc)){
			if(rcv_cnt<64){
				rcv[rcv_cnt] = serialGetchar(port_desc);
			}
			else{
				serialGetchar(port_desc);
				//don't store data if overflowing
			}
			rcv_cnt++;
		}
	/*************************    USB IN READ    *************************/
		if (usb_in_flag){
			usb_in_flag = 0;
			unsigned int time_stamp;
			unsigned char check_length;
			unsigned short check_sum = 0, rcv_check_sum;
			static unsigned char flight_mode = 1;
			static unsigned char last_flight_mode = 1;
			unsigned char in_head[3] = {'?','?','?'};
			unsigned char descriptor = '?';

			memcpy(in_head, rcv, 3);
			if(in_head[0] == '>' && in_head[1] == '*' && in_head[0] == '>' ) {
				memcpy(&descriptor, rcv + 3, 1);
			}
			if(descriptor != '?') {
				//read time, check length, and flight mode
				last_flight_mode = flight_mode;
				memcpy(&time_stamp, rcv + 4, 4);
				memcpy(&check_length, rcv + 8, 1);
				memcpy(&flight_mode, rcv + 9, 1);
				memcpy(&rcv_check_sum, rcv + 62, 2);
			}
			for(int i=0; i<check_length; i++){
				check_sum += rcv[i];
			}
			if(check_sum == rcv_check_sum){
				switch(descriptor){
					case 's'://states
					{
						r2d2::states states_msg;
						r2d2::commands commands_msg;
						int _x_est[2];
						int _y_est[2];
						int _z_est[2];
						short rc[6];
						int _q[4];
						float q[4];
						float yaw;
						memcpy(_x_est, rcv + 10, 8);
						memcpy(_y_est, rcv + 18, 8);
						memcpy(_z_est, rcv + 26, 8);
						memcpy(rc, rcv + 34, 12);
						memcpy(_q, rcv + 46, 16);
						for(int i=0;i<2;i++){
							states_msg.x_est[i] = _x_est[i] / 1000.0;
							states_msg.y_est[i] = _y_est[i] / 1000.0;
							states_msg.z_est[i] = _z_est[i] / 1000.0;
						}
						states_msg.x_acc = 0;
						states_msg.y_acc = 0;
						states_msg.z_acc = 0;
						for(int i=0;i<4;i++){
							q[i] = _q[i] / DSCRT_F;
						}
						yaw  = -atan2(2*q[1]*q[2] - 2*q[0]*q[3], -2*q[1]*q[1] - 2*q[3]*q[3] + 1);
						states_msg.yaw = yaw;
						states_msg.R[0]=2.0 * (0.5 - q[2] * q[2] - q[3] * q[3]);
						states_msg.R[1]=2.0 * (q[1] * q[2] - q[0] * q[3]);
						states_msg.R[2]=2.0 * (q[1] * q[3] + q[0] * q[2]);
						states_msg.R[3]=2.0 * (q[1] * q[2] + q[0] * q[3]);
						states_msg.R[4]=2.0 * (0.5 - q[1] * q[1] - q[3] * q[3]);
						states_msg.R[5]=2.0 * (q[2] * q[3] - q[0] * q[1]);
						states_msg.R[6]=2.0 * (q[1] * q[3] - q[0] * q[2]);
						states_msg.R[7]=2.0 * (q[2] * q[3] + q[0] * q[1]);
						states_msg.R[8]=2.0 * (0.5 - q[1] * q[1] - q[2] * q[2]);
						states_msg.q[0]=q[0];
						states_msg.q[1]=q[1];
						states_msg.q[2]=q[2];
						states_msg.q[3]=q[3];
						states_msg.time_stamp = time_stamp;

						commands_msg.time_stamp = time_stamp;
						for (int i = 0; i < 6; i++)
							commands_msg.rc[i] = rc[i];
						commands_msg.flight_mode = flight_mode;
						commands_msg.last_flight_mode = last_flight_mode;
						states_pub.publish(states_msg);
						commands_pub.publish(commands_msg);
				//		printf("\nStates published!\n%3x", count);
				//		fflush(stdout);
						break;
					}//end of case s
					case 2:
					break;
					default:
					break;
				}//end of switch
			}//end of if checksum ok
		}//end of if(usb_in_flag)
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}//end of while (ros::ok())
	return 0;
}
