#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include <math.h>
#include <r2d2/commands.h>
#include <r2d2/states.h>


void commandsCallback(const r2d2::commands msg)
{
	short rc[6];
	unsigned short mode = msg.flight_mode;
	for(int i=0; i<6; i++)
		rc[i] = msg.rc[i];
/*	printf("\nmode: [%3x]",mode);
	printf("\nrc:");
	printf("\n%d",rc[0]);
	printf("   %d",rc[1]);
	printf("   %d",rc[2]);
	printf("   %d",rc[3]);
	printf("   %d",rc[4]);
	printf("   %d",rc[5]);
	fflush(stdout);*/
}
void statesCallback(const r2d2::states msg)
{
	float pitch_R, roll_R, yaw_R;
	float pitch_Q, roll_Q, yaw_Q;
	float q0 = msg.q[0];
	float q1 = msg.q[1];
	float q2 = msg.q[2];
	float q3 = msg.q[3];
	pitch_R = -asin(msg.R[6]) * 57.3;
	roll_R  = atan2(msg.R[7], msg.R[8]) * 57.3;
	yaw_R = -atan2(msg.R[1], msg.R[4]) * 57.3;

	pitch_Q = asin(2*q0*q2-2*q1*q3 )*57.3;
	roll_Q  = atan2(2*q2*q3 + 2*q0*q1, 1-2*q1*q1-2*q2*q2)*57.3;
	yaw_Q   = -atan2(2*q1*q2 - 2*q0*q3, -2*q1*q1 - 2*q3*q3 + 1)*57.3;

	printf("\ntime stamp: [%d]",msg.time_stamp);
	printf("\nx: [%f]",msg.x_est[0]);
	printf("   y: [%f]",msg.y_est[0]);
	printf("   z: [%f]",msg.z_est[0]);
	printf("\nvx: [%f]",msg.x_est[1]);
	printf("   vy: [%f]",msg.y_est[1]);
	printf("   vz: [%f]",msg.z_est[1]);
	printf("\nq0: [%f]",msg.q[0]);
	printf("   q1: [%f]",msg.q[1]);
	printf("   q2: [%f]",msg.q[2]);
	printf("   q3: [%f]",msg.q[3]);
	printf("\npitch_R: [%f]",pitch_R);
	printf("   pitch_Q: [%f]",pitch_Q);
	printf("\nroll_R: [%f]",roll_R);
	printf("   roll_Q: [%f]",roll_Q);
	printf("\nyaw_R: [%f]",yaw_R);	
	printf("   yaw_Q: [%f]",yaw_Q);
	fflush(stdout);

}
int main(int argc,char **argv)
{
	ros::init(argc,argv,"listener");
	ros::NodeHandle n;
	ros::Subscriber states_sub = n.subscribe("states",5,statesCallback);
	ros::Subscriber commands_sub = n.subscribe("commands",5,commandsCallback);
	ros::spin();
	return 0;
}
