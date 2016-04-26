#include "ros/ros.h"
#include <math.h>
#include <Eigen/Eigen/Eigen>
#include <Eigen/Eigen/Geometry>

//#include <geometry_msgs/PoseStamped.h>
#include <r2d2/commands.h>
#include <r2d2/output.h>
#include <r2d2/states.h>
#include <r2d2/control_sp.h>

#include <stdio.h>
#include <string.h>
#include "commons.h"
using namespace Eigen;
#define MAX_ALT_VEL 2000
#define MAX_XY_VEL 8000
#define VEL_FF_Z_P 0.5
#define VEL_FF_XY_P 0.6
#define ACC_FF_P 0.0

unsigned short flight_mode;
unsigned short last_flight_mode;
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
	unsigned int timestamp;
};
struct _out
{
	float q_sp[4];
	float thrust_force;
};
struct _est
{
	float pos[3];
	float vel[3];
	float body_z[3];
	float yaw;
};
typedef struct _PID {
	int Err;
	int RateErr;
	int l_RateErr;
	int int_RateErr;
	float P;		
	float Prate;
	float Irate;
	float Drate;
}PID;
PID altPID = {0,0,0,0,
		2.2,
		2.1, 0.02, 0.2};
PID pos_xPID = {0,0,0,0,
		0.3,
		2.5, 0.03, 0.1};
PID pos_yPID = {0,0,0,0,
		0.3,
		2.5, 0.03, 0.1};
struct _ctrl ctrl = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},0,0,0,0,0};
struct _out out = {{0,0,0,0},0};
struct _est est = {{0,0,0},{0,0,0},{0,0,1.0f},0};
float R_sp[3][3];
bool USB_connected = false;
void commandsCallback(const r2d2::commands msg)
{
	flight_mode = msg.flight_mode;
	last_flight_mode = msg.last_flight_mode;
	USB_connected = true;
}
void statesCallback(const r2d2::states msg)
{
	est.pos[0] = msg.x_est[0];
	est.pos[1] = msg.y_est[0];
	est.pos[2] = msg.z_est[0];
	est.vel[0] = msg.x_est[1];
	est.vel[1] = msg.y_est[1];
	est.vel[2] = msg.z_est[1];
	est.yaw = msg.yaw;
	for(int i=0; i<3; i++){
		est.body_z[i] = msg.R[i * 3 + 2];
	}
}
void control_spCallback(const r2d2::control_sp msg)
{
	for(int i = 0; i < 3; i++){
		ctrl.pos_sp[i] = msg.pos_sp[i];
//		ctrl.vel_sp[i] = msg.vel_sp[i];
		ctrl.vel_ff[i] = msg.vel_ff[i];
		ctrl.acc_ff[i] = msg.acc_ff[i];
	}
	ctrl.pitch_sp = msg.pitch_sp;
	ctrl.roll_sp = msg.roll_sp;
	ctrl.yaw_sp = msg.yaw_sp;
	ctrl.throttle = msg.throttle;
	ctrl.timestamp = msg.time_stamp;
}

float external_pid(PID *pid, float err, short dt)
{
 	 float retValue;								
 	 pid->Err = err;
 	 retValue = pid->P * err;
	 return retValue;
}
float internal_pid(PID *pid, float rate_err, short dt)
{
   	float rateOuput;								
  	pid->RateErr = rate_err;
  	rateOuput = pid->Prate * rate_err 
  		+ pid->Drate * (rate_err - pid->l_RateErr) *1000/ dt 
  		+ pid->Irate * pid->int_RateErr / 1000;
  	pid->l_RateErr = rate_err;
	pid->int_RateErr += rate_err * dt;
  	return rateOuput;
}

void manual_R_sp_generate(void)
{
	float cp = cos(ctrl.pitch_sp);
	float sp = sin(ctrl.pitch_sp);
	float sr = sin(ctrl.roll_sp);
	float cr = cos(ctrl.roll_sp);
	float sy = sin(ctrl.yaw_sp);
	float cy = cos(ctrl.yaw_sp);
	R_sp[0][0] = cp * cy;
	R_sp[0][1] = ((sr * sp * cy) - (cr * sy));
	R_sp[0][2] = ((cr * sp * cy) + (sr * sy));
	R_sp[1][0] = cp * sy;
	R_sp[1][1] = ((sr * sp * sy) + (cr * cy));
	R_sp[1][2] = ((cr * sp * sy) - (sr * cy));
	R_sp[2][0] = -sp;
	R_sp[2][1] = sr * cp;
	R_sp[2][2] = cr * cp;
}
void manual_thrust_generate(void)
{
	out.thrust_force = ctrl.throttle * GRAVITY/1000 * VEHICLE_MASS / 512;
}
void altitude_control(short dt)
{
	ctrl.vel_sp[2] = constrain_f(external_pid(&altPID, ctrl.pos_sp[2] - est.pos[2], dt), -MAX_ALT_VEL, MAX_ALT_VEL);
	ctrl.vel_sp[2] +=  VEL_FF_Z_P * ctrl.vel_ff[2];
	float altAcc_sp = GRAVITY + internal_pid(&altPID, ctrl.vel_sp[2] - est.vel[2], dt);
	out.thrust_force = (altAcc_sp / est.body_z[2]) * VEHICLE_MASS / 1000;
}

void position_control(short dt)
{
	Vector3d acc_sp, z_b, y_c;
	Vector3d body_x_sp, body_y_sp, body_z_sp;
	float norm_temp;
	ctrl.vel_sp[0] = constrain_f(external_pid(&pos_xPID, ctrl.pos_sp[0] - est.pos[0], dt), -MAX_XY_VEL, MAX_XY_VEL);
	ctrl.vel_sp[1] = constrain_f(external_pid(&pos_yPID, ctrl.pos_sp[1] - est.pos[1], dt), -MAX_XY_VEL, MAX_XY_VEL);
	ctrl.vel_sp[2] = constrain_f(external_pid(&altPID, ctrl.pos_sp[2] - est.pos[2], dt), -MAX_ALT_VEL, MAX_ALT_VEL);
	ctrl.vel_sp[0] += VEL_FF_XY_P * ctrl.vel_ff[0];
	ctrl.vel_sp[1] += VEL_FF_XY_P * ctrl.vel_ff[1];
	ctrl.vel_sp[2] += VEL_FF_Z_P * ctrl.vel_ff[2];
	acc_sp(0) = internal_pid(&pos_xPID, ctrl.vel_sp[0] - est.vel[0], dt);
	acc_sp(1) = internal_pid(&pos_yPID, ctrl.vel_sp[1] - est.vel[1], dt);
	acc_sp(2) = GRAVITY + internal_pid(&altPID, ctrl.vel_sp[2] - est.vel[2], dt);
	acc_sp(0) += ACC_FF_P * ctrl.acc_ff[0];
	acc_sp(1) += ACC_FF_P * ctrl.acc_ff[1];
	acc_sp(2) += ACC_FF_P * ctrl.acc_ff[2];
	for(int i=0; i<3; i++)
		z_b(i) = est.body_z[i];
	out.thrust_force = acc_sp.dot(z_b) * VEHICLE_MASS / 1000;
	norm_temp = acc_sp.norm();
	body_z_sp = acc_sp / norm_temp;
	y_c(0) = -sin(ctrl.yaw_sp);
	y_c(1) = cos(ctrl.yaw_sp);
	y_c(2) = 0;
	body_x_sp = y_c.cross(body_z_sp);
	norm_temp = body_x_sp.norm();
	body_x_sp = body_x_sp / norm_temp;
	body_y_sp = body_z_sp.cross(body_x_sp);
	for (int i = 0; i < 3; i++) {
		R_sp[i][0] = body_x_sp(i);
		R_sp[i][1] = body_y_sp(i);
		R_sp[i][2] = body_z_sp(i);
	}
	ctrl.pitch_sp = -asin(R_sp[2][0]);
	ctrl.roll_sp = atan2(R_sp[2][1], R_sp[2][2]);
}
void reset_variables(void)
{
	altPID.int_RateErr = 0;
	pos_xPID.int_RateErr = 0;
	pos_yPID.int_RateErr = 0;
}
void Rsp_to_Qsp(void)
{
	float q_sp[4];
	float tr = R_sp[0][0] + R_sp[1][1] + R_sp[2][2];
	if (tr > 0.0f) {
		float s = sqrt(tr + 1.0f);
		q_sp[0] = s * 0.5f;
		s = 0.5f / s;
		q_sp[1] = (R_sp[2][1] - R_sp[1][2]) * s;
		q_sp[2] = (R_sp[0][2] - R_sp[2][0]) * s;
		q_sp[3] = (R_sp[1][0] - R_sp[0][1]) * s;
	} else {
		/* Find maximum diagonal element in dcm
		* store index in dcm_i */
		int dcm_i = 0;
		for (int i = 1; i < 3; i++) {
			if (R_sp[i][i] > R_sp[dcm_i][dcm_i]) {
				dcm_i = i;
			}
		}
		int dcm_j = (dcm_i + 1) % 3;
		int dcm_k = (dcm_i + 2) % 3;
		float s = sqrtf((R_sp[dcm_i][dcm_i] - R_sp[dcm_j][dcm_j] -
		R_sp[dcm_k][dcm_k]) + 1.0f);
		q_sp[dcm_i + 1] = s * 0.5f;
		s = 0.5f / s;
		q_sp[dcm_j + 1] = (R_sp[dcm_i][dcm_j] + R_sp[dcm_j][dcm_i]) * s;
		q_sp[dcm_k + 1] = (R_sp[dcm_k][dcm_i] + R_sp[dcm_i][dcm_k]) * s;
		q_sp[0] = (R_sp[dcm_k][dcm_j] - R_sp[dcm_j][dcm_k]) * s;
	}
	for(int i = 0; i < 4; i++)
		out.q_sp[i] = q_sp[i];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;
	ros::Publisher output_pub = n.advertise<r2d2::output>("output",5);
	ros::Subscriber commands_sub = n.subscribe("commands",5,commandsCallback);
	ros::Subscriber control_sp_sub = n.subscribe("control_sp",5,control_spCallback);
	ros::Subscriber states_sub = n.subscribe("states",5,statesCallback);
	ros::Rate loop_rate(125);
	while(!USB_connected && ros::ok()){//waiting for connection with autopilot
		ros::spinOnce();
		loop_rate.sleep();
	}
	int count = 0;
	reset_variables();
	while (ros::ok()){
		r2d2::output output_msg;
		if(flight_mode == MANUEL || flight_mode == RASP_MANUEL){
			manual_R_sp_generate();
			manual_thrust_generate();
		}
		else if(flight_mode == ALT_CTRL || flight_mode == RASP_ALT){
			manual_R_sp_generate();
			altitude_control(8);
		}
		else if(flight_mode == POS_CTRL || flight_mode == RASP_POS 
			||flight_mode == RASP_NURBS_SEMI || flight_mode == RASP_NURBS_AUTO){
			position_control(8);
		}
		Rsp_to_Qsp();
		for(int i = 0; i < 4; i++)
			output_msg.q_sp[i] = out.q_sp[i];
		output_msg.thrust_force = out.thrust_force;
		output_msg.time_stamp = ctrl.timestamp;
		output_pub.publish(output_msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
