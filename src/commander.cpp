#include "ros/ros.h"
#include <math.h>
//#include <Eigen/Eigen/Eigen>
//#include <Eigen/Eigen/LU>
//#include <geometry_msgs/PoseStamped.h>
#include <r2d2/commands.h>
#include <r2d2/output.h>
#include <r2d2/states.h>
#include <r2d2/control_sp.h>

#include <stdio.h>
#include <string.h>
#include "nurbs.h"
#include "tangential_smooth.h"
#include "commons.h"

#define LOOP_RATE 125
#define LOOP_PERIOD 8
//commander modes
#define ON_BOARD_MODES 0
#define AIDING_MODES 1
#define WAITING_MODES 2
#define NURBS_SEMI 3
#define NURBS_AUTO 4
struct _nurbs
{
	bool plan_ok;
	float v_plan;
	float tan_acc;
};

//set at 
//cleared at any mode changes

struct _ctrl
{
	float pos_sp[3];
	float vel_sp[3];//no use
	float vel_ff[3];
	float acc_ff[3];

	float pitch_sp;
	float roll_sp;
	float yaw_sp;

	float throttle;
};
struct _est//only changeable in statesCallback
{
	float pos[3];
	float vel[3];
	float yaw;
	unsigned int timestamp;
};
struct _cmd//only changeable in commandsCallback
{
	unsigned short commander_mode;//can be in waiting mode
	unsigned short flight_mode;//corresponds to switches
	unsigned short last_flight_mode;
	short rc[6];
	unsigned int timestamp;
};


struct _ctrl ctrl = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},0,0,0,0};
struct _est est = {{0,0,0},{0,0,0},0,0};
struct _cmd cmd = {0,1,1,{0,0,-1024,0,0,0},0};
struct _nurbs nbs;
bool USB_connected = false;
void planOkCallback(void)
{
	nbs.plan_ok = true;
	//TODO store the NURBS Params
}
void commandsCallback(const r2d2::commands msg)
{
	cmd.timestamp = msg.time_stamp;
	for(int i = 0; i < 6; i++)
		cmd.rc[i] = msg.rc[i];
	cmd.flight_mode = msg.flight_mode;
	cmd.last_flight_mode = msg.last_flight_mode;
	if(cmd.flight_mode < 5)
		cmd.commander_mode = ON_BOARD_MODES;
	else if(cmd.flight_mode < 8)
		cmd.commander_mode = AIDING_MODES;
	else if(cmd.flight_mode == RASP_NURBS_SEMI){
		if(nbs.plan_ok)
			cmd.commander_mode = NURBS_SEMI;
		else
			cmd.commander_mode = NURBS_SEMI;
	}
	else if(cmd.flight_mode == RASP_NURBS_AUTO){
		if(nbs.plan_ok)
			cmd.commander_mode = NURBS_AUTO;
		else
			cmd.commander_mode = NURBS_AUTO;
	}
//	USB_connected = true;
}
void statesCallback(const r2d2::states msg)
{
	est.timestamp = msg.time_stamp;
	est.yaw = msg.yaw;
	est.pos[0] = msg.x_est[0];
	est.pos[1] = msg.y_est[0];
	est.pos[2] = msg.z_est[0];
	est.vel[0] = msg.x_est[1];
	est.vel[1] = msg.y_est[1];
	est.vel[2] = msg.z_est[1];
	USB_connected = true;
}
void reset_yaw_sp(void)
{
	ctrl.yaw_sp = est.yaw;
}
//TODO set point ahead of current point considering vel
void reset_position_sp(void)
{
	for(int i=0; i<3; i++){
		ctrl.pos_sp[i] = est.pos[i];
		ctrl.vel_ff[i] = 0;
		ctrl.acc_ff[i] = 0;
	}
}
//arguments: period
//global: ctrl.(all), nbs.vel_plan
void rc2sp(short dt)
{
	#define MAX_ATT_MANUEL 0.70f //40 deg
	#define MAX_ALT_VEL_MANUEL 500.0f//manuel
	#define MAX_XY_VEL_MANUEL 1500.0f
	#define MAX_YAW_RATE_MANEUL 0.87f
	#define THR_CMD_RATIO 1.0f
	float expctBodyVel[2];
	float expctGlobVel[2];
	float expctAltRate;
	
	if(cmd.flight_mode == RASP_MANUEL || cmd.flight_mode == RASP_ALT
		|| cmd.flight_mode == MANUEL || cmd.flight_mode == ALT_CTRL){
		ctrl.pitch_sp = dead_zone_f(cmd.rc[1] * MAX_ATT_MANUEL / 1024.0f, 0);
		ctrl.roll_sp = dead_zone_f(-cmd.rc[0] * MAX_ATT_MANUEL / 1024.0f, 0);
	}
	//xy pos command for posctrl
	else if(cmd.flight_mode == RASP_POS || cmd.flight_mode == POS_CTRL){
		expctBodyVel[0] = dead_zone_f(cmd.rc[1] * MAX_XY_VEL_MANUEL / 1024.0f, 400);
		expctBodyVel[1] = dead_zone_f(cmd.rc[0] * MAX_XY_VEL_MANUEL / 1024.0f, 400);
		body2glob_2D(expctBodyVel, expctGlobVel, est.yaw);
		ctrl.pos_sp[0] += expctGlobVel[0] * dt / 1000;	
		ctrl.pos_sp[1] += expctGlobVel[1] * dt / 1000;
		ctrl.vel_ff[0] = expctGlobVel[0];
		ctrl.vel_ff[1] = expctGlobVel[1];
		nbs.v_plan = expctGlobVel[0];
	}

	//yaw command for all
	float expctYawRate = dead_zone_f(cmd.rc[3] * MAX_YAW_RATE_MANEUL / 1024.0f, 0.26f);//15 deg
	ctrl.yaw_sp += expctYawRate * dt / 1000;

	//throtle command for manuel, acrob
	if(cmd.flight_mode == RASP_MANUEL || cmd.flight_mode == MANUEL ){
		ctrl.throttle = dead_zone_f(constrain((cmd.rc[2] + 1024)/2 * THR_CMD_RATIO, 0, 1024), 50);
	}
	//alt command for altctrl, posctrl
	else if(cmd.flight_mode == RASP_ALT||cmd.flight_mode == RASP_POS
		|| cmd.flight_mode == ALT_CTRL||cmd.flight_mode == POS_CTRL){
		expctAltRate = dead_zone_f(cmd.rc[2] * MAX_ALT_VEL_MANUEL / 1024.0f, 100);
		ctrl.pos_sp[2] += expctAltRate * dt / 1000;
		ctrl.vel_ff[2] = expctAltRate;
	}

	//all the unused sp need to be dealt with
	if(cmd.flight_mode == RASP_MANUEL || cmd.flight_mode == MANUEL){
		for(int i=0;i<3;i++){
			ctrl.pos_sp[i] = est.pos[i];
			ctrl.vel_sp[i] = 0;
			ctrl.vel_ff[i] = 0;
			ctrl.acc_ff[i] = 0;
			nbs.v_plan = 0;
		}
	}
	else if(cmd.flight_mode == RASP_ALT || cmd.flight_mode == ALT_CTRL){
		ctrl.pos_sp[0] = est.pos[0];
		ctrl.pos_sp[1] = est.pos[1];
		ctrl.vel_ff[0] = 0;
		ctrl.vel_ff[1] = 0;
		for(int i=0;i<3;i++){
			ctrl.vel_sp[i] = 0;
			ctrl.acc_ff[i] = 0;
		}
		nbs.v_plan = 0;
	}
	else if(cmd.flight_mode == RASP_POS || cmd.flight_mode == POS_CTRL){
		for(int i=0;i<3;i++){
			ctrl.vel_sp[i] = 0;
			ctrl.acc_ff[i] = 0;
		}
		nbs.v_plan = 0;
	}
}
void rc_v_plan(void)
{
	nbs.v_plan = dead_zone_f(cmd.rc[1] * MAX_XY_VEL_MANUEL / 1024.0f, 400);
}

void rc_yaw(short dt)
{
	float expctYawRate = dead_zone_f(cmd.rc[3] * MAX_YAW_RATE_MANEUL / 1024.0f, 0.26f);//15 deg
	ctrl.yaw_sp += expctYawRate * dt / 1000;
}
/**************
"commander" asks "planner" to solve nurbs when "commander" detect the mode change.
"planner" solves nurbs using library nurbs.cpp, 
during which "commander" continues to give posctrl commands.
When solved, the important params of nurbs Q, P, Knots are stored in "planner" and sent to "commander" also.
Current u is owned and set by "commander".
At semi-auto mode, "commander" gets V from rc, at auto mode, "commander" gets V from "planner".
As interpolation, the u is updated in nurbs.cpp and given back to "commander".
Then "commander" publish the sp and ff to let "controller" do the controls
****************/
int main(int argc, char **argv)
{
	nurbs nurbs1;
	trpz trpz1;
	float t_plan = 0.0;
//	jopt jopt1;
	ros::init(argc, argv, "commander");
	ros::NodeHandle n;
	ros::Publisher control_sp_pub = n.advertise<r2d2::control_sp>("control_sp",5);
	ros::Subscriber commands_sub = n.subscribe("commands",5,commandsCallback);
	ros::Subscriber states_sub = n.subscribe("states",5,statesCallback);
	ros::Rate loop_rate(LOOP_RATE);
	while(!USB_connected && ros::ok()){//waiting for connection with autopilot
		ros::spinOnce();
		loop_rate.sleep();
	}
	int count = 0;
	reset_position_sp();
	reset_yaw_sp();
	while (ros::ok()){
		
		//mode change
		if(cmd.last_flight_mode != cmd.flight_mode){
			reset_position_sp();
			nbs.v_plan = 0.0;
			nbs.tan_acc = 0.0;
			nbs.plan_ok = false;//nurbs needs to be replanned each time
			if(cmd.flight_mode == RASP_NURBS_SEMI||cmd.flight_mode == RASP_NURBS_AUTO){
				//TODO
				//send a request to planner
				RowVector3d Start;
				Start(0) = est.pos[0]/1000.0;
				Start(1) = est.pos[1]/1000.0;
				Start(2) = est.pos[2]/1000.0;
				Matrix<double, Dynamic, 3> Q;
				Q.resize(14, 3);
				Q << 
				0,		0,		0, 
				0,		3,		3,
				0,		4.5,	3,
				0,		6,		3, 
				0,		7.5,	3,
				0,		9,		3, 
				0,		12,		3, 
				3,		12,		3, 
				3,		9,		3, 
				3,		7.5,	3,
				3,		6,		3, 
				3,		4.5,	3,
				3,		3,		3, 
				3,		0,		3;
				for(int i=0;i<14;i++){
					Q(i,0) += Start(0);
					Q(i,1) += Start(1);
					Q(i,2) += Start(2);
				}
				nurbs1.waypts2nurbs(Q);
				trpz1.trpz_gen(nurbs1._len, 2.5, 25, 2.5);
				t_plan = 0.0;
				nbs.plan_ok = true;
			}
		}

		//manuel landing
		if(cmd.rc[2] < -819 && (cmd.flight_mode == RASP_MANUEL || cmd.flight_mode == MANUEL)){
			reset_yaw_sp();
		}

		//give sp according to commander_mode
		if(cmd.commander_mode == ON_BOARD_MODES || cmd.commander_mode == AIDING_MODES){
			rc2sp(LOOP_PERIOD);
		}
		else if(cmd.commander_mode == WAITING_MODES){
			rc_yaw(LOOP_PERIOD);
			//sp frozen for pos ctrl
			//it stays the same 
			//as the last reset_position_sp() during mode change
		}
		else if(cmd.commander_mode == NURBS_SEMI){
			rc_v_plan();
		}
		else if(cmd.commander_mode == NURBS_AUTO){
			//TODO get v_plan from planner
		//	t_plan += 1.0/LOOP_RATE;
		//	if(t_plan >= trpz1._total_time){
		//		t_plan = trpz1._total_time;
		//	}
			
		//	Vector4f javp;
		//	int not_finished = trpz1.allPlan(t_plan, javp);
		//	if(not_finished)
		//		nbs.v_plan = javp(2) * 1000.0;
		//	else
		//		nbs.v_plan = 0;
			float pos = nurbs1._u * nurbs1._len;
			float t = trpz1.inv_posPlan(pos);
			Vector4f javp;
			int not_finished = trpz1.allPlan(t, javp);
			if(not_finished){
				nbs.v_plan = javp(2)*1000.0;
				nbs.tan_acc = javp(1)*1000.0;
			}
			else{
				nbs.v_plan = 0;
				nbs.tan_acc = 0;
			}
		}
		if(cmd.commander_mode == NURBS_SEMI || cmd.commander_mode == NURBS_AUTO){
			rc_yaw(LOOP_PERIOD);
			Matrix<double, 3, 3> psp_vff_aff;
			#define DIS_TAN_ACC_FF 0
			#define USE_TAN_ACC_FF 1
			if(cmd.commander_mode == NURBS_SEMI)
				psp_vff_aff = nurbs1.psp_vff_aff_interp(nbs.v_plan/1000.0, LOOP_PERIOD/1000.0, DIS_TAN_ACC_FF, 0);
			else if(cmd.commander_mode == NURBS_AUTO)
				psp_vff_aff = nurbs1.psp_vff_aff_interp(nbs.v_plan/1000.0, LOOP_PERIOD/1000.0, USE_TAN_ACC_FF, nbs.tan_acc/1000.0);
			for(int i=0; i<3; i++){
				ctrl.pos_sp[i] = psp_vff_aff(i,0)*1000;
				ctrl.vel_ff[i] = psp_vff_aff(i,1)*1000;
				ctrl.acc_ff[i] = psp_vff_aff(i,2)*1000;
			}
		}
		
		//publish the sp
		r2d2::control_sp control_sp_msg;
		for(int i = 0; i < 3; i++){
			control_sp_msg.pos_sp[i] = ctrl.pos_sp[i];
			control_sp_msg.vel_sp[i] = 0;
			control_sp_msg.vel_ff[i] = ctrl.vel_ff[i];
			control_sp_msg.acc_ff[i] = ctrl.acc_ff[i];
		}
		control_sp_msg.pitch_sp = ctrl.pitch_sp;
		control_sp_msg.roll_sp = ctrl.roll_sp;
		control_sp_msg.yaw_sp = ctrl.yaw_sp;
		control_sp_msg.throttle = ctrl.throttle;
		control_sp_msg.time_stamp = cmd.timestamp;
		control_sp_pub.publish(control_sp_msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
