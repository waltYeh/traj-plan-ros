#ifndef COMMONS_H
#define COMMONS_H
void body2glob_2D(float body[], float glob[], float yaw);
void glob2body_2D(float body[], float glob[], float yaw);
int constrain(int a, int b, int c);
int dead_zone(int a, int b);
float constrain_f(float a, float b, float c);
float dead_zone_f(float a, float b);
#define GRAVITY 9810
#define VEHICLE_MASS 1300

//flight modes
#define ACROBATIC 0
#define MANUEL 1
#define ALT_CTRL 2	
#define POS_CTRL 4
#define RASP_MANUEL 5
#define RASP_ALT 6
#define RASP_POS 7
#define RASP_NURBS_SEMI 8
#define RASP_NURBS_AUTO 9
#define MOTOR_CUT 10

#endif
