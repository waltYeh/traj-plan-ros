#include <math.h>
#include "commons.h"

void body2glob_2D(float body[], float glob[], float yaw)
{	
	glob[0] = body[0]*cos(yaw) + body[1]*sin(-yaw);
	glob[1] = body[0]*sin(yaw) + body[1]*cos(yaw); 
}
void glob2body_2D(float body[], float glob[], float yaw)//body=inv(R)*glob
{	
	body[0] = glob[0]*cos(yaw) + glob[1]*sin(yaw);
	body[1] = glob[0]*sin(-yaw) + glob[1]*cos(yaw); 
}
//insert a into [b,c]
int constrain(int a, int b, int c)
{
	return ((a)<(b)?(b):(a)>(c)?c:a);
}
//dead zone +-b
int dead_zone(int a, int b)
{
	return ((a)>(b)?(a):(a)<(-b)?(a):0);
}
float constrain_f(float a, float b, float c)
{
	return ((a)<(b)?(b):(a)>(c)?c:a);
}
float dead_zone_f(float a, float b)
{
	return ((a)>(b)?(a):(a)<(-b)?(a):0);
}
