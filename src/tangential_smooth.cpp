#include "tangential_smooth.h"
#include "std_msgs/String.h"
#include <math.h>
#include <stdio.h>
trpz::trpz()
{

}
trpz::~trpz()
{
  
}

int trpz::trpz_gen(
  float length_given,
  float max_v_given, float max_pitch_deg_given, float max_j_given
  )
{
  float max_a_given = 9.8 * tan(max_pitch_deg_given / 57.3f);
  _max_j = max_j_given;
  _nodes_t = VectorXf::Zero(8);
  _nodes_p = VectorXf::Zero(8);
  _nodes_v = VectorXf::Zero(8);

  _len = length_given;
  _stage = 1;
  VectorXf blocks_t = VectorXf::Zero(7);
  float duration_amax = max_v_given / max_a_given - max_a_given / max_j_given;
  float duration_vmax = (_len - (1.0 / max_a_given * max_v_given * max_v_given + max_a_given / max_j_given * max_v_given)) / max_v_given;
  
  if(duration_amax<0 && duration_vmax>0){
//    ROS_INFO("MAX_a unreachable");
    _max_a = sqrt(max_j_given * (max_v_given));
    _max_v = max_v_given;
    blocks_t(1) = 0;
    duration_vmax = (_len - (1 / _max_a * _max_v * _max_v + _max_a / _max_j * _max_v)) / _max_v;
    if (duration_vmax<0){
        duration_vmax = 0;
        
//        ROS_INFO("MAX_v unreachable");
//      ROS_INFO("Please use Muellers method");
//      return 1;
    }
    blocks_t(3) = duration_vmax;
  }
  else if(duration_vmax<0){
    duration_vmax = 0;
    _max_a = max_a_given;
    float alpha = 1.0f/_max_a;
    float betta = _max_a/_max_j;
    float gamma = _len;
    _max_v = (-betta + sqrt(betta * betta - 4*alpha*gamma))/2/alpha;
    duration_amax = _max_v / _max_a - _max_a / _max_j;
    if(duration_amax < 0){
      duration_amax = 0;
      _max_a = pow((-gamma * _max_j * _max_j /2), 1.0f/3.0f);
    }
//    ROS_INFO("MAX_v unreachable");
//    ROS_INFO("Please use Muellers method");
    blocks_t(1) = duration_amax;
    blocks_t(3) = duration_vmax;
//    return 2;
  }
//  else if(duration_amax<0 && duration_vmax<0){
//    ROS_INFO("Both MAX_a and MAX_v unreachable");
//    ROS_INFO("Please use Muellers method");
//    return 3;
//  }
  else{
    blocks_t(1) = duration_amax;
    blocks_t(3) = duration_vmax;
    _max_a = max_a_given;
    _max_v = max_v_given;
  }
  blocks_t(0) = _max_a / _max_j;
  blocks_t(2) = blocks_t(0);
  blocks_t(4) = blocks_t(2);
  blocks_t(5) = blocks_t(1);
  blocks_t(6) = blocks_t(0);
  _nodes_t(0) = 0;
  for(int i = 1; i < 8; i++)
    _nodes_t(i) = _nodes_t(i - 1) + blocks_t(i - 1);
  _nodes_v(0) = 0;
  _nodes_v(1) = _nodes_v(0) + _max_j * blocks_t(0) * blocks_t(0) / 2;
  _nodes_v(2) = _nodes_v(1) + _max_a * blocks_t(1);
  _nodes_v(3) = _nodes_v(2) + _max_a * blocks_t(2) - _max_j * blocks_t(2) * blocks_t(2) / 2;
  _nodes_v(4) = _nodes_v(3);
  _nodes_v(5) = _nodes_v(4) - _max_j * blocks_t(4) * blocks_t(4) / 2;
  _nodes_v(6) = _nodes_v(5) - _max_a * blocks_t(5);
  _nodes_v(7) = 0;
  _nodes_p(0) = 0;
  _nodes_p(1) = _nodes_p(0) + _nodes_v(0) * blocks_t(0) + _max_j * blocks_t(0) * blocks_t(0) * blocks_t(0)/6;
  _nodes_p(2) = _nodes_p(1) + _nodes_v(1) * blocks_t(1) + _max_a * blocks_t(1) * blocks_t(1) / 2;
  _nodes_p(3) = _nodes_p(2) + _nodes_v(2) * blocks_t(2) + _max_a * blocks_t(2) * blocks_t(2) / 2 - _max_j * blocks_t(2) * blocks_t(2) * blocks_t(2) / 6;
  _nodes_p(4) = _nodes_p(3) + _nodes_v(3) * blocks_t(3);
  _nodes_p(5) = _nodes_p(4) + _nodes_v(4) * blocks_t(4) - _max_j * blocks_t(4) * blocks_t(4) * blocks_t(4)/6;
  _nodes_p(6) = _nodes_p(5) + _nodes_v(5) * blocks_t(5) - _max_a * blocks_t(5) * blocks_t(5) / 2;
  _nodes_p(7) = _len;
  _total_time = _nodes_t(7);
  return 0;
}
float trpz::jerkPlan(float t)
{
  float jerk = 0;
  switch (_stage){
  case 1:
    jerk = _max_j;
  break;
  case 2:
    jerk = 0;
  break;
  case 3:
    jerk = -_max_j;
  break;
  case 4:
    jerk = 0;
  break;
  case 5:
    jerk = -_max_j;
  break;
  case 6:
    jerk = 0;
  break;
  case 7:
    jerk = _max_j;
  break;
  default:
  break;
  }
  return jerk;
}
float trpz::accPlan(float t)
{
  float tau = t - _nodes_t(_stage - 1);
  float acc = 0;
  switch (_stage){
  case 1:
    acc = _max_j * tau;
  break;
  case 2:
    acc = _max_a;
  break;
  case 3:
    acc = _max_a - _max_j * tau;
  break;
  case 4:
    acc = 0;
  break;
  case 5:
    acc = -_max_j * tau;
  break;
  case 6:
    acc = -_max_a;
  break;
  case 7:
    acc = -_max_a + _max_j * tau;
  break;
  default:
  break;
  }
  return acc;
}
float trpz::velPlan(float t, int stage)
{
  float tau = t - _nodes_t(stage - 1);
  float vel = 0;
  switch (stage){
  case 1:
    vel = _nodes_v(0) + _max_j * tau * tau / 2;
  break;
  case 2:
    vel = _nodes_v(1) + _max_a * tau;
  break;
  case 3:
    vel = _nodes_v(2) + _max_a * tau - _max_j * tau * tau / 2;
  break;
  case 4:
    vel = _nodes_v(3);
  break;
  case 5:
    vel = _nodes_v(4) - _max_j * tau * tau / 2;
  break;
  case 6:
    vel = _nodes_v(5) - _max_a * tau;
  break;
  case 7:
    vel = _nodes_v(6) - _max_a * tau + _max_j * tau * tau / 2;
  break;
  default:
  break;
  }
  return vel;
}
float trpz::posPlan(float t, int stage)
{
  float tau = t - _nodes_t(stage - 1);
  float pos = 0;
  switch (stage){
  case 1:
    pos = _nodes_p(0) + _nodes_v(0) * tau + _max_j * tau * tau * tau / 6;
  break;
  case 2:
    pos = _nodes_p(1) + _nodes_v(1) * tau + _max_a * tau * tau / 2;
  break;
  case 3:
    pos = _nodes_p(2) + _nodes_v(2) * tau + _max_a * tau * tau / 2 - _max_j * tau * tau * tau / 6;
  break;
  case 4:
    pos = _nodes_p(3) + _nodes_v(3) * tau;
  break;
  case 5:
    pos = _nodes_p(4) + _nodes_v(4) * tau - _max_j * tau * tau * tau / 6;
  break;
  case 6:
    pos = _nodes_p(5) + _nodes_v(5) * tau - _max_a * tau * tau / 2;
  break;
  case 7:
    pos = _nodes_p(6) + _nodes_v(6) * tau - _max_a * tau * tau / 2 + _max_j * tau * tau * tau / 6;
  break;
  default:
    pos = _nodes_p(7);
  break;
  }
  return pos;
}
int trpz::allPlan(float t, Vector4f& javp)
{
  if(t > _nodes_t(_stage)){
    _stage++;
  }
  if(_stage > 7 || t >= _total_time){
    javp(0) = 0;
    javp(1) = 0;
    javp(2) = 0;
    javp(3) = _nodes_p(7);
    return 0;//finished
  }
  javp(0) = jerkPlan(t);
  javp(1) = accPlan(t);
  javp(2) = velPlan(t, _stage);
  javp(3) = posPlan(t, _stage);
  return 1;
}
#define NEWTON 1
#define BISECT 0 // not stable
#define TOL 0.01
float trpz::inv_posPlan(float pos)
{
  float t2, t1, tc;
  float t, l_t;
  int stage = 1;
  unsigned int iteration = 0;
  for(int i=0; i<7; i++){
    if(pos > _nodes_p(i))
      stage = i+1;
  }
  printf("stage: %d\n", stage);
  #if BISECT
  t2 = _nodes_t(stage);
  t1 = _nodes_t(stage-1);
  while((t2-t1)/2> TOL||(t2-t1)/2< -TOL){
    iteration++;
    tc = (t2+t1)/2;
    if(posPlan(tc, stage) - pos == 0)
      return tc;
    if((posPlan(t1, stage) - pos)*(posPlan(tc, stage) - pos) < 0)
      t2=tc;
    else
      t1 = tc;
  }
  printf("iteration: %d\n", iteration);
  if(_total_time - t < 5*TOL)
    t=_total_time;
  return (t1+t2)/2;

  #elif NEWTON
  
  t = (_nodes_t(stage) + _nodes_t(stage - 1))/2;
  l_t = -1;
  while((t-l_t)/2> TOL||(t-l_t)/2< -TOL){
    iteration++;
    l_t = t;
    float vel = velPlan(t, stage);
    if(vel != 0)
      t = t - (posPlan(t, stage) - pos)/vel;
    else {
      printf("woops");
      if(pos == 0)
        t = 0;
      else
        t = _total_time;
    }

  }
  printf("iteration: %d\n", iteration);
  if(_total_time - t < 5*TOL)
    t=_total_time;
  return t;
  #endif
}


jopt::jopt()
{

}
jopt::~jopt()
{

}
int jopt::jopt_gen(
    const Vector3f& pi, const Vector3f& vi, const Vector3f& ai,
    const Vector3f& pf, const Vector3f& vf, const Vector3f& af,
    float duration
    )
{
  _duration = duration;
  for(int i = 0; i < 3; i++){
    _initState(i, 0) = pi(i);
    _initState(i, 1) = vi(i);
    _initState(i, 2) = ai(i);
  }
  for(int i = 0; i < 3; i++){
    Vector3f delt_s;
    delt_s(0) = af(i)-ai(i);
    delt_s(1) = vf(i)-vi(i)-ai(i)*duration;
    delt_s(2) = pf(i)-pi(i)-vi(i)*duration-0.5*ai(i)*duration*duration;

    Matrix<float, 3, 3> temp;
    temp << 
      60/pow(duration,3),-360/pow(duration,4),720/pow(duration,5),
      -24/pow(duration,2),168/pow(duration,3),-360/pow(duration,4),
      3/duration,-24/pow(duration,2),60/pow(duration,3);
    //std::cout << temp;
    Vector3f const_paras;//(alfa,beta,gamma)
    const_paras = temp * delt_s;
    //std::cout << const_Paras_matrix;
    _param(i,0) = const_paras(0);
    _param(i,1) = const_paras(1);
    _param(i,2) = const_paras(2);
  }
//  std::cout << _param;
}


float jopt::jerkPlan(int axis, float t)
{
  return 0.5*_param(axis, 0)*t*t+_param(axis, 1)*t+_param(axis, 2);
}
float jopt::accPlan(int axis, float t)
{
  return _param(axis, 0)*pow(t,3)/6+_param(axis, 1)*t*t/2+_param(axis, 2)*t+_initState(axis, 2);
}
float jopt::velPlan(int axis, float t)
{
  return _param(axis, 0)*pow(t,4)/24+_param(axis, 1)*pow(t,3)/6+_param(axis, 2)*t*t/2+_initState(axis, 2)*t+_initState(axis, 1);
}
float jopt::posPlan(int axis, float t)
{
  return _param(axis, 0)*pow(t,5)/120+_param(axis, 1)*pow(t,4)/24+_param(axis, 2)*pow(t,3)/6+_initState(axis, 2)*t*t/2+_initState(axis, 1)*t+_initState(axis, 0);
}

int jopt::allPlan(float t, Matrix<float, 3, 4>& javp)
{
  if(t > _duration)
    return 0;
  for(int i = 0; i < 3; i++){
    javp(i, 0) = jerkPlan(i, t);
    javp(i, 1) = accPlan(i, t);
    javp(i, 2) = velPlan(i, t);
    javp(i, 3) = posPlan(i, t);
  }
  return 1;
}