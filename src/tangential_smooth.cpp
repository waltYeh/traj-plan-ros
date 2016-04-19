
class jerkOptim
{
public:
  jerkOptim();
  ~jerkOptim();
  void trajectory_Paras_generation_i(int num, float p0, float pf, float T, Matrix<float, 3, 3>& Paras_matrix);
  float j_optimal_calculate(int num, float alfa, float beta, float gamma, float t);
  float a_optimal_calculate(int num, float alfa, float beta, float gamma, float t);
  float v_optimal_calculate(int num, float alfa, float beta, float gamma, float t);
  float p_optimal_calculate(int num, float alfa, float beta, float gamma, float t, float p0);
};
class accTrapez
{
public:
  accTrapez();
  ~accTrapez();
  
};
void trajectory_Paras_generation_i(int num, float p0, float pf, float T, Matrix<float, 3, 3>& Paras_matrix)//num 0,1,2 reoresents  x, y, z
{
  float v0 = 0, a0 = 0, vf = 0, af = 0;
  MatrixXd delt_s(3,1);
  delt_s(0,0) = af-a0;
  delt_s(1,0) = vf-v0-a0*T;
  delt_s(2,0) = pf-p0-v0*T-0.5*a0*T*T;

  MatrixXd temp(3,3);
  temp << 60/pow(T,3),-360/pow(T,4),720/pow(T,5),-24/pow(T,2),168/pow(T,3),-360/pow(T,4),3/T,-24/pow(T,2),60/pow(T,3);
  //std::cout << temp;
  MatrixXd const_paras(3,1);//(alfa,beta,gamma)
  const_paras = temp * delt_s;
  //std::cout << const_Paras_matrix;
  Paras_matrix(num,0) = const_paras(0,0);
  Paras_matrix(num,1) = const_paras(1,0);
  Paras_matrix(num,2) = const_paras(2,0);
}

float j_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
  return 0.5*alfa*t*t+beta*t+gamma;
}

float p_optimal_calculate(int num, float alfa, float beta, float gamma, float t, float p0)
{
  return alfa*pow(t,5)/120+beta*pow(t,4)/24+gamma*pow(t,3)/6+p0;
}
float p_simple_calculate(int num, float t, float vel, float p0)
{
  return p0 + vel * t;
}
float v_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
  return alfa*pow(t,4)/24+beta*pow(t,3)/6+gamma*t*t/2;
}

float a_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
  return alfa*pow(t,3)/6+beta*t*t/2+gamma*t;
}

int trapezoidalTraj(float start_pos, float ended_pos, 
  float MAX_v, float MAX_pitch_deg, float MAX_j,
  VectorXf& nodes_time, 
  VectorXf& nodes_vel, 
  VectorXf& nodes_pos, 
  float* p_max_acc)

{
  float MAX_a = 9.8*tan(MAX_pitch_deg/57.3f);
  float max_v, max_a, max_j = MAX_j;
  VectorXf nodes_t = VectorXf::Zero(8);
  VectorXf nodes_p = VectorXf::Zero(8);
  VectorXf nodes_v = VectorXf::Zero(8);
  VectorXf blocks_t = VectorXf::Zero(7);
  float duration_amax = MAX_v / MAX_a - MAX_a / MAX_j;
  float total_length = ended_pos - start_pos;
  float duration_vmax = (total_length - (1 / MAX_a * MAX_v * MAX_v + MAX_a / MAX_j * MAX_v)) / MAX_v;
  if(duration_amax<0 && duration_vmax>0){
    // ROS_INFO("MAX_a unreachable");
    max_a = sqrt(MAX_j * (MAX_v));
    max_v = MAX_v;
    blocks_t(1) = 0;
    duration_vmax = (total_length - (1 / max_a * max_v * max_v + max_a / MAX_j * max_v)) / max_v;
    if (duration_vmax<0){
        duration_vmax = 0;
      //   ROS_INFO("MAX_v unreachable");
      // ROS_INFO("Please use Muellers method");
      return 1;
    }
    blocks_t(3) = duration_vmax;
  }
  else if(duration_amax>0 && duration_vmax<0){
    // ROS_INFO("MAX_v unreachable");
    // ROS_INFO("Please use Muellers method");
    return 1;
  }
  else if(duration_amax<0 && duration_vmax<0){
    // ROS_INFO("Both MAX_a and MAX_v unreachable");
    // ROS_INFO("Please use Muellers method");
    return 1;
  }
  else{
    blocks_t(1) = duration_amax;
    blocks_t(3) = duration_vmax;
    max_a = MAX_a;
    max_v = MAX_v;
  }
  blocks_t(0) = max_a / max_j;
  blocks_t(2) = blocks_t(0);
  blocks_t(4) = blocks_t(2);
  blocks_t(5) = blocks_t(1);
  blocks_t(6) = blocks_t(0);
  nodes_t(0) = 0;
  for(int i = 1; i < 8; i++)
    nodes_t(i) = nodes_t(i - 1) + blocks_t(i - 1);
  nodes_v(0) = 0;
  nodes_v(1) = nodes_v(0) + max_j * blocks_t(0) * blocks_t(0) / 2;
  nodes_v(2) = nodes_v(1) + max_a * blocks_t(1);
  nodes_v(3) = nodes_v(2) + max_a * blocks_t(2) - max_j * blocks_t(2) * blocks_t(2) / 2;
  nodes_v(4) = nodes_v(3);
  nodes_v(5) = nodes_v(4) - max_j * blocks_t(4) * blocks_t(4) / 2;
  nodes_v(6) = nodes_v(5) - max_a * blocks_t(5);
  nodes_v(7) = 0;
  nodes_p(0) = start_pos;
  nodes_p(1) = nodes_p(0) + nodes_v(0) * blocks_t(0) + max_j * blocks_t(0) * blocks_t(0) * blocks_t(0)/6;
  nodes_p(2) = nodes_p(1) + nodes_v(1) * blocks_t(1) + max_a * blocks_t(1) * blocks_t(1) / 2;
  nodes_p(3) = nodes_p(2) + nodes_v(2) * blocks_t(2) + max_a * blocks_t(2) * blocks_t(2) / 2 - max_j * blocks_t(2) * blocks_t(2) * blocks_t(2) / 6;
  nodes_p(4) = nodes_p(3) + nodes_v(3) * blocks_t(3);
  nodes_p(5) = nodes_p(4) + nodes_v(4) * blocks_t(4) - max_j * blocks_t(4) * blocks_t(4) * blocks_t(4)/6;
  nodes_p(6) = nodes_p(5) + nodes_v(5) * blocks_t(5) - max_a * blocks_t(5) * blocks_t(5) / 2;
  nodes_p(7) = ended_pos;
  nodes_time = nodes_t;
  nodes_vel = nodes_v;
  nodes_pos = nodes_p;
  *p_max_acc = max_a;
  return 0;
}
float jerkPlan(float max_jerk, int stage)
{
  float jerk = 0;
  switch (stage){
  case 1:
    jerk = max_jerk;
  break;
  case 2:
    jerk = 0;
  break;
  case 3:
    jerk = -max_jerk;
  break;
  case 4:
    jerk = 0;
  break;
  case 5:
    jerk = -max_jerk;
  break;
  case 6:
    jerk = 0;
  break;
  case 7:
    jerk = max_jerk;
  break;
  default:
  break;
  }
  return jerk;
}
float accPlan(float max_jerk, float max_acc, float t, 
  int stage, const VectorXf& nodes_time)
{
  float tau = t - nodes_time(stage - 1);
  float acc = 0;
  switch (stage){
  case 1:
    acc = max_jerk * tau;
  break;
  case 2:
    acc = max_acc;
  break;
  case 3:
    acc = max_acc - max_jerk * tau;
  break;
  case 4:
    acc = 0;
  break;
  case 5:
    acc = -max_jerk * tau;
  break;
  case 6:
    acc = -max_acc;
  break;
  case 7:
    acc = -max_acc + max_jerk * tau;
  break;
  default:
  break;
  }
  return acc;
}
float velPlan(float max_jerk, float max_acc, float t, 
  int stage, const VectorXf& nodes_time, const VectorXf& nodes_vel)
{
  float tau = t - nodes_time(stage - 1);
  float vel = 0;
  switch (stage){
  case 1:
    vel = nodes_vel(0) + max_jerk * tau * tau / 2;
  break;
  case 2:
    vel = nodes_vel(1) + max_acc * tau;
  break;
  case 3:
    vel = nodes_vel(2) + max_acc * tau - max_jerk * tau * tau / 2;
  break;
  case 4:
    vel = nodes_vel(3);
  break;
  case 5:
    vel = nodes_vel(4) - max_jerk * tau * tau / 2;
  break;
  case 6:
    vel = nodes_vel(5) - max_acc * tau;
  break;
  case 7:
    vel = nodes_vel(6) - max_acc * tau + max_jerk * tau * tau / 2;
  break;
  default:
  break;
  }
  return vel;
}
float posPlan(float max_jerk, float max_acc, float t, 
  int stage, const VectorXf& nodes_time, 
  const VectorXf& nodes_vel, const VectorXf& nodes_pos)
{
  float tau = t - nodes_time(stage - 1);
  float pos = nodes_pos(7);
  switch (stage){
  case 1:
    pos = nodes_pos(0) + nodes_vel(0) * tau + max_jerk * tau * tau * tau / 6;
  break;
  case 2:
    pos = nodes_pos(1) + nodes_vel(1) * tau + max_acc * tau * tau / 2;
  break;
  case 3:
    pos = nodes_pos(2) + nodes_vel(2) * tau + max_acc * tau * tau / 2 - max_jerk * tau * tau * tau / 6;
  break;
  case 4:
    pos = nodes_pos(3) + nodes_vel(3) * tau;
  break;
  case 5:
    pos = nodes_pos(4) + nodes_vel(4) * tau - max_jerk * tau * tau * tau / 6;
  break;
  case 6:
    pos = nodes_pos(5) + nodes_vel(5) * tau - max_acc * tau * tau / 2;
  break;
  case 7:
    pos = nodes_pos(6) + nodes_vel(6) * tau - max_acc * tau * tau / 2 + max_jerk * tau * tau * tau / 6;
  break;
  default:
  break;
  }
  return pos;
}