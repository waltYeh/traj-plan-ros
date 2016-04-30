#ifndef TRAJ_SM_H
#define TRAJ_SM_H
#include <Eigen/Eigen/Eigen>
using namespace Eigen;
class trpz
{
public:
	trpz();
	~trpz();
	int trpz_gen(
		float length_given,
		float max_v_given, float max_pitch_deg_given, float max_j_given
		);
	
	int allPlan(float t, Vector4f& javp);
	float inv_posPlan(float pos);
	float _total_time;
private:
	VectorXf _nodes_t;
	VectorXf _nodes_p;
	VectorXf _nodes_v;
	int _stage;
	float _len;
	float _max_j;
	float _max_a;
	float _max_v;
	float jerkPlan(float t);
	float accPlan(float t);
	float velPlan(float t, int stage);
	float posPlan(float t, int stage);
};

class jopt
{
public:
	jopt();
	~jopt();
	int jopt_gen(
		const Vector3f& pi, const Vector3f& vi, const Vector3f& ai,
		const Vector3f& pf, const Vector3f& vf, const Vector3f& af,
		float duration
		);
	
	int allPlan(float t, Matrix<float, 3, 4>& javp);
private:
	Matrix<float, 3, 3> _param;
	Matrix<float, 3, 3> _initState;
	float _duration;
	float jerkPlan(int axis, float t);
	float accPlan(int axis, float t);
	float velPlan(int axis, float t);
	float posPlan(int axis, float t);
};
#endif