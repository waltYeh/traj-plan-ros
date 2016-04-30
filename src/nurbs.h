#ifndef NURBS_H
#define NURBS_H
#include <Eigen/Eigen/Eigen>
#include <Eigen/Eigen/LU>
using namespace Eigen;
//Vector3d vel_feedforward(double u, double V);
//Vector3d acc_feedforward(double u, double V);
//double interpolation (double u, double V, double Ts);
//Vector3d acc_feedforward_interpolation(double *u, double V, double Ts);
class nurbs
{
public:
	nurbs();
	~nurbs();
	double _u;
	float _len;
	Matrix<double, 3, 3> psp_vff_aff_interp(double V, double Ts, bool use_tan_acc_ff, double tan_acc);
	void waypts2nurbs (const Matrix<double, Dynamic, 3>& Q);
private:
	int p;
	Matrix<double, Dynamic, 3> P;
	VectorXd Knots;
	double getBaseFunVal(double u, int i, int k);
	double getDerivative(double u, int i, int k, int l);
};
#endif
