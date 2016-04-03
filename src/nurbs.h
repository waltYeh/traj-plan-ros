#ifndef NURBS_H
#define NURBS_H
#include <Eigen/Eigen/Eigen>
#include <Eigen/Eigen/LU>
using namespace Eigen;
//Vector3d vel_feedforward(double u, double V);
//Vector3d acc_feedforward(double u, double V);
//double interpolation (double u, double V, double Ts);
//Vector3d acc_feedforward_interpolation(double *u, double V, double Ts);
Matrix<double, 3, 3> psp_vff_aff_interp(double *u, double V, double Ts, const Matrix<double, Dynamic, 3>& P, const VectorXd& Knots);
void waypts2nurbs (const RowVector3d& start_pt, const Matrix<double, Dynamic, 3>& Q, Matrix<double, Dynamic, 3>& P, VectorXd& Knots);
#endif
