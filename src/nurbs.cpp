#include "nurbs.h"
#include "std_msgs/String.h"
nurbs::nurbs()
{
	p = 4;
	_u = 0;
}
nurbs::~nurbs()
{

}
// 4th order NURBS is jerk continuous,
// that is to say angular velocity of UAV continuous
double nurbs::getBaseFunVal(double u, int i, int k)
{
	double Val = 0.0;
	double val1 = 0.0;
	double val2 = 0.0;
	int n = P.rows() - 1;
	if (k == 0) {
		if (u < Knots(i) || u >= Knots(i+1))
			if (i == n && u == 1)
				return 1.0;
			else
				return 0.0;
		else {
			return 1.0;
		}
	}
	else if (k > 0) {
		if (u < Knots(i) || u > Knots(i + k + 1))
			return 0.0;
		else {
			double alpha = 0.0;
			double beta = 0.0;
			double denTemp = 0.0;
			//denotation of alpha
			denTemp = Knots(i + k) - Knots(i);
			if (denTemp == 0)
				alpha = 0.0;
			else
				alpha = (u - Knots(i)) / denTemp;
			denTemp = Knots(i + k + 1) - Knots(i + 1);
			if (denTemp == 0)
				beta = 0.0;
			else
				beta = (Knots(i + k + 1) - u) / denTemp;
			val1 = alpha * getBaseFunVal(u,i,k-1);
			val2 = beta * getBaseFunVal(u,i+1,k-1);
			Val = val1 + val2;
		}
	}
	return Val;
}
double nurbs::getDerivative(double u, int i, int k, int l)
{
	double denTemp, val1, val2;
	if (l == 1) {
		denTemp = Knots(i + k) - Knots(i);
		if (denTemp == 0)
			val1 = 0;
		else
			val1 = getBaseFunVal(u, i, k - 1) / denTemp;
		denTemp = Knots(i + k + 1) - Knots(i + 1);
		if (denTemp == 0)
			val2 = 0;
		else
			val2 = getBaseFunVal(u, i + 1, k - 1) / denTemp;		
	}
	else {
		denTemp = Knots(i + k) - Knots(i);
		if (denTemp == 0)
			val1 = 0;
		else
			val1 = getDerivative(u, i, k-1, l - 1) / denTemp;
		denTemp = Knots(i + k + 1) - Knots(i + 1);
		if (denTemp == 0)
			val2 = 0;
		else
			val2 = getDerivative(u, i + 1, k-1, l - 1) / denTemp;
	}
	return (k * (val1 - val2));
}
//u is a referenced value from outside and will be 
//changed hier in the process of interpolation
//the function returns pos_sp, vel_ff, acc_ff in a 3*3 matrix
//Ref Dr. Dong 6-27
Matrix<double, 3, 3> nurbs::psp_vff_aff_interp(double V, double Ts)
{
	static double last_V = 0, last_k = 1;
	//last_k cannot be 0 at first
	int n = P.rows() - 1;
	double curr_u = _u;
	RowVectorXd N = RowVectorXd::Zero(n+1);
	RowVectorXd dN = RowVectorXd::Zero(n+1);
	RowVectorXd ddN = RowVectorXd::Zero(n+1);
	for (int m = 0; m < n+1; m++) {
		N(m) = getBaseFunVal(curr_u, m, p);
		ddN(m) = getDerivative(curr_u, m, p, 2);
		dN(m) = getDerivative(curr_u, m, p, 1);
	}
	Vector3d C = N * P;
	Vector3d C_u = dN * P;
	Vector3d C_uu = ddN * P;
	Vector3d pos_sp = C;
	double k = C_u.norm();
	Vector3d vel_ff = C_u * V / k;
	double V_k = V / k;
	double B = -V * V * C_u.dot(C_uu) / k / k / k / k;
	double new_u = curr_u + V_k * Ts + B * Ts * Ts / 2;
	if (new_u > 1)
		new_u = 1;
	if (new_u <0)
		new_u = 0;
	double last_V_k = last_V / last_k;
	//last_k cannot be 0 at first, but last_V_k is 0 at first
	Vector3d normal_acc = C_uu * V_k * V_k;
	double a_u;
	if(curr_u != new_u){
		a_u = (V_k * V_k - last_V_k * last_V_k) / (new_u - curr_u) / 2;
	}else{
		a_u = 0;
	}
	Vector3d tangential_acc = C_u * a_u;
	Vector3d acc_ff = normal_acc + tangential_acc;
	last_V = V;
	last_k = k;
	
	_u = new_u;
	Matrix<double, 3, 3> retVal;//[pos,vel,acc]
	retVal.col(0) = pos_sp;
	retVal.col(1) = vel_ff;
	retVal.col(2) = acc_ff;
	return retVal;
}
void nurbs::waypts2nurbs (const Matrix<double, Dynamic, 3>& Q)
{
	int n = Q.rows() - 1;
	P.resize(n+1, 3);
	Knots.resize(n + p + 2);
	Knots.Zero(n + p + 2);
	VectorXd chord_len = VectorXd::Zero(n);
	VectorXd step_len = VectorXd::Zero(n + 1);
	for (int i = 0; i<n; i++) {
		Vector3d seg = Q.row(i) - Q.row(i + 1);
		chord_len(i) = seg.norm();
	}
	for (int i = 0; i<n; i++) {
		step_len(i + 1) = step_len(i) + chord_len(i);
	}
	double sum_len = chord_len.sum();
	VectorXd params = step_len / sum_len;
	for (int i = 1; i < n - p + 1; i++) {
		VectorXd para_i = params.segment(i, p);
		Knots(p + i) = 1 / (double)p * para_i.sum();
	}
	Knots.tail(p + 1) = VectorXd::Ones(p + 1);
	Knots.head(p + 1) = VectorXd::Zero(p + 1);
	MatrixXd PHY(n + 1, n + 1);
	for (int i = 0; i < n + 1; i++) {
		for (int j = 0; j < n + 1; j++) {
			PHY(i, j) = getBaseFunVal(params(i), j, p);
		}
	}
	MatrixXd PHY_t = PHY.transpose();
	MatrixXd PHY_t_PHY = PHY_t * PHY;
	MatrixXd PHY_LU = PHY_t_PHY.lu() .solve(PHY_t);
	P = PHY_LU * Q;
}
/*
Vector3d vel_feedforward(double u, double V)
{
	RowVectorXd dN = RowVectorXd::Zero(n+1);
	for(int m = 0; m < n+1; m++) {
		dN(m) = getDerivative(u, m, p, 1);
	}
	Vector3d C_u = dN * P;
	double k = C_u.norm();
	Vector3d vel_ff = C_u * V * k;
	return vel_ff;
}
//denominator might be 0!
Vector3d acc_feedforward(double u, double V)
{
	static double last_V = 0, last_u = 0, last_k = 0;
	RowVectorXd dN = RowVectorXd::Zero(n+1);
	RowVectorXd ddN = RowVectorXd::Zero(n+1);
	for (int m = 0; m < n+1; m++) {
		ddN(m) = getDerivative(u, m, p, 2);
		dN(m) = getDerivative(u, m, p, 1);
	}
	Vector3d C_u = dN * P;
	Vector3d C_uu = ddN * P;
	double k = C_u.norm();
	double V_k = V / k;
	double last_V_k = last_V / last_k;
	Vector3d normal_acc = C_uu * V_k * V_k;
	double a_u = (V_k * V_k - last_V_k * last_V_k) / (u - last_u) / 2;
	Vector3d tangential_acc = C_u * a_u;
	last_V = V;
	last_u = u;
	last_k = k;
	return (normal_acc + tangential_acc);
}
double interpolation (double u, double V, double Ts)
{
	RowVectorXd dN = RowVectorXd::Zero(n+1);
	RowVectorXd ddN = RowVectorXd::Zero(n+1);
	for (int m = 0; m < n+1; m++) {
		ddN(m) = getDerivative(u, m, p, 2);
		dN(m) = getDerivative(u, m, p, 1);
	}
	Vector3d C_u = dN * P;
	Vector3d C_uu = ddN * P;
	double k = C_u.norm();
	double A = V / k;
	double B = -V * V * C_u.dot(C_uu) / k / k / k / k;
	double u_new = u + A * Ts + B * Ts * Ts /2;
	if (u_new > 1)
		u_new = 1;
	if (u_new <0)
		u_new = 0;
	return u_new;
}
Vector3d acc_feedforward_interpolation(double *u, double V, double Ts)
{
	static double last_V = 0, last_u = 0, last_k = 0;
	double curr_u = *u;
	RowVectorXd dN = RowVectorXd::Zero(n+1);
	RowVectorXd ddN = RowVectorXd::Zero(n+1);
	for (int m = 0; m < n+1; m++) {
		ddN(m) = getDerivative(curr_u, m, p, 2);
		dN(m) = getDerivative(curr_u, m, p, 1);
	}
	Vector3d C_u = dN * P;
	Vector3d C_uu = ddN * P;
	double k = C_u.norm();
	double V_k = V / k;
	double B = -V * V * C_u.dot(C_uu) / k / k / k / k;
	double last_V_k = last_V / last_k;
	Vector3d normal_acc = C_uu * V_k * V_k;
	double a_u = (V_k * V_k - last_V_k * last_V_k) / (curr_u - last_u) / 2;
	Vector3d tangential_acc = C_u * a_u;
	last_V = V;
	last_u = curr_u;
	last_k = k;
	double u_new = curr_u + V_k * Ts + B * Ts * Ts / 2;
	if (u_new > 1)
		u_new = 1;
	if (u_new <0)
		u_new = 0;
	*u = u_new;
	return (normal_acc + tangential_acc);
}
*/

