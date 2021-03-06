#include<endohap/Kalman.h>
#include <iostream>
Kalman::Kalman()
{
}

void Kalman::initializeFilter(MatrixXd Ai, MatrixXd Bi, MatrixXd Ci, MatrixXd Qi, MatrixXd Ri, MatrixXd x0)
{
	A = Ai; B = Bi; C = Ci; Q = Qi; R = Ri;	x = x0;

	P = B*Q*B.transpose();
}

MatrixXd Kalman::estimateOutput(MatrixXd yv, MatrixXd u)
{


	MatrixXd M_;
	M_ = C*P*C.transpose()+R;
	Mn = (P*C.transpose())*M_.inverse();

	x = x + Mn*(yv-C*x);

	MatrixXd I = MatrixXd::Identity(A.rows(),A.cols());
	P = (I - Mn*C)*P;

	MatrixXd ye = C*x;

	x = A*x + B*u;
	P = A*P*A.transpose() + B*Q*B.transpose();

	return ye;
}
