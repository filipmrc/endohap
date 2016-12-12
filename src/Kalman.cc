#include<endohap/Kalman.h>
#include <iostream>
Kalman::Kalman()
{
}

void Kalman::initializeFilter(MatrixXd Ai, MatrixXd Bi, MatrixXd Ci, MatrixXd Qi, MatrixXd Ri, MatrixXd x0)
{
	A = Ai; B = Bi; C = Ci; Q = Qi; R = Ri;

	P = B*Q*B.transpose();

	x.resize(x0.rows(),1);
	x = x0;
}

MatrixXd Kalman::estimateOutput(MatrixXd yv, MatrixXd u)
{
	MatrixXd M_;
	M_ = C*P*C.transpose()+R;
	Mn = (P*C.transpose())*M_.inverse();
	std::cout << Mn << std::endl;

	x = x + Mn*(yv-C*x);
	std::cout << x << std::endl;
	std::cout << C << std::endl;
	std::cout << C*x << std::endl;
	std::cout << (yv-C*x) << std::endl;
	std::cout << x << std::endl;

	MatrixXd I = MatrixXd::Identity(A.rows(),A.cols());
	P = (I - Mn*C)*P;
	std::cout << P << std::endl;

	MatrixXd ye = C*x;

	x = A*x + B*u;
	P = A*P*A.transpose() + B*Q*B.transpose();

	return ye;
}
