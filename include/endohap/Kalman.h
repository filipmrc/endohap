#include <Eigen/Dense>
using Eigen::MatrixXd;

class Kalman
{
public:
	Kalman();

	void initializeFilter(MatrixXd Ai, MatrixXd Bi, MatrixXd Ci, MatrixXd Qi, MatrixXd Ri, MatrixXd x0);

	MatrixXd estimateOutput(MatrixXd yv, MatrixXd u);

private:
	MatrixXd A ,B ,C ,Q ,R ,P ,Mn ,x;

};
