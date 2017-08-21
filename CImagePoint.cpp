#include <Eigen/Dense>
#include <vector>
#include "CImagePoint.h"

#include "main.h"

CImagePoint::CImagePoint()
{
	// initialize the variables
	x = 0.0;
	y = 0.0;
	xyCovariance = Eigen::MatrixXd::Zero(2, 2);
	flagA = false;
	flagB = false;
	flagI = false;
}

CImagePoint::~CImagePoint()
{

}