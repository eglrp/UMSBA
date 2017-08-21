#include "CObjectPoint.h"
#include "main.h"

CObjectPoint::CObjectPoint()
{
	// initialize the variables
	X = 0.0;
	Y = 0.0;
	Z = 0.0;
	XYZCovariance = Eigen::MatrixXd::Zero(3, 3);

}


CObjectPoint::~CObjectPoint()
{
}
