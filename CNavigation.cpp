#include "CNavigation.h"
#include "main.h"

CNavigation::CNavigation()
{
	// initialize the variables
	GPSTime = 0.0;
	position = Eigen::Vector3d::Zero(3);
	orientation = Eigen::Vector3d::Zero(3);
	positionCovariance = Eigen::MatrixXd::Zero(3, 3);
	orientationCovariance = Eigen::MatrixXd::Zero(3, 3);
}


CNavigation::~CNavigation()
{
}
