#include <Eigen/Dense>
#include <vector>
#include "CPatch.h"

#include "main.h"

CPatch::CPatch()
{
	patchCovariance = Eigen::MatrixXd::Zero(3, 3);
}

CPatch::~CPatch()
{

}