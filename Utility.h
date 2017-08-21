#pragma once

#ifndef UMSAT_UTILITY_H
#define UMSAT_UTILITY_H

#include <vector>
#include "stdlib.h"
#include "CNavigation.h"
#include <iostream>
//#include "ceres/internal/eigen.h"
#include <Eigen/Dense>
using namespace std;

// the GPS time tag struct which include the time tag ID and a vector which storing the corresponded image indices
struct GPSTimeTag
{
	char * GPSTime;
	vector<int> imageIndex;
	CNavigation navigationData;
};


void split(char* message, Eigen::MatrixXd &covMatrix);			// split an input matrix character which with the format "x x ... x; x x ... x; ...", assign values to a matrix
void split_vector(char* message, Eigen::VectorXd &vector);		// split an input vector character which with the format "x x ... x", assign values to a vector


#endif