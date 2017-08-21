#pragma once

#ifndef UMSAT_COBJECTPOINT_H
#define UMSAT_COBJECTPOINT_H

#include<iostream>
#include<stdio.h>
//#include<ceres/internal/eigen.h>
#include <Eigen/Dense>
#include<vector>
using namespace std;

#pragma once
class CObjectPoint
{
public:
	CObjectPoint();						// constructor function
	~CObjectPoint();					// destructor function

public:
	char *GCPID;						// object point ID
	double X, Y, Z;						// object point coordinates
	Eigen::MatrixXd XYZCovariance;		// XYZ coordinates covariance matrix
};

#endif