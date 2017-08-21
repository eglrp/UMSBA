#ifndef UMSAT_CNAVIGATION_H
#define UMSAT_CNAVIGATION_H

#include<iostream>
#include<stdio.h>
//#include<ceres/internal/eigen.h>
#include <Eigen/Dense>
#include<vector>
using namespace std;

#pragma once
class CNavigation
{
public:
	CNavigation();							// constructor function
	~CNavigation();							// destructor function

public:
	double GPSTime;							// GPS time tag
	Eigen::Vector3d position;				// a vector which storing the position information X,Y,Z
	Eigen::Vector3d orientation;			// a vector which storing the orientation information Phi,Omega,Kappa
	Eigen::MatrixXd positionCovariance;		// position covariance matrix
	Eigen::MatrixXd orientationCovariance;	// orientation covariance matrix

	//Testing Merge
};

#endif