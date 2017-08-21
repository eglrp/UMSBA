#pragma once

#ifndef UMSAT_CIMAGEPOINT_H
#define UMSAT_CIMAGEPOINT_H

#include <Eigen/Dense>
#include <vector>

class CImagePoint
{
public:
	CImagePoint();					// constructor function
	~CImagePoint();					// destructor function

public:
	char* imagePointID;				// image point ID
	double x, y;					// image coordinates
	Eigen::MatrixXd xyCovariance;	// Xf to Xd and xyCov to xyCovariance ~ Ron
	bool flagA;						// whether this point is a start point or not
	bool flagB;						// whether this point is a end point or not
	bool flagI;						// whether this point is an intermediate point or not /Updated from flagC ~Ron
};

#endif