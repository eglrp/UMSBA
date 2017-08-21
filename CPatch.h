#pragma once

#ifndef UMSAT_CPATCH_H
#define UMSAT_CPATCH_H

#include <Eigen/Dense>
#include <vector>


using namespace std;

struct point
{
	double X;
	double Y;
	double Z;
};

class CPatch
{
public:
	CPatch();
	~CPatch();
public:
	char* patchID;
	char* patchA;
	char* patchB;
	char* patchC;
	Eigen::MatrixXd patchCovariance;
	vector <point> patchPoints;
};

#endif;