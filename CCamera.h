#pragma once

#ifndef UMSAT_CCAMERA_H
#define UMSAT_CCAMERA_H

#include <Eigen/Dense>
#include <vector>


using namespace std;

class CCamera
{
public:
	CCamera();								// constructor function
	~CCamera();								// destructor function
	
public:
	char* cameraID;							// character type variable of cameraID 
	int cameraIndex;						// int type variable of cameraID which associated with the index of the vector which stored the cameraID characters
	char* cameraType;						// character type variable of camera type (either line camera or frame camera)
	char* refCameraID;						// character type variable of the reference cameraID
	double xp, yp, c;						// interior orientation parameters xp, yp and c
	int xMin, xMax, yMin, yMax;				// int type variables of the scene range
	int lineRate;							// int type variable of scan rate
	int pixelSize;							// int type variable of pixel size
	int lineOffset;							// int type variable of the line offset
	int distortionModel;					// int type variable to represent the distortion model which the bundle adjustment adopted
	int numParameters;						// number of parameters
	Eigen::VectorXd distortionParameters;	// an unknown size vector which is storing the distortion parameters
	Eigen::MatrixXd xycCovariance;			// a matrix which is storing the covariance matrix of the interior orientation parameters, which will be initialized in constructor function
	Eigen::MatrixXd distortionCovariance;	// an unknown size matrix which is storing the distortion covariance
	Eigen::Vector3d leverarm;				// a vector which is storing the lever arm (x, y, z)
	Eigen::Vector3d boresight;				// a vector which is storing the bore sight (omega, phi, kappa)
	// vector<double> leverarm;
	// vector<double> boresight;
	Eigen::MatrixXd leverarmCovariance;		// a matrix which is storing the covariance matrix of the lever arm
	Eigen::MatrixXd boresightCovariance;	// a matrix which is storing the covariance matrix of teh bore sight
};

#endif;