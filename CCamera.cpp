#include <Eigen/Dense>
#include <vector>
#include "CCamera.h"

#include "main.h"
#include "UtilityRoutine.h"

using namespace std;

CCamera::CCamera()										// constructor function
{														
	cameraIndex = 0;									// initialize the camera index
	
	xp = 0.0;											// initialize xp
	yp = 0.0;											// initialize yp
	c = 0.0;											// initialize c
	
	distortionModel = 0;								// initialize distortion model
	numParameters = 0;									// initialize number of parameters

	xMax = 0; xMin = 0;									// initialize x max and x min
	yMax = 0; yMin = 0;									// initialize y max and y min
	lineRate = 0;										// initialize line rate
	pixelSize = 0;										// initialize pixel size
	lineOffset = 0;										// initialize line offset

	//distortionParameters = Eigen::VectorXd::Zero(9); //Check with Weifeng ~Ron
	leverarm = Eigen::Vector3d::Zero();					// initialize the lever arm vector
	boresight = Eigen::Vector3d::Zero();				// initialize the bore sight vector

	xycCovariance = Eigen::MatrixXd::Zero(3,3);			// initialize the xyc covariance matrix
	leverarmCovariance = Eigen::MatrixXd::Zero(3,3);	// initialize the lever arm covariance matrix
	boresightCovariance = Eigen::MatrixXd::Zero(3,3);	// initialize the bore sight covariance matrix
}

CCamera::~CCamera()										// destructor function
{
}