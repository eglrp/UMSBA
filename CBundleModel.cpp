#include "CBundleModel.h"
#include "main.h"

//Ceres Cost Function: Collinearity Equation

CBundleModel::CBundleModel(){

	

}
CBundleModel::~CBundleModel(){

}


/***************************
distortionParameters(0) = K0
distortionParameters(1) = K1
distortionParameters(2) = K2
distortionParameters(3) = K3
distortionParameters(4) = P1
distortionParameters(5) = P2
distortionParameters(6) = P3
distortionParameters(7) = A1
distortionParameters(8) = A2
distortionParameters(9-11) = Additional non-physical parameters
***************************/

/*
void CBundleModel::calculateNxNyD(const int imageIndex, const char* imagePointID, Eigen::Vector3d rBmt, Eigen::Matrix3d RbMt, vector<Eigen::Vector3d> cameraPositions, Eigen::Matrix3d mappingToImageRotation, double &Nx, double &Ny, double &D){

	//Object Point Coordinates in mapping frame
	Eigen::Vector3d rIm;
	Eigen::Matrix3d rImCovariance;

	

	Eigen::Vector3d NxNyD = Eigen::Vector3d::Zero(3);

//	NxNyD = mappingToImageRotation*RbMt.transpose()*(rIm - rBmt);

	for (int i = 0; i < cameraPositions.size(); i++)
	{

		NxNyD -= RbMt.transpose()*cameraPositions[i];
	}



	// Pass Nx, Ny, D values back to the problem
	Nx = NxNyD(0);
	Ny = NxNyD(1);
	D = NxNyD(2);

}



//Get rotation and translation components for the master, slave cameras and rotation from image camera to mapping frame
void CBundleModel::getCameraLeverArmAndBoresight(const int imageIndex, Eigen::Matrix3d &mappingToImageRotation, vector<Eigen::Vector3d> &cameraPositions){

	int camIndex = imageBlock->images[imageIndex].cameraIndex;

	Eigen::Matrix3d camRotation, tempRotation;

	//Get Rotation matrix from boresight	
	createRotationMatrixFromOmegaPhiKappa(imageBlock->cameras[camIndex].boresight, camRotation);

	//Get initial mapping to image transformation // To be transposed later
	mappingToImageRotation = camRotation;

	//Get image camera transformation
	Eigen::Vector3d tempCamTransformation = camRotation*imageBlock->cameras[camIndex].leverarm;

	//Output first camera transformation
	cameraPositions.push_back(tempCamTransformation);


	//If Reference Camera ID is not equal to camera ID get reference camera information
	while (strcmp(imageBlock->cameras[camIndex].cameraID, imageBlock->cameras[camIndex].refCameraID) != 0){

		cout << imageBlock->cameras[camIndex].cameraID << endl;
		cout << imageBlock->cameras[camIndex].refCameraID << endl;

		//Get the camIndex of the reference camera
		getReferenceCameraIndex(imageBlock->cameras[camIndex].refCameraID, camIndex);

		cout << "camIndex:" << camIndex << endl;

		//Create Rotation matrix from the boresight angles
		createRotationMatrixFromOmegaPhiKappa(imageBlock->cameras[camIndex].boresight, camRotation);

		//Get orientation of the intermediate/reference camera with respect to the image/slave camera
		camRotation = camRotation*mappingToImageRotation;

		//Get the transformatoin of the new camera
		tempCamTransformation = camRotation*imageBlock->cameras[camIndex].leverarm;

		cameraPositions.push_back(tempCamTransformation);

	}

	cout << mappingToImageRotation(0, 0) << mappingToImageRotation(0, 1) << mappingToImageRotation(0, 2) << endl;

	//Transpose is done to relate the mapping frame to the camera frame
	tempRotation = mappingToImageRotation.transpose();
	mappingToImageRotation = tempRotation;

}


//Get Reference Camera camIndex
void CBundleModel::getReferenceCameraIndex(const char* refCameraID, int &camIndex)
{
	int indexFound = -1;
	for (int i = 0; i < imageBlock->cameras.size(); i++)
	{
		if (strcmp(imageBlock->cameras[i].cameraID, refCameraID) == 0)
		{
			camIndex = i; // Return Reference camera index
			cout << "i: " << camIndex << endl;
			break;
		}
	}

	if (indexFound == -1){

		cout << "Error: Reference Cam not found." << endl;

	}
}


// Adapted from Mehdi's SfM code
void CBundleModel::createRotationMatrixFromOmegaPhiKappa(Eigen::Vector3d boresight, Eigen::Matrix3d &rotationMatrix){

	//Create Rotation matrix from euler angles

	const double kPi = 3.14159265358979323846;
	const double degrees_to_radians(kPi / 180.0);

	const double omega(boresight(0) * degrees_to_radians);
	const double phi(boresight(1) * degrees_to_radians);
	const double kappa(boresight(2) * degrees_to_radians);

	const double cosomega = cos(omega);
	const double sinomega = sin(omega);
	const double cosphi = cos(phi);
	const double sinphi = sin(phi);
	const double coskappa = cos(kappa);
	const double sinkappa = sin(kappa);

	rotationMatrix(0, 0) = cosphi*coskappa;
	rotationMatrix(0, 1) = (cosomega*sinkappa + sinomega*sinphi*coskappa);
	rotationMatrix(0, 2) = (sinomega*sinkappa - cosomega*sinphi*coskappa);
	rotationMatrix(1, 0) = -cosphi*sinkappa;
	rotationMatrix(1, 1) = (cosomega*coskappa - sinomega*sinphi*sinkappa);
	rotationMatrix(1, 2) = (sinomega*coskappa + cosomega*sinphi*sinkappa);
	rotationMatrix(2, 0) = sinphi;
	rotationMatrix(2, 1) = -sinomega*cosphi;
	rotationMatrix(2, 2) = +cosomega*cosphi;

}

//Adapted from Mehdi's SfM code
void CBundleModel::extractOmegaPhiKappaFromRotationMatrix(Eigen::Matrix3d rotationMatrix, Eigen::Vector3d &boresight)
{
	const double kPi = 3.14159265358979323846;
	const double degrees_to_radians(kPi / 180.0);

	const double phi(asin(rotationMatrix(1, 2)));
	const double omega(atan2(-rotationMatrix(2, 0), rotationMatrix(2, 1)));
	const double kapa(atan2(-rotationMatrix(0, 2), rotationMatrix(0, 0)));

	boresight(0) = omega / degrees_to_radians;
	boresight(1) = phi / degrees_to_radians;
	boresight(2) = kapa / degrees_to_radians;

}*/