#ifndef UMSAT_CBUNDLEMODEL_H
#define UMSAT_CBUNDLEMODEL_H

#include "main.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/dynamic_autodiff_cost_function.h"

//Camera Block size has been decided based on the maximum number of distortion parameters currently
//being coded. It is 12 for PCI model. Hence, camera block size is calculated by
//xp, yp, c, 3 boresight, 3 leverarm, and 12 distortion parameters which totals 21.
#define CameraBlockSize 21 //Testing 

//Ground Points Block size
#define PointBlockSize 3

//GPS B-frame Location and Orientation Block Size
#define BFrameBlockSize 6


using namespace std;

class CBundleModel
{
public:
	CBundleModel();
	~CBundleModel();


public:

	template <typename T>
	static	void calculateDPRGDistortion(const T * camera, const double x, const double y, T *dist = 0){

		T xp = camera[0];
		T yp = camera[1];

		T dx = T(x) - xp;
		T dy = T(y) - yp;

		T r2 = dx*dx + dy*dy;

		T K[4], P[2], A[2];

		//Vectors K, P and A hold the distortion coefficients according to the DPRG Model
		
		for (int i = 0; i < 4; i++)
		{
			K[i] = camera[i + 9];
		}

		for (int i = 0; i < 3; i++)
		{
			P[i] = camera[i + 13];
		}

		for (int i = 0; i < 2; i++)
		{
			A[i] = camera[i + 16];
		}

		dist[0] = dx*(K[0] + K[1] * r2 + K[2] * r2*r2 + K[3] * r2*r2*r2) //radial
			+ (T(1) + P[2] * r2) * (P[0] * (r2 + T(2) * dx*dx) + P[1] * T(2) * dx*dy) //de-centric
			- A[0] * dx + A[1] * dy; //affine


		dist[1] = dy*(K[0] + K[1] * r2 + K[2] * r2*r2 + K[3] * r2*r2*r2) //radial
			+ (T(1) + P[2] * r2)*(P[1] * (r2 + T(2) * dy*dy) + P[0] * T(2) * dx*dy) //de-centric
			+ A[0] * dy; //affine


	}

	template <typename T>
	static void calculateUofCDistortion(const T * const camera, const double x, const double y, T *dist = 0){

		T xp = camera[0];
		T yp = camera[1];

		T dx = T(x) - xp;
		T dy = T(y) - yp;

		T r2 = dx*dx + dy*dy;


		T K[4], P[3];

		//Vectors K and P hold the distortion coefficients according to the UofC Model

		for (int i = 0; i < 4; i++)
		{
			K[i] = camera[i + 9];
		}

		for (int i = 0; i < 2; i++)
		{
			P[i] = camera[i + 13];
		}


		dist[0] = K[0] * r2*dx + K[1] * r2*r2*dx //radial
			+ K[2] * (r2 + T(2) * dx*dx) + K[3] * T(2) * dx*dy //decentric
			- P[0] * dx + P[1] * dy; //affine



		dist[1] = K[0] * r2*dy + K[1] * r2*r2*dy //radial
			+ K[2] * T(2) * dx*dy + K[3] * (r2 + T(2) * dy*dy) //decentric
			+ P[0] * dy; //affine

	}

	template <typename T>
	static	void calculateSMACDistortion(const T * camera, const double x, const double y, T *dist = 0){

		T xp = camera[0];
		T yp = camera[1];



		T dx = T(x) - xp;
		T dy = T(y) - yp;

		T r2 = dx*dx + dy*dy;


		T K[4], P[3];

		//Vectors K and P hold the distortion coefficients according to the UofC Model

		for (int i = 0; i < 4; i++)
		{
			K[i] = camera[i + 9];
		}

		for (int i = 0; i < 3; i++)
		{
			P[i] = camera[i + 13];
		}


		dist[0] = dx*(K[0] + K[1] * r2 + K[2] * r2*r2 + K[3] * r2*r2*r2) //radial
			+ (T(1) + P[2] * r2)*(P[0] * (r2 + T(2) * dx*dx) + P[1] * T(2) * dx*dy); //decentric



		dist[1] = dy*(K[0] + K[1] * r2 + K[2] * r2*r2 + K[3] * r2*r2*r2) //radial
			+ (T(1) + P[2] * r2)*(P[0] * (r2 + T(2) * dy*dy) + P[1] * T(2) * dx*dy); //decentric



	}

	template <typename T>
	static	void calculatePCIDistortion(const T * camera, const double x, const double y, T *dist){

		T xp = camera[0];
		T yp = camera[1];

		T dx = T(x) - xp;
		T dy = T(y) - yp;

		T r2 = dx*dx + dy*dy;
		T r = sqrt(r2);
		T r3 = r2*r;

		//Vectors K, P and A hold the distortion coefficients according to the PCI Model
		//NP = Additional Non-Physical parameters

		T K[4], P[2], A[2], NP[3];

		for (int i = 0; i < 4; i++)
		{
			K[i] = camera[i + 9];
		}

		for (int i = 4; i < 7; i++)
		{
			P[i] = camera[i + 13];
		}

		for (int i = 0; i < 2; i++)
		{
			A[i] = camera[i + 16];
		}

		for (int i = 0; i < 3; i++)
		{
			NP[i] = camera[i + 18];
		}



		T deltaR = K[0] + K[1] * r + K[2] * r2 + K[3] * r3 + P[0] * r2*r2
			+ P[1] * r3*r2 + P[2] * r3*r3 + A[0] * r3*r2*r2;

		dist[0] = (deltaR *dx) / r + //radial
			(T(1) + NP[1] * r2 + NP[2] * r2*r2)*(A[1] * (r2 + T(2) * dx*dx) + NP[0] * T(2) * dx*dy); //decentric

		dist[1] = (deltaR*dy) / r + //radial
			(T(1) + NP[1] * r2 + NP[2] * r2*r2)*(NP[0] * (r2 + T(2) * dy*dy) + A[1] * T(2) * dx*dy); //decentric

	}

	///Below function adapted from Mehdi's code

	template <typename T>
	static	void createRotationMatrixFromOmegaPhiKappa(const T *angles, Eigen::Matrix<T, 3, 3> &rotationMatrix){

		//Create Rotation matrix from euler angles

		const double kPi = 3.14159265358979323846;
		const double degrees_to_radians(kPi / 180.0);

		const T omega = angles[0] * T(degrees_to_radians);
		const T phi = angles[1] * T(degrees_to_radians);
		const T kappa = angles[2] * T(degrees_to_radians);

		const T cosomega = cos(omega);
		const T sinomega = sin(omega);
		const T cosphi = cos(phi);
		const T sinphi = sin(phi);
		const T coskappa = cos(kappa);
		const T sinkappa = sin(kappa);

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


	static	void extractOmegaPhiKappaFromRotationMatrix(Eigen::Matrix3d rotationMatrix, Eigen::Vector3d &boresight){


		const double kPi = 3.14159265358979323846;
		const double degrees_to_radians(kPi / 180.0);

		const double phi(asin(rotationMatrix(1, 2)));
		const double omega(atan2(-rotationMatrix(2, 0), rotationMatrix(2, 1)));
		const double kappa(atan2(-rotationMatrix(0, 2), rotationMatrix(0, 0)));

		boresight(0) = omega / degrees_to_radians;
		boresight(1) = phi / degrees_to_radians;
		boresight(2) = kappa / degrees_to_radians;
	}

	//Nx, Ny and D computation for collinearity functional models
	//hasRefCamera: 0=No, 1=YES
	//bframe: Body frame position & orientation
	//Camera: camera parameters
	//refcamera: reference camera parameters, NULL if no reference camera
	//camIndex: camera index to check the distortion model
	//obsx, obsy, sigmax, sigmay : observations and scaled variances
	template <typename T>
	static	void calculateNxNyD(const T hasRefCamera, const T *bframe, const T *point, const T *camera, const T *refcamera, const int camIndex, const double obsx, const double obsy, const double sigmax, const double sigmay, const int distortionModel, T *residuals){

		//int cameraIndex = camIndex;
		T distortion[2];
		//Initialize distortion x and y 
		distortion[0] = T(0.0);
		distortion[1] = T(0.0);

		T xp = *(camera + 0);
		T yp = *(camera + 1);
		T c = *(camera + 2);

		//Distortion Model
		//1 = UofC, 2= SMAC, 3 = PCI, 4 = DPRG
		if (distortionModel == 1){
			calculateUofCDistortion(camera, obsx, obsy, distortion);

		}

		else if (distortionModel == 2){
			calculateSMACDistortion(camera, obsx, obsy, distortion);
		}

		else if (distortionModel == 3){

			calculatePCIDistortion(camera, obsx, obsy, distortion);
		}

		else if (distortionModel == 4){

			calculateDPRGDistortion(camera, obsx, obsy, distortion);
		}

		//Object Point Coordinates in mapping frame
		Eigen::Matrix<T, 3, 1> rIm;
		rIm(0) = *(point + 0);
		rIm(1) = *(point + 1);
		rIm(2) = *(point + 2);

		//Body frame position
		Eigen::Matrix<T, 3, 1> rBmt;
		rBmt(0) = *(bframe + 0);
		rBmt(1) = *(bframe + 1);
		rBmt(2) = *(bframe + 2);

		//Body frame orientation
		T angles[3];

		angles[0] = *(bframe + 3);
		angles[1] = *(bframe + 4);
		angles[2] = *(bframe + 5);

		Eigen::Matrix<T, 3, 3> RbMt;
		createRotationMatrixFromOmegaPhiKappa(angles, RbMt); 


		//Get components of the collinearity equation Nx, Ny, D
		Eigen::Matrix<T, 3, 3> mappingToImageRotation;

		Eigen::Matrix<T, 3, 3> camRotation;
		Eigen::Matrix<T, 3, 3> refCamRotation;

		angles[0] = *(camera + 6);
		angles[1] = *(camera + 7);
		angles[2] = *(camera + 8);

		createRotationMatrixFromOmegaPhiKappa(angles, camRotation);

		Eigen::Matrix<T, 3, 1> camLeverArm;

		camLeverArm(0) = *(camera + 3);
		camLeverArm(1) = *(camera + 4);
		camLeverArm(2) = *(camera + 5);


		Eigen::Matrix<T, 3, 1> N;


		N = -camRotation*camLeverArm;

		mappingToImageRotation = camRotation;

		//If reference camera exists, include them in the model
		if (hasRefCamera == T(1))
		{
			T refangles[3];
			Eigen::Matrix<T, 3, 1> refCamLeverArm;

			refangles[0] = *(refcamera + 6);
			refangles[1] = *(refcamera + 7);
			refangles[2] = *(refcamera + 8);

			createRotationMatrixFromOmegaPhiKappa(refangles, refCamRotation);

			refCamLeverArm(0) = *(refcamera + 3);
			refCamLeverArm(1) = *(refcamera + 4);
			refCamLeverArm(2) = *(refcamera + 5);

			mappingToImageRotation = mappingToImageRotation*refCamRotation;

			N -= mappingToImageRotation*refCamLeverArm;

		}

		mappingToImageRotation = mappingToImageRotation*RbMt;

		N += mappingToImageRotation*(rIm - rBmt);

		//Compute estimates for x and y
		T estimatedx = xp - c * (N(0) / N(2)) + distortion[0];
		T estimatedy = yp - c * (N(1) / N(2)) + distortion[1];

		//Pass reprojection error as residuals
		residuals[0] = (estimatedx - T(obsx)) / T(sqrt(sigmax));
		residuals[1] = (estimatedy - T(obsy)) / T(sqrt(sigmay));
//		if (imageBlock->cameras[camIndex].cameraID == "11617056")
//		{
			//#pragma omp single
			//{
			//	cout << imageBlock->cameras[camIndex].cameraID << " " << "x:" << residuals[0] << ", y:" << residuals[1] << endl;
			//}
//		}
	}

	//Function to compute coplanarity triple product
	template <typename T>
	static	void calculateCoplanarityConstraint(const T hasRefCamera, const int camIndex, const double obsx, const double obsy, const double sigmax, const double sigmay, const T* pointA, const T* pointB, const T* camera, const T* refcamera, const T* bframe, const int distortionModel, T *residuals){

		//int cameraIndex = camIndex;

		T distortion[2];
		//Initialize distortion x and y 
		distortion[0] = T(0.0);
		distortion[1] = T(0.0);

		T xp = *(camera + 0);
		T yp = *(camera + 1);
		T c = *(camera + 2);

		//Distortion Model
		//1 = UofC, 2= SMAC, 3 = PCI, 4 = DPRG
		if (distortionModel == 1){
			calculateUofCDistortion(camera, obsx, obsy, distortion);

		}

		else if (distortionModel == 2){
			calculateSMACDistortion(camera, obsx, obsy, distortion);
		}

		else if (distortionModel == 3){

			calculatePCIDistortion(camera, obsx, obsy, distortion);
		}

		else if (distortionModel == 4){

			calculateDPRGDistortion(camera, obsx, obsy, distortion);
		}



		Eigen::Matrix<T, 3, 1> rAcjt;
		Eigen::Matrix<T, 3, 1> rBcjt;
		Eigen::Matrix<T, 3, 1> ricjt;
		Eigen::Matrix<T, 3, 3> imageToMappingRotation;


		//Start Point for linear feature
		Eigen::Matrix<T, 3, 1> rAm;
		rAm(0) = *(pointA + 0); 
		rAm(1) = *(pointA + 1); 
		rAm(2) = *(pointA + 2); 

		//End Point for linear feature
		Eigen::Matrix<T, 3, 1> rBm;
		rBm(0) = *(pointB + 0); 
		rBm(1) = *(pointB + 1); 
		rBm(2) = *(pointB + 2); 


		//Body frame position and orientation
		Eigen::Matrix<T, 3, 1> rBmt;
		rBmt(0) = *(bframe + 0); 
		rBmt(1) = *(bframe + 1); 
		rBmt(2) = *(bframe + 2);

		T RbMtAngles[3];

		RbMtAngles[0] = *(bframe + 3); 
		RbMtAngles[1] = *(bframe + 4); 
		RbMtAngles[2] = *(bframe + 5); 

		Eigen::Matrix<T, 3, 3> RbMt;
		createRotationMatrixFromOmegaPhiKappa(RbMtAngles, RbMt);


		Eigen::Matrix<T, 3, 1> rCjMt;

		rCjMt = rBmt;

		//If reference camera exists, include them into the equations
		if (hasRefCamera == T(1))
		{

			T refCamAngles[3];

			refCamAngles[0] = *(refcamera + 6);
			refCamAngles[1] = *(refcamera + 7);
			refCamAngles[2] = *(refcamera + 8);

			Eigen::Matrix<T, 3, 3> refCamRotation;

			createRotationMatrixFromOmegaPhiKappa(refCamAngles, refCamRotation);
			imageToMappingRotation = RbMt.transpose()*refCamRotation.transpose();

			Eigen::Matrix<T, 3, 1> refCamLeverArm;

			refCamLeverArm(0) = *(refcamera + 3);
			refCamLeverArm(1) = *(refcamera + 4);
			refCamLeverArm(2) = *(refcamera + 5);

			rCjMt += RbMt.transpose()*(refCamLeverArm);
		}
		else
		{
			imageToMappingRotation = RbMt.transpose();
		}


		Eigen::Matrix<T, 3, 1> camLeverArm;

		camLeverArm(0) = *(camera + 3);
		camLeverArm(1) = *(camera + 4);
		camLeverArm(2) = *(camera + 5);

		rCjMt += imageToMappingRotation*camLeverArm;

		T camAngles[3];

		camAngles[0] = *(camera + 6);
		camAngles[1] = *(camera + 7);
		camAngles[2] = *(camera + 8);

		Eigen::Matrix<T, 3, 3> camRotation;

		createRotationMatrixFromOmegaPhiKappa(camAngles, camRotation);

		imageToMappingRotation = imageToMappingRotation*camRotation.transpose();

		///////////////////////////////////////

		rAcjt = rAm - rCjMt;
		rBcjt = rBm - rCjMt;


		ricjt(0) = (T(obsx) - xp - distortion[0]);
		ricjt(1) = (T(obsy) - yp - distortion[1]);
		ricjt(2) = -c;

		Eigen::Matrix<T, 3, 1> ricjmt = imageToMappingRotation*ricjt;


		Eigen::Matrix<T, 3, 1> rAXBcj;
		rAXBcj = rAcjt.cross(rBcjt);

		T rAXBcjMagnitude = sqrt(rAXBcj(0)*rAXBcj(0) + rAXBcj(1)*rAXBcj(1) + rAXBcj(2)*rAXBcj(2));
		
		rAXBcj(0) = rAXBcj(0) / rAXBcjMagnitude;
		rAXBcj(1) = rAXBcj(1) / rAXBcjMagnitude;// Normalize the cross product to  scale the magnitude of residual based on observations
		rAXBcj(2) = rAXBcj(2) / rAXBcjMagnitude;


		T boxProduct = ricjmt.dot(rAXBcj);


		//Compute weight for the coplanarity constraint
		Eigen::Matrix<T, 3, 1> rXY;
		rXY = imageToMappingRotation.transpose()*rAXBcj;

		Eigen::Matrix<T,1,2> aXY;

		aXY(0) = rXY(0);
		aXY(1) = rXY(1);
		
		//Covariance E
		Eigen::Matrix<T,2,2> E;
		E.setZero();
		E(0, 0) = T(sigmax);
		E(1, 1) = T(sigmay);

		// Variance matrix
		T M = aXY*E*aXY.transpose();
		// sqrt(M) is the standard deviation
		residuals[0] = -boxProduct/ sqrt(M);


	}

/////////////////// Frame Camera Functional Models /////////////////////////////////

public:

	struct CollinearityEquationCost{

	private:
		double obsx;
		double obsy;
		double sigmax;
		double sigmay;
		int camIndex;
		int hasRefCam;

		int distortionModel;

	public:
		CollinearityEquationCost(const double obsx, const double obsy, const double sigmax, const double sigmay, const int camIndex, const int distortionModel) :
			obsx(obsx), obsy(obsy), sigmax(sigmax), sigmay(sigmay), camIndex(camIndex), distortionModel(distortionModel)
		{		}

		template <typename T>
		bool operator()(const T* const camera, const T* const point, const T* const bframe, T* residuals) const {


			T hasRefCamera = T(0);
			T *refcamera = 0;

			calculateNxNyD(hasRefCamera, bframe, point, camera, refcamera, camIndex, obsx, obsy,sigmax,sigmay, distortionModel, residuals);

			return true;

		}

		//Creating Ceres Cost Fuction
		static ceres::CostFunction* Create(const double obsx, const double obsy, const double sigmax, const double sigmay, const int camIndex, const int distortionModel){
			return(new ceres::AutoDiffCostFunction<CollinearityEquationCost, 2, CameraBlockSize, PointBlockSize, BFrameBlockSize>(
				new CollinearityEquationCost(obsx, obsy, sigmax, sigmay, camIndex, distortionModel)));
		}
	};

public:
	struct CollinearityEquationReferenceCost{

	private:
		double obsx;
		double obsy;
		double sigmax;
		double sigmay;
		int camIndex;

		int distortionModel;

	public:
		CollinearityEquationReferenceCost(const double obsx, const double obsy, const double sigmax, const double sigmay, const int camIndex, const int distortionModel) :
			obsx(obsx), obsy(obsy), sigmax(sigmax), sigmay(sigmay), camIndex(camIndex), distortionModel(distortionModel)
		{		}

		template <typename T>
		bool operator()(const T* const camera, const T* const refcamera, const T* const point, const T* const bframe, T* residuals) const {

			T hasRefCamera = T(1);

			calculateNxNyD(hasRefCamera, bframe, point, camera, refcamera, camIndex, obsx, obsy, sigmax, sigmay, distortionModel, residuals);


			return true;

		}

		//Creating Ceres Cost Fuction
		static ceres::CostFunction* Create(const double obsx, const double obsy, const double sigmax, const double sigmay, const int camIndex, const int distortionModel){
			return(new ceres::AutoDiffCostFunction<CollinearityEquationReferenceCost, 2, CameraBlockSize, CameraBlockSize, PointBlockSize, BFrameBlockSize>(
				new CollinearityEquationReferenceCost(obsx, obsy, sigmax, sigmay, camIndex, distortionModel)));
		}
	};

	//Adpated from Mehdi's Code
	//Distance observations functional model
public:

	struct DistanceCost {

	private:
		double  observed_dist;
		double  std_dist;

	public:
		DistanceCost(const double observed_dist, const double std_dist)
			:observed_dist(observed_dist), std_dist(std_dist) {}

		template <typename T>
		bool operator()(const T* const point1, const T* const point2, T* residuals) const {

			T P1[3], P2[3];

			P1[0] = *(point1 + 0);
			P1[1] = *(point1 + 1);
			P1[2] = *(point1 + 2);

			P2[0] = *(point2 + 0);
			P2[1] = *(point2 + 1);
			P2[2] = *(point2 + 2);

			T estimated_dist = sqrt((P1[0] - P2[0])*(P1[0] - P2[0]) + (P1[1] - P2[1])*(P1[1] - P2[1]) + (P1[2] - P2[2])*(P1[2] - P2[2]));


			if (std_dist < 0)
			{
				residuals[0] = T(0.0);
			}
			else
			{
				residuals[0] = (T(observed_dist) - estimated_dist) / T(sqrt(std_dist));

			}


			return true;
		}

		// Factory to hide the construction of the CostFunction object from
		// the client code.
		static ceres::CostFunction* Create(const double  observed_dist, const double  std_dist)
		{
			return (new ceres::AutoDiffCostFunction<DistanceCost, 1, PointBlockSize, PointBlockSize>(
				new DistanceCost(observed_dist, std_dist)));
		}


	};

public:
	//Patch observations fuctional model
	struct PatchCoordinateCost {

	private:
		double  obsX, obsY, obsZ;
		double  sigmaX, sigmaY, sigmaZ;

	public:
		PatchCoordinateCost(const double obsX, const double obsY, const double obsZ, const double sigmaX, const double sigmaY, const double sigmaZ)
			:obsX(obsX), obsY(obsY), obsZ(obsZ), sigmaX(sigmaX), sigmaY(sigmaY), sigmaZ(sigmaZ) {}

		template <typename T>
		bool operator()(const T* const patchA, const T* const patchB, const T* const patchC, T* residuals) const {

			Eigen::Matrix<T, 4, 4> XABPL;

			XABPL(0, 0) = T(obsX);
			XABPL(0, 1) = T(obsY);
			XABPL(0, 2) = T(obsZ);
			XABPL(0, 3) = T(1.0);

			XABPL(1, 0) = patchA[0];
			XABPL(1, 1) = patchA[1];
			XABPL(1, 2) = patchA[2];
			XABPL(1, 3) = T(1.0);

			XABPL(2, 0) = patchB[0];
			XABPL(2, 1) = patchB[1];
			XABPL(2, 2) = patchB[2];
			XABPL(2, 3) = T(1.0);

			XABPL(3,0) = patchC[0];
			XABPL(3,1) = patchC[1];
			XABPL(3,2) = patchC[2];
			XABPL(3, 3) = T(1.0);
			
			T pyramidVolume = XABPL.determinant();
			
			// Scaled Covariance Matrix
			Eigen::Matrix<T, 3, 3> E;
			E.setZero();
			E(0, 0) = T(sigmaX);
			E(1, 1) = T(sigmaY);
			E(2, 2) = T(sigmaZ);

			//Creating M Matrix (weight for volume constraint)
			Eigen::Matrix<T, 1, 3> bP;

			bP(0) = patchA[1] * (patchB[2] - patchC[2]) - patchA[2] * (patchB[1] - patchC[1]) + (patchB[1] * patchC[2] - patchC[1] * patchB[2]);
			bP(1) = -(patchA[0] * (patchB[2] - patchC[2]) - patchA[2] * (patchB[0] - patchC[0]) + (patchB[0] * patchC[2] - patchC[0] * patchB[2]));
			bP(2) = patchA[0] * (patchB[1] - patchC[1]) - patchA[1] * (patchB[0] - patchC[0]) + (patchB[0] * patchC[1] - patchC[0] * patchB[1]);
			

			T M = bP*E*bP.transpose();


			residuals[0] = -pyramidVolume/sqrt(M);

			return true;
		}

		// Factory to hide the construction of the CostFunction object from
		// the client code.
		static ceres::CostFunction* Create(const double obsX, const double obsY, const double obsZ, const double sigmaX, const double sigmaY, const double sigmaZ)
		{
			return (new ceres::AutoDiffCostFunction<PatchCoordinateCost, 1, PointBlockSize, PointBlockSize, PointBlockSize>(
				new PatchCoordinateCost(obsX, obsY, obsZ, sigmaX, sigmaY, sigmaZ)));
		}


	};


	////!!!!!Adapted from Mehdi's code
	//Function for weighting parameters when prior information is available
	class PseudoObservation :public ceres::CostFunction {
	private:
		const double * std_params;
		double scaleSigma;
		
	public:
		std::vector<double> Observed_param;
		

	public:

		PseudoObservation(int num_params_, const double * std_params_, const double scaleSigma_)
		{
			set_num_residuals(num_params_);//number of residuals equals to params
			mutable_parameter_block_sizes()->push_back(num_params_);
			std_params = std_params_;
			scaleSigma = scaleSigma_;

		}
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const {

			int nres = num_residuals();


			for (int i = 0; i < nres; i++)
			{
				if (std_params[i]>sigmaFixed)//only define residual for
				{
					residuals[i] = (parameters[0][i] - Observed_param[i]) *scaleSigma / (sqrt(std_params[i]));
				}
				else
				{
					residuals[i] = 0.0;
				}
			}


			if (jacobians != NULL && jacobians[0] != NULL){

				for (int i = 0; i < nres; i++)
				{
					for (int j = 0; j < nres; j++)
					{
						if (std_params[i]>sigmaFixed  && i == j)
						{
							jacobians[0][i*nres + j] = +1.0  *scaleSigma / (sqrt(std_params[i]));

						}
						else
							jacobians[0][i*nres + j] = 0.0;
					}
				}
			}

			return true;
		}

	};

	//Scaling and weighting for IOPs
	//This has been done separately since parameters must be passed as a block for weighting
	//Hence along with the scaling for distortion parameters, other IOPs are weighed as well
	class ScaleAndWeightCameraParameters :public ceres::CostFunction {
	private:
		const double * std_params;
		double scaleSigma;
		int distortionModel;

	public:
		std::vector<double> Observed_param;


	public:

		ScaleAndWeightCameraParameters(int num_params_, const double * std_params_, const double scaleSigma_, const int distortionModel_)
		{
			set_num_residuals(num_params_);//number of residuals equals to params
			mutable_parameter_block_sizes()->push_back(num_params_);
			std_params = std_params_;
			scaleSigma = scaleSigma_;
			distortionModel = distortionModel_;

		}
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const {

			int nres = num_residuals();

			double SCALE1, SCALE2, SCALE3, SCALE4, SCALE5, SCALE6, SCALE7, SCALE8, SCALE9, SCALE10, SCALE11, SCALE12;

			if (distortionModel == 1) //UofC
			{
				SCALE1 = 10000; //Radial Scale one
				SCALE2 = 10000; //Radial Scale Two

				SCALE3 = 10000; //Decentric Scale one
				SCALE4 = 10000; //Decentric Scale Two

				SCALE5 = 100; //Affine Scale one
				SCALE6 = 100; //Affine Scale Two

				SCALE7 = 1.0; //IOPs not part of the model
				SCALE8 = 1.0;
				SCALE9 = 1.0;
				SCALE10 = 1.0;
				SCALE11 = 1.0;
				SCALE12 = 1.0;

			}
			else if (distortionModel == 2) //SMAC
			{
				SCALE1 = 1000000; //Radial distortion scale
				SCALE2 = 1000000;
				SCALE3 = 1000000;
				SCALE4 = 1000000;

				SCALE5 = 1000000; //Decentric distortion scale
				SCALE6 = 1000000;
				SCALE7 = 1000000;

				SCALE8 = 1.0; //IOPs not part of the model
				SCALE9 = 1.0;
				SCALE10 = 1.0;
				SCALE11 = 1.0;
				SCALE12 = 1.0;


			}
			else if (distortionModel == 3) //PCI
			{
				SCALE1 = 1000000; //Radial distortion scale
				SCALE2 = 1000000;
				SCALE3 = 1000000;
				SCALE4 = 1000000;
				SCALE5 = 1000000;
				SCALE6 = 1000000;
				SCALE7 = 1000000;
				SCALE8 = 1000000;

				SCALE9 = 1000000; //Decentric distortion scale
				SCALE10 = 1000000;
				SCALE11 = 1000000;
				SCALE12 = 1000000;

			}
			else if (distortionModel == 4) // DPRG
			{
				SCALE1 = 1000000; //Radial distortion scale
				SCALE2 = 1000000;
				SCALE3 = 1000000;
				SCALE4 = 1000000;

				SCALE5 = 1000000; //Decentric distortion scale
				SCALE6 = 1000000;

				SCALE7 = 1000000; //Affine scale one
				SCALE8 = 1000000; //Affine scale two

				SCALE9 = 1.0; //IOPs not part of the model
				SCALE10 = 1.0;
				SCALE11 = 1.0;
				SCALE12 = 1.0;

			}

			for (int i = 0; i < nres; i++)
			{
				if (std_params[i] > sigmaFixed && i<9)//only define residual for
				{
					residuals[i] = (parameters[0][i] - Observed_param[i]) *scaleSigma / (sqrt(std_params[i]));
				}
				else if (std_params[i] > sigmaFixed && i == 9)
				{
					residuals[i] = (parameters[0][i] - Observed_param[i]) *scaleSigma / (sqrt(std_params[i])*SCALE1);
				}
				else if (std_params[i] > sigmaFixed && i == 10)
				{
					residuals[i] = (parameters[0][i] - Observed_param[i])  *scaleSigma / (sqrt(std_params[i])*SCALE2);
				}
				else if (std_params[i] > sigmaFixed && i == 11)
				{
					residuals[i] = (parameters[0][i] - Observed_param[i]) *scaleSigma / (sqrt(std_params[i])*SCALE3);
				}
				else if (std_params[i] > sigmaFixed && i == 12)
				{
					residuals[i] = (parameters[0][i] - Observed_param[i])  *scaleSigma / (sqrt(std_params[i])*SCALE4);
				}
				else if (std_params[i] > sigmaFixed && i == 13)
				{
					residuals[i] = (parameters[0][i] - Observed_param[i]) *scaleSigma / (sqrt(std_params[i])*SCALE5);
				}
				else if (std_params[i] > sigmaFixed && i == 14)
				{
					residuals[i] = (parameters[0][i] - Observed_param[i]) *scaleSigma / (sqrt(std_params[i])*SCALE6);
				}
				else if (std_params[i] > sigmaFixed && i == 15)
				{
					residuals[i] = (parameters[0][i] - Observed_param[i]) *scaleSigma / (sqrt(std_params[i])*SCALE7);
				}
				else if (std_params[i] > sigmaFixed && i == 16)
				{
					residuals[i] = (parameters[0][i] - Observed_param[i]) *scaleSigma / (sqrt(std_params[i])*SCALE8);
				}
				else if (std_params[i] > sigmaFixed && i == 17)
				{
					residuals[i] = (parameters[0][i] - Observed_param[i])  *scaleSigma / (sqrt(std_params[i])*SCALE9);
				}
				else if (std_params[i] > sigmaFixed && i == 18)
				{
					residuals[i] = (parameters[0][i] - Observed_param[i]) *scaleSigma / (sqrt(std_params[i])*SCALE10);
				}
				else if (std_params[i] > sigmaFixed && i == 19)
				{
					residuals[i] = (parameters[0][i] - Observed_param[i]) *scaleSigma / (sqrt(std_params[i])*SCALE11);
				}
				else if (std_params[i] > sigmaFixed && i == 20)
				{
					residuals[i] = (parameters[0][i] - Observed_param[i]) *scaleSigma / (sqrt(std_params[i])*SCALE12);
				}
				else
				{
					residuals[i] = 0.0;
				}
			}


			if (jacobians != NULL && jacobians[0] != NULL){

				for (int i = 0; i < nres; i++)
				{
					for (int j = 0; j < nres; j++)
					{

						if (std_params[j]>sigmaFixed && i == j && i<9)
						{
							jacobians[0][i*nres + j] = +1.0  *scaleSigma / (sqrt(std_params[i]));

						}
						else if (std_params[j]>sigmaFixed && i == j && i == 9)
						{
							jacobians[0][i*nres + j] = +1.0 *scaleSigma / (sqrt(std_params[i])*SCALE1);

						}
						else if (std_params[j]>sigmaFixed && i == j && i == 10)
						{
							jacobians[0][i*nres + j] = +1.0 *scaleSigma / (sqrt(std_params[i])*SCALE2);

						}
						else if (std_params[j]>sigmaFixed && i == j && i == 11)
						{
							jacobians[0][i*nres + j] = +1.0 *scaleSigma / (sqrt(std_params[i])*SCALE3);

						}
						else if (std_params[j]>sigmaFixed && i == j && i == 12)
						{
							jacobians[0][i*nres + j] = +1.0 *scaleSigma / (sqrt(std_params[i])*SCALE4);

						}
						else if (std_params[j]>sigmaFixed && i == j && i == 13)
						{
							jacobians[0][i*nres + j] = +1.0  *scaleSigma / (sqrt(std_params[i])*SCALE5);

						}
						else if (std_params[j]>sigmaFixed && i == j && i == 14)
						{
							jacobians[0][i*nres + j] = +1.0  *scaleSigma / (sqrt(std_params[i])*SCALE6);

						}
						else if (std_params[j]>sigmaFixed && i == j && i == 15)
						{
							jacobians[0][i*nres + j] = +1.0 *scaleSigma / (sqrt(std_params[i])*SCALE7);

						}
						else if (std_params[j]>sigmaFixed && i == j && i == 16)
						{
							jacobians[0][i*nres + j] = +1.0 *scaleSigma / (sqrt(std_params[i])*SCALE8);

						}
						else if (std_params[j]>sigmaFixed && i == j && i == 17)
						{
							jacobians[0][i*nres + j] = +1.0 *scaleSigma / (sqrt(std_params[i])*SCALE9);

						}
						else if (std_params[j]>sigmaFixed && i == j && i == 18)
						{
							jacobians[0][i*nres + j] = +1.0*scaleSigma / (sqrt(std_params[i])*SCALE10);

						}
						else if (std_params[j]>sigmaFixed && i == j && i == 19)
						{
							jacobians[0][i*nres + j] = +1.0 *scaleSigma / (sqrt(std_params[i])*SCALE11);

						}
						else if (std_params[j]>sigmaFixed && i == j && i == 20)
						{
							jacobians[0][i*nres + j] = +1.0  *scaleSigma / (sqrt(std_params[i])*SCALE12);

						}
						else
							jacobians[0][i*nres + j] = 0.0;
					}
				}
			}

			return true;
		}

	};

public:
	struct CoplanarityEquationCost{

	private:
		double obsx;
		double obsy;
		double sigmax;
		double sigmay;
		int camIndex;

		int distortionModel;

	public:
		CoplanarityEquationCost(double obsx, double obsy, double sigmax, double sigmay, int camIndex, int distortionModel) :
			obsx(obsx), obsy(obsy), sigmax(sigmax), sigmay(sigmay), camIndex(camIndex), distortionModel(distortionModel)
		{		}


		template <typename T>
		bool operator()(const T* const camera, const T* const pointA, const T* const pointB, const T* const bframe, T* residuals) const {

			T hasRefCamera = T(0);

			calculateCoplanarityConstraint(hasRefCamera, camIndex, obsx, obsy, sigmax, sigmay, pointA, pointB, camera, camera, bframe, distortionModel, residuals);
		
			return true;

		}

		//Creating Ceres Cost Fuction
		static ceres::CostFunction* Create(double obsx, double obsy, double sigmax, double sigmay, int camIndex, int distortionModel){
			return(new ceres::AutoDiffCostFunction<CoplanarityEquationCost, 1, CameraBlockSize, PointBlockSize, PointBlockSize, BFrameBlockSize>(
				new CoplanarityEquationCost(obsx, obsy, sigmax, sigmay, camIndex, distortionModel)));
		}
	};

	public:
		struct CoplanarityEquationReferenceCost{

		private:
			double obsx;
			double obsy;
			double sigmax;
			double sigmay;
			int camIndex;

			int distortionModel;

		public:
			CoplanarityEquationReferenceCost(double obsx, double obsy, double sigmax, double sigmay, int camIndex, int distortionModel) :
				obsx(obsx), obsy(obsy), sigmax(sigmax), sigmay(sigmay), camIndex(camIndex), distortionModel(distortionModel)
			{		}


			template <typename T>
			bool operator()(const T* const camera, const T* const refcamera, const T* const pointA, const T* const pointB, const T* const bframe, T* residuals) const {

				T hasRefCamera = T(1);

				calculateCoplanarityConstraint(hasRefCamera, camIndex, obsx, obsy, sigmax, sigmay, pointA, pointB, camera, refcamera, bframe, distortionModel, residuals);

				
				return true;

			}

			//Creating Ceres Cost Fuction
			static ceres::CostFunction* Create(double obsx, double obsy, double sigmax, double sigmay, int camIndex, int distortionModel){
				return(new ceres::AutoDiffCostFunction<CoplanarityEquationReferenceCost, 1, CameraBlockSize, CameraBlockSize, PointBlockSize, PointBlockSize, BFrameBlockSize>(
					new CoplanarityEquationReferenceCost(obsx, obsy, sigmax, sigmay, camIndex, distortionModel)));
			}
		};



//Functions used by line camera functional models

template <typename T>
static void getOrientationImageInterPolated(int index, const double time, const double zeroOriTime, const double*oriTime, const int interPolationOrder, const T* const* parameters, T *oriImage){

	
	int n = interPolationOrder + 1;


	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> omegaOri(n, 1);
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> phiOri(n, 1);
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> kappaOri(n, 1);
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> XoOri(n, 1);
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> YoOri(n, 1);
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> ZoOri(n, 1);




	//Get corresponding reference GPS/INS data for interpolation
		for (int i = 0; i < n; i++)
		{
			omegaOri(i, 0) = parameters[i + index + 2][3]; //index defines the start of the orientation data
			phiOri(i, 0) = parameters[i + index + 2][4];
			kappaOri(i, 0) = parameters[i + index+ 2][5];

			XoOri(i, 0) = parameters[i + index + 2][0];
			YoOri(i, 0) = parameters[i + index + 2][1];
			ZoOri(i, 0) = parameters[i + index + 2][2];
		}
	


	//MMatrix needs to be moved to a different place for efficiency
	
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MMatrix(n, n);
	
	//calculateMMatrix(time, n, MMatrix); //currently lower set as zero. !!Needs Updating

	//////////Compute MMatrix//////////////////

	double dtime;

	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A(n, n);


	for (int i = 0; i<n; i++)
	{
		dtime = (oriTime[i] - oriTime[0]); //time difference


		for (int j = 0; j<n; j++)
		{
			A(i, j) = T(pow(dtime, j));
		}
	}


	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> N(n, n);
	N = A.transpose()*A;

	MMatrix = N.inverse()*A.transpose();

	///////////////////////////////////////

	//Compute coefficients for omega

	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Coefficients(n, 1);
	Coefficients= MMatrix*omegaOri;

	//double Omega, Phi, Kappa; //Orientation Images

	//calculate the coefficient for omega
	oriImage[3] = T(0.0);

	for (int k = 0; k<n; k++)
	{
		oriImage[3] += Coefficients(k, 0)*T(pow(time, k));
	}

	//calculate the coefficient for phi
	Coefficients = MMatrix*phiOri;
	oriImage[4] = T(0.0);
	for (int k = 0; k<n; k++)
	{
		oriImage[4] += Coefficients(k, 0)*T(pow(time, k));
	}

	//calculate the coefficient for kappa
	Coefficients = MMatrix*kappaOri;
	oriImage[5] = T(0.0);
	for (int k = 0; k<n; k++)
	{
		oriImage[5] += Coefficients(k, 0)*T(pow(time, k));
	}



	//calculate the coefficient for Xo
	oriImage[0] = T(0.0);
	Coefficients = MMatrix*XoOri;
	for (int k = 0; k<n; k++)
	{
		oriImage[0] += Coefficients(k, 0)*T(pow(time, k));
	}

	//calculate the coefficient for Yo
	Coefficients = MMatrix*YoOri;
	oriImage[1] = T(0.0);
	for (int k = 0; k<n; k++)
	{
		oriImage[1] += Coefficients(k, 0)*T(pow(time, k));
	}

	//calculate the coefficient for Zo
	Coefficients = MMatrix*ZoOri;
	oriImage[2] = T(0.0);
	for (int k = 0; k<n; k++)
	{
		oriImage[2] += Coefficients(k, 0)*T(pow(time, k));
	}


}

//////////Functional Models Line Camera


	class CollinearityEquationCostDynamicLineCamera {


	public:
	typedef ceres::DynamicAutoDiffCostFunction<CollinearityEquationCostDynamicLineCamera, 4> CollinearityEquationDynamicLineCameraCostFunction;

	private:
		double obsx;
		double obsy;
		double sigmax;
		double sigmay;
		int camIndex;
		int hasRefCam;
		int imgIndex;
		double time;
		double *oriTime;
		int interPolationOrder;
		double zeroOriTime;

		int distortionModel;


	public:
		CollinearityEquationCostDynamicLineCamera(const double obsx, const double obsy, const double sigmax, const double sigmay, const int camIndex, double time, double *oriTime, double zeroOriTime,const int interPolationOrder, const int distortionModel) :
			obsx(obsx), obsy(obsy), sigmax(sigmax), sigmay(sigmay), camIndex(camIndex), time(time), oriTime(oriTime), zeroOriTime(zeroOriTime), interPolationOrder(interPolationOrder), distortionModel(distortionModel)
		{		}

		template <typename T>
		bool operator()(const T* const* parameters, T* residuals) const {

			//Get Orientation Image for Line camera


			T hasRefCamera = T(0);
			T *refcamera = 0;

			T oriImage[BFrameBlockSize];

			//Parameter order: camera, point, oriImages

			T camera[CameraBlockSize];

			for (int i = 0; i < CameraBlockSize; i++)//Cam block size = 21
			{
				camera[i] = parameters[0][i];

			}

			T point[PointBlockSize];

			for (int i = 0; i < PointBlockSize; i++)//Size of point block = 3
			{
				point[i] = parameters[1][i];

			}

			getOrientationImageInterPolated(0,time,zeroOriTime, oriTime, interPolationOrder, parameters, oriImage);

			//calculateNxNyD(hasRefCamera, oriImage, point, camera, refcamera, N);
			calculateNxNyD(hasRefCamera, oriImage, point, camera, refcamera, camIndex, obsx, obsy, sigmax, sigmay, distortionModel, residuals);

			return true;

		}

		//Creating Ceres Cost Fuction
		static CollinearityEquationDynamicLineCameraCostFunction* Create(const double obsx, const double obsy, const double sigmax, const double sigmay, const int camIndex, double time, double *oriTime, double zeroOriTime, const int interPolationOrder, const int distortionModel){
			CollinearityEquationCostDynamicLineCamera* dynamicCost = new CollinearityEquationCostDynamicLineCamera(obsx, obsy, sigmax, sigmay, camIndex, time, oriTime, zeroOriTime,interPolationOrder, distortionModel);
			CollinearityEquationDynamicLineCameraCostFunction *dynamicCostFunction = new CollinearityEquationDynamicLineCameraCostFunction(dynamicCost);

			return (dynamicCostFunction);


		}
	};



public:
	struct CollinearityEquationReferenceCostDynamicLineCamera{

	public:
		typedef ceres::DynamicAutoDiffCostFunction<CollinearityEquationReferenceCostDynamicLineCamera, 4> CollinearityEquationReferenceCostDynamicLineCameraCostFunction;

	private:
		double obsx;
		double obsy;
		double sigmax;
		double sigmay;
		int camIndex;
		int hasRefCam;
		int imgIndex;
		double time;
		double *oriTime;
		int interPolationOrder;
		double zeroOriTime;

		int distortionModel;

	public:
		CollinearityEquationReferenceCostDynamicLineCamera(const double obsx, const double obsy, const double sigmax, const double sigmay, const int camIndex, double time, double *oriTime, double zeroOriTime,const int interPolationOrder, const int distortionModel) :
			obsx(obsx), obsy(obsy), sigmax(sigmax), sigmay(sigmay), camIndex(camIndex), time(time), oriTime(oriTime), zeroOriTime(zeroOriTime), interPolationOrder(interPolationOrder), distortionModel(distortionModel)
		{		}
		template <typename T>
		bool operator()(const T* const* parameters, T* residuals) const {

			//Parameter order: camera, point, oriImages

			T camera[CameraBlockSize];

			for (int i = 0; i < CameraBlockSize; i++)//Size of camera block = 21
			{
				camera[i] = parameters[0][i];

			}

			T refcamera[CameraBlockSize];

			for (int i = 0; i < CameraBlockSize; i++)//Size of camera block = 21
			{
				refcamera[i] = parameters[1][i];

			}

			T point[PointBlockSize];

			for (int i = 0; i < 3; i++)//Size of point block = 3
			{
				point[i] = parameters[2][i];

			}

			T oriImage[BFrameBlockSize];

			getOrientationImageInterPolated(1, time, zeroOriTime,oriTime, interPolationOrder, parameters, oriImage);

			T hasRefCamera = T(1);

			calculateNxNyD(hasRefCamera, oriImage, point, camera, refcamera, camIndex, obsx, obsy, sigmax, sigmay, distortionModel, residuals);

			return true;

		}

		//Creating Ceres Cost Fuction
		static CollinearityEquationReferenceCostDynamicLineCameraCostFunction* Create(const double obsx, const double obsy, const double sigmax, const double sigmay, const int camIndex, double time, double *oriTime, double zeroOriTime, const int interPolationOrder, const int distortionModel){
			CollinearityEquationReferenceCostDynamicLineCamera* dynamicCost = new CollinearityEquationReferenceCostDynamicLineCamera(obsx, obsy, sigmax, sigmay, camIndex, time, oriTime, zeroOriTime, interPolationOrder, distortionModel);
			CollinearityEquationReferenceCostDynamicLineCameraCostFunction *dynamicCostFunction = new CollinearityEquationReferenceCostDynamicLineCameraCostFunction(dynamicCost);

			return (dynamicCostFunction);
		}

	};

	struct CoplanarityEquationCostDynamicLineCamera{


		typedef ceres::DynamicAutoDiffCostFunction<CoplanarityEquationCostDynamicLineCamera, 4> CoplanarityEquationCostDynamicLineCameraCostFunction;

	private:
		double obsx;
		double obsy;
		double sigmax;
		double sigmay;
		int camIndex;
		double time;
		double *oriTime;
		int interpolationOrder;
		double zeroOriTime;

		int distortionModel;

	public:
		CoplanarityEquationCostDynamicLineCamera(double obsx, double obsy, double sigmax, double sigmay, int camIndex, double time, double *oriTime, double zeroOriTime, int interpolationOrder, int distortionModel) :
			obsx(obsx), obsy(obsy), sigmax(sigmax), sigmay(sigmay), camIndex(camIndex), time(time), oriTime(oriTime), zeroOriTime(zeroOriTime), interpolationOrder(interpolationOrder), distortionModel(distortionModel)
		{		}


		template <typename T>
		bool operator()(const T* const* parameters, T* residuals) const {

			T hasRefCamera = T(0);

			T oriImage[BFrameBlockSize];

			//Parameter order: camera, point, oriImages

			T camera[CameraBlockSize];

			for (int i = 0; i < CameraBlockSize; i++)//Size of camera block = 21
			{
				camera[i] = parameters[0][i];

			}

			T pointA[PointBlockSize];

			for (int i = 0; i < PointBlockSize; i++)//Size of point block = 3
			{
				pointA[i] = parameters[1][i];

			}

			T pointB[3];

			for (int i = 0; i < PointBlockSize; i++)//Size of point block = 3
			{
				pointB[i] = parameters[2][i];

			}

			getOrientationImageInterPolated(1, time, zeroOriTime,oriTime, interpolationOrder, parameters, oriImage);


			calculateCoplanarityConstraint(hasRefCamera, camIndex, obsx, obsy, sigmax, sigmay, pointA, pointB, camera, camera, oriImage, distortionModel, residuals);


			return true;

		}

		//Creating Ceres Cost Fuction
		static CoplanarityEquationCostDynamicLineCameraCostFunction* Create(double obsx, double obsy, double sigmax, double sigmay, int camIndex, double time, double *oriTime, double zeroOriTime, int interpolationOrder, int distortionModel){
			CoplanarityEquationCostDynamicLineCamera* dynamicCost = new CoplanarityEquationCostDynamicLineCamera(obsx, obsy, sigmax, sigmay, camIndex, time, oriTime, zeroOriTime, interpolationOrder, distortionModel);
			CoplanarityEquationCostDynamicLineCameraCostFunction *dynamicCostFunction = new CoplanarityEquationCostDynamicLineCameraCostFunction(dynamicCost);

			return (dynamicCostFunction);


		}
	};


	public:
		struct CoplanarityEquationReferenceCostDynamicLineCamera{

		typedef ceres::DynamicAutoDiffCostFunction<CoplanarityEquationReferenceCostDynamicLineCamera, 4> CoplanarityEquationReferenceCostDynamicLineCameraCostFunction;

		private:
			double obsx;
			double obsy;
			double sigmax;
			double sigmay;
			int camIndex;
			double time;
			double *oriTime;
			int interpolationOrder;
			double zeroOriTime;

			int distortionModel;

		public:
			CoplanarityEquationReferenceCostDynamicLineCamera(double obsx, double obsy, double sigmax, double sigmay, int camIndex, double time, double *oriTime, double zeroOriTime, int interpolationOrder, int distortionModel) :
				obsx(obsx), obsy(obsy), sigmax(sigmax), sigmay(sigmay), camIndex(camIndex), time(time), oriTime(oriTime), zeroOriTime(zeroOriTime), interpolationOrder(interpolationOrder), distortionModel(distortionModel)
			{		}


			template <typename T>
			bool operator()(const T* const* parameters, T* residuals) const {

				T hasRefCamera = T(1);

				T oriImage[6];

				//Parameter order: camera, point, oriImages

				T camera[CameraBlockSize];

				for (int i = 0; i < CameraBlockSize; i++)//Size of camera block = 21
				{
					camera[i] = parameters[0][i];

				}

				T refcamera[CameraBlockSize];

				for (int i = 0; i < CameraBlockSize; i++)//Size of camera block = 21
				{
					refcamera[i] = parameters[1][i];

				}

				T pointA[3];

				for (int i = 0; i < PointBlockSize; i++)//Size of point block = 3
				{
					pointA[i] = parameters[2][i];

				}

				T pointB[3];

				for (int i = 0; i < PointBlockSize; i++)//Size of point block = 3
				{
					pointB[i] = parameters[3][i];

				}

				getOrientationImageInterPolated(2, time, zeroOriTime, oriTime, interpolationOrder, parameters, oriImage);

				calculateCoplanarityConstraint(hasRefCamera, camIndex, obsx, obsy, sigmax, sigmay, pointA, pointB, camera, refcamera, oriImage, distortionModel, residuals);
				
				return true;

			}

			//Creating Ceres Cost Fuction
			static CoplanarityEquationReferenceCostDynamicLineCameraCostFunction* Create(double obsx, double obsy, double sigmax, double sigmay, int camIndex, double time, double *oriTime, double zeroOriTime,int interpolationOrder, int distortionModel){
				CoplanarityEquationReferenceCostDynamicLineCamera* dynamicCost = new CoplanarityEquationReferenceCostDynamicLineCamera(obsx, obsy, sigmax, sigmay, camIndex, time, oriTime, zeroOriTime,interpolationOrder, distortionModel);
				CoplanarityEquationReferenceCostDynamicLineCameraCostFunction *dynamicCostFunction = new CoplanarityEquationReferenceCostDynamicLineCameraCostFunction(dynamicCost);

				return (dynamicCostFunction);
			}
		};





};









#endif