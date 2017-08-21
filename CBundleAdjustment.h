#ifndef UMSAT_CBUNDLEADJUSTMENT_H
#define UMSAT_CBUNDLEADJUSTMENT_H

#include "main.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/conditioned_cost_function.h"
#include "ceres/dynamic_autodiff_cost_function.h"
#include <algorithm>
#include <numeric>
#include <limits>

//Camera Block size has been decided based on the maximum number of distortion parameters currently
//being coded. It is 12 for PCI model. Hence, camera block size is calculated by
//xp, yp, c, 3 boresight, 3 leverarm, and 12 distortion parameters which totals 21.
#define CameraBlockSize 21 //Testing 

//Ground Points Block size
#define PointBlockSize 3

//GPS B-frame Location and Orientation Block Size
#define BFrameBlockSize 6

using namespace std;

class CBundleAdjustment
{
public:
	CBundleAdjustment();
	~CBundleAdjustment();

	//Linear Solver options adapted from Mehdi's code
enum LinearSolverType {
		// These solvers are for general rectangular systems formed from the
		// normal equations A'A x = A'b. They are direct solvers and do not
		// assume any special problem structure.

		// Solve the normal equations using a dense Cholesky solver; based
		// on Eigen.
		DENSE_NORMAL_CHOLESKY,

		// Solve the normal equations using a dense QR solver; based on
		// Eigen.
		DENSE_QR,

		// Solve the normal equations using a sparse cholesky solver; requires
		// SuiteSparse or CXSparse.
		SPARSE_NORMAL_CHOLESKY,

		// Specialized solvers, specific to problems with a generalized
		// bi-partitite structure.

		// Solves the reduced linear system using a dense Cholesky solver;
		// based on Eigen.
		DENSE_SCHUR,

		// Solves the reduced linear system using a sparse Cholesky solver;
		// based on CHOLMOD.
		SPARSE_SCHUR,

		// Solves the reduced linear system using Conjugate Gradients, based
		// on a new Ceres implementation.  Suitable for large scale
		// problems.
		ITERATIVE_SCHUR,

		// Conjugate gradients on the normal equations.
		CGNR
	};





public:

	void createAndSolveCeresProblem();
	void getGCPIndex(char* imagePointID, int &gcpIndex);

	void Checkforconstantparams(ceres::Problem * problem, double * X, double *stdX, const int blocksize, const int nblocks, const double scaleSigma, vector<int> &check);
	void getReferenceCameraIndex(const char* refCameraID, int &camIndex);
	void getGPSTimeTagIndex(double GPSTimeTag, int &gpsIndex);
	void CBundleAdjustment::getGPSTimeListIndex(double GPSTimeTag, double *gpsTimes, int &gpsIndex);
	void AddParameterBlocks(ceres::Problem * problem, double * X, const int blocksize, const int nblocks);
	//void createRotationMatrixFromOmegaPhiKappa(const double* angles, Eigen::Matrix3d &rotationMatrix);
	void findImagePointPosition(int order, int nOriImages, double time, double *oriTimes, int &startPosition);
	void calculateMMatrix(int lower, int order, int imgIndex, Eigen::MatrixXd &MMatrix);
	void fillCovarianceMatrix(double **covarianceMatrix, int rowBlock, int colBlock, int rowIndex, int colIndex, double *covarianceBlock);

	//To be Deleted!!
	/*void calculateNxNyD(const int imageIndex, const char* imagePointID, Eigen::Vector3d rBmt, Eigen::Matrix3d RbMt, vector<Eigen::Vector3d> cameraPositions, Eigen::Matrix3d mappingToImageRotation , double &Nx, double &Ny, double &D);
	void createRotationMatrixFromOmegaPhiKappa(Eigen::Vector3d boresight, Eigen::Matrix3d &rotationMatrix);
	void extractOmegaPhiKappaFromRotationMatrix(Eigen::Matrix3d rotationMatrix, Eigen::Vector3d &boresight);
	
	void getCameraLeverArmAndBoresight(const int imageIndex, Eigen::Matrix3d &mappingToImageRotation, vector<Eigen::Vector3d> &cameraPositions);
	void getReferenceCameraIndex(const char* refCameraID, int &camIndex);*/
};


#endif