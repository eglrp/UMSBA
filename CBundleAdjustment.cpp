#include "CBundleAdjustment.h"
#include "CBundleModel.h"
#include "Eigen/Dense"
#include "main.h"
#include <fstream>
#include <utility>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>

using namespace cv;


CBundleAdjustment::CBundleAdjustment()
{

}
CBundleAdjustment::~CBundleAdjustment()
{

}

int n_fixed_params = 0;
int n_weighted_params = 0;
int n_extra_residuals = 0;


void CBundleAdjustment::createAndSolveCeresProblem(){



	//Extracting the number of various parameters to generate the ceres problem
	int numberOfCameras = imageBlock->cameras.size();
	int numberOfPoints = imageBlock->objectPoints.size();
	int numberOfImages = imageBlock->images.size();
	int numberOfBodyFrames = imageBlock->GPSTimeList.size();
	int numberOfDistances = imageBlock->distances.size();

	cout << numberOfCameras << "< numcam .. numpts > " << numberOfPoints << endl;

	//double *mutableCamera=0, *mutablePoints=0;
	//create arrays to hold the parameters to be passed into ceres
	double *cameras = new double[CameraBlockSize*numberOfCameras];
	double *cameraStd = new double[CameraBlockSize*numberOfCameras];
	double *points = new double[PointBlockSize*numberOfPoints];
	double *pointStd = new double[PointBlockSize*numberOfPoints];
	double *bframes = new double[BFrameBlockSize*numberOfBodyFrames];
	double *bframeStd = new double[BFrameBlockSize*numberOfBodyFrames];

	double *gpsTimes = new double[numberOfBodyFrames]; //GPS Time array ascending order

	//Computing the number of initial unknowns
	int numberOfUnknowns = CameraBlockSize*numberOfCameras + PointBlockSize*numberOfPoints + BFrameBlockSize*numberOfBodyFrames;

	// 2016/05/21 Residuals
	vector<ceres::ResidualBlockId> residual_block_ids;
	ceres::ResidualBlockId block_id;

	cout << "No. of unknowns: " << numberOfUnknowns << endl;
	//Create Instance of Bundle options;
	//BundleAdjustmentOptions *options = new BundleAdjustmentOptions();


	//Create new ceres problem
	ceres::Problem *problem = new ceres::Problem;

	//Define Ceres Cost Function
	ceres::CostFunction *costFunction;

	//Normalize imagescale based on the first image that is read. 
	double stdSigma = imageBlock->images[0].measurementAccuracy;



	//Camera and GCP Index
	int camIndex, refCamIndex, gcpIndex, gpsIndex, tempGpsIndex, oriIndex;

	double Xo = 0, Yo = 0, Zo = 0;
	double Omega = 0, Phi = 0, Kappa = 0;


	//Filling Data Arrays as required by ceres

	//Filling Camera Data

	for (int i = 0; i < numberOfCameras; i++)
	{
		//xp yp c
		*(cameras + i*CameraBlockSize + 0) = imageBlock->cameras[i].xp;
		*(cameras + i*CameraBlockSize + 1) = imageBlock->cameras[i].yp;
		*(cameras + i*CameraBlockSize + 2) = imageBlock->cameras[i].c;

		//camera leverarm
		*(cameras + i*CameraBlockSize + 3) = imageBlock->cameras[i].leverarm(0);
		*(cameras + i*CameraBlockSize + 4) = imageBlock->cameras[i].leverarm(1);
		*(cameras + i*CameraBlockSize + 5) = imageBlock->cameras[i].leverarm(2);

		//camera boresight
		*(cameras + i*CameraBlockSize + 6) = imageBlock->cameras[i].boresight(0);
		*(cameras + i*CameraBlockSize + 7) = imageBlock->cameras[i].boresight(1);
		*(cameras + i*CameraBlockSize + 8) = imageBlock->cameras[i].boresight(2);


		//STD
		*(cameraStd + i*CameraBlockSize + 0) = imageBlock->cameras[i].xycCovariance(0, 0);
		*(cameraStd + i*CameraBlockSize + 1) = imageBlock->cameras[i].xycCovariance(1, 1);
		*(cameraStd + i*CameraBlockSize + 2) = imageBlock->cameras[i].xycCovariance(2, 2);

		//camera leverarm
		*(cameraStd + i*CameraBlockSize + 3) = imageBlock->cameras[i].leverarmCovariance(0, 0);
		*(cameraStd + i*CameraBlockSize + 4) = imageBlock->cameras[i].leverarmCovariance(1, 1);
		*(cameraStd + i*CameraBlockSize + 5) = imageBlock->cameras[i].leverarmCovariance(2, 2);

		//camera boresight
		*(cameraStd + i*CameraBlockSize + 6) = imageBlock->cameras[i].boresightCovariance(0, 0);
		*(cameraStd + i*CameraBlockSize + 7) = imageBlock->cameras[i].boresightCovariance(1, 1);
		*(cameraStd + i*CameraBlockSize + 8) = imageBlock->cameras[i].boresightCovariance(2, 2);


		//cout << imageBlock->cameras[imageBlock->images[i].cameraIndex].numParameters << endl;

		//camera distortion parameters
		for (int par = 0; par < 12; par++) //Testing
		{
			if (par < imageBlock->cameras[i].numParameters)
			{
				*(cameras + i*CameraBlockSize + par + 9) = imageBlock->cameras[i].distortionParameters(par);
				*(cameraStd + i*CameraBlockSize + par + 9) = imageBlock->cameras[i].distortionCovariance(par, par);
			}
			else
			{
				*(cameras + i*CameraBlockSize + par + 9) = 0;
				*(cameraStd + i*CameraBlockSize + par + 9) = sigmaFixed;
			}

		}


	}


	//Filling Body Frame Data
	for (int i = 0; i < numberOfBodyFrames; i++)
	{

		//GPS B-Frame position and orientation parameters

		*(bframes + i*BFrameBlockSize + 0) = imageBlock->GPSTimeTags[i].navigationData.position(0);//Frame Camera Testing!!!
		*(bframes + i*BFrameBlockSize + 1) = imageBlock->GPSTimeTags[i].navigationData.position(1);//Frame Camera Testing!!!
		*(bframes + i*BFrameBlockSize + 2) = imageBlock->GPSTimeTags[i].navigationData.position(2);//Frame Camera Testing!!!
		*(bframes + i*BFrameBlockSize + 3) = imageBlock->GPSTimeTags[i].navigationData.orientation(0);//Frame Camera Testing!!!
		*(bframes + i*BFrameBlockSize + 4) = imageBlock->GPSTimeTags[i].navigationData.orientation(1);//Frame Camera Testing!!!
		*(bframes + i*BFrameBlockSize + 5) = imageBlock->GPSTimeTags[i].navigationData.orientation(2);//Frame Camera Testing!!!

		*(bframeStd + i*BFrameBlockSize + 0) = imageBlock->GPSTimeTags[i].navigationData.positionCovariance(0, 0);//Frame Camera Testing!!!
		*(bframeStd + i*BFrameBlockSize + 1) = imageBlock->GPSTimeTags[i].navigationData.positionCovariance(1, 1);//Frame Camera Testing!!!
		*(bframeStd + i*BFrameBlockSize + 2) = imageBlock->GPSTimeTags[i].navigationData.positionCovariance(2, 2);//Frame Camera Testing!!!
		*(bframeStd + i*BFrameBlockSize + 3) = imageBlock->GPSTimeTags[i].navigationData.orientationCovariance(0, 0);//Frame Camera Testing!!!
		*(bframeStd + i*BFrameBlockSize + 4) = imageBlock->GPSTimeTags[i].navigationData.orientationCovariance(1, 1);//Frame Camera Testing!!!
		*(bframeStd + i*BFrameBlockSize + 5) = imageBlock->GPSTimeTags[i].navigationData.orientationCovariance(2, 2);//Frame Camera Testing!!!

	}


	//Filling Ground Control and Tie Point Data

	for (int i = 0; i < numberOfPoints; i++)
	{
		*(points + i*PointBlockSize + 0) = imageBlock->objectPoints[i].X;//Frame Camera Testing!!!
		*(points + i*PointBlockSize + 1) = imageBlock->objectPoints[i].Y;//Frame Camera Testing!!!
		*(points + i*PointBlockSize + 2) = imageBlock->objectPoints[i].Z;//Frame Camera Testing!!!

		*(pointStd + i*PointBlockSize + 0) = imageBlock->objectPoints[i].XYZCovariance(0, 0);//Frame Camera Testing!!!
		*(pointStd + i*PointBlockSize + 1) = imageBlock->objectPoints[i].XYZCovariance(1, 1);//Frame Camera Testing!!!
		*(pointStd + i*PointBlockSize + 2) = imageBlock->objectPoints[i].XYZCovariance(2, 2);//Frame Camera Testing!!!

	}

	//Sort the input GPS observations in ascending order
	int oriCount;

	for (int i = 0; i < numberOfBodyFrames; i++)
	{
		oriCount = 0;

		for (set<double>::iterator iter = imageBlock->GPSTimeList.begin(); iter != imageBlock->GPSTimeList.end(); iter++) {

			if (oriCount == i)
			{
				*(gpsTimes + i) = *iter;
			}
			oriCount++;
		}

	}




	//Add Parameter Blocks
	AddParameterBlocks(problem, cameras, CameraBlockSize, numberOfCameras);
	AddParameterBlocks(problem, points, PointBlockSize, numberOfPoints);
	AddParameterBlocks(problem, bframes, BFrameBlockSize, numberOfBodyFrames);



	double *oritime; //time array for all OI involved
	double time; //time for OI
	int interPolationOrder; //for Line Cameras
	int num_residuals, num_parameters; //number of residuals and parameters to be added for dynamic cost function
	int lowerOriImage; //Lower orientation image associated with the point
	int zeroTimeIndex; //Index associated with the zeroth index orientation image
	double zeroOriTime; //Time associated with the zeroth orientation image

	//Build Ceres Problem
	int pointNum = 0;
	for (int i = 0; i < imageBlock->images.size(); i++)
	{
		camIndex = imageBlock->images[i].cameraIndex;

		getReferenceCameraIndex(imageBlock->cameras[camIndex].refCameraID, refCamIndex);
		getGPSTimeTagIndex(imageBlock->images[i].navigationData[0].GPSTime, gpsIndex);


		//Normalizing Image Sigma
		double imageSigma = imageBlock->images[i].measurementAccuracy;
		double normSigma = (imageSigma*imageSigma) / (stdSigma*stdSigma);
		//cout << " Normal Sigma: " << normSigma << endl;

		double *camera = cameras + camIndex*CameraBlockSize;
		double *bframe = bframes + gpsIndex*BFrameBlockSize;
		double *refcamera = cameras + refCamIndex*CameraBlockSize;



		for (int j = 0; j < imageBlock->images[i].imagePoints.size(); j++)
		{
			// **** 2015/05/19 ****
			bool bSinglePoint = false;
			for (int k = 0; k < imageBlock->singlePointID.size(); k++)
			{
				if (strcmp(imageBlock->images[i].imagePoints[j].imagePointID, imageBlock->singlePointID[k]) == 0)
				{
					bSinglePoint = true;
				}
			}

			if (bSinglePoint)
				continue;

			// **** 2015/05/19 ****
			double obsx, obsy;
			double pointTime; //time associated with the image point

			//If Frame camera, use the pixel observations directly
			if (strcmp(imageBlock->cameras[camIndex].cameraType, "FRAME") == 0) //Frame Cameras
			{
				obsx = imageBlock->images[i].imagePoints[j].x;
				obsy = imageBlock->images[i].imagePoints[j].y;
			}
			//If line camera, coordinates need to be converted
			else if (strcmp(imageBlock->cameras[camIndex].cameraType, "LINE") == 0) //Line Cameras
			{

				int yMin = imageBlock->cameras[camIndex].yMin;
				int yMax = imageBlock->cameras[camIndex].yMax;
				int xMin = imageBlock->cameras[camIndex].xMin;
				int xMax = imageBlock->cameras[camIndex].xMax;
				int pixelSize = imageBlock->cameras[camIndex].pixelSize;
				int lineRate = imageBlock->cameras[camIndex].lineRate;
				int lineOffset = imageBlock->cameras[camIndex].lineOffset;

				//Observations for line camera to be passed to ceres
				obsx = lineOffset;
				obsy = (imageBlock->images[i].imagePoints[j].y - (yMax - yMin) / 2.0) * pixelSize / 1000.0;
				pointTime = imageBlock->images[i].imagePoints[j].x / lineRate;

				interPolationOrder = imageBlock->images[i].interpolationOrder;

				zeroOriTime = imageBlock->images[i].navigationData[0].GPSTime;//Zeroth orientation image time for current image
				int numOriImages = imageBlock->images[i].navigationData.size();


				getGPSTimeListIndex(zeroOriTime, gpsTimes, zeroTimeIndex);

				//!!Function to get position of image point w.r.t orientation images
				findImagePointPosition(numOriImages, interPolationOrder, pointTime, gpsTimes + zeroTimeIndex, lowerOriImage);

				//Image specific orientation images

				oritime = new double[interPolationOrder + 1];

				for (int k = 0; k < interPolationOrder + 1; k++)
				{
					oritime[k] = gpsTimes[zeroTimeIndex + lowerOriImage + k]; //orientation image times to be passed to ceres cost function

				}

				//Time w.r.t the zeroth OI
				time = pointTime - (gpsTimes[zeroTimeIndex + lowerOriImage] - zeroOriTime);

			}

			else
			{

				std::cout << "Camera type " << imageBlock->cameras[camIndex].cameraType << " not allowed in the current version of the program!!" << endl;
				break;
			}


			//Image point variance-covariance
			double sigmax = imageBlock->images[i].imagePoints[j].xyCovariance(0, 0);
			double sigmay = imageBlock->images[i].imagePoints[j].xyCovariance(1, 1);


			//scale sigmax and sigmay here;
			sigmax *= normSigma;
			sigmay *= normSigma;

			//Flags to decide point and line features
			int flagA = imageBlock->images[i].imagePoints[j].flagA;
			int flagB = imageBlock->images[i].imagePoints[j].flagB;
			int flagI = imageBlock->images[i].imagePoints[j].flagI;

			char GCPIDA[256], GCPIDB[256];


			if (strcmp(imageBlock->cameras[camIndex].cameraType, "FRAME") == 0) //Frame Cameras
			{


				//Check to see if a point belongs to a point or linear feature
				if ((flagA == 0 && flagB == 0 && flagI == 0) || (flagA == 1 && flagB == 0 && flagI == 0) || (flagA == 0 && flagB == 1 && flagI == 0))
				{
					//Now the point features are to be added to the collinearity model

					getGCPIndex(imageBlock->images[i].imagePoints[j].imagePointID, gcpIndex);

					double *point = points + gcpIndex*PointBlockSize;

					//Separate collinearity model based on if the camera is reference/non-reference
					if (camIndex == refCamIndex)
					{

						costFunction = CBundleModel::CollinearityEquationCost::Create(obsx, obsy, sigmax, sigmay, camIndex, imageBlock->cameras[camIndex].distortionModel);

						// get residual ID 2016/05/21
						block_id = problem->AddResidualBlock(costFunction, NULL, camera, point, bframe);
						residual_block_ids.push_back(block_id);

					}
					else
					{
						costFunction = CBundleModel::CollinearityEquationReferenceCost::Create(obsx, obsy, sigmax, sigmay, camIndex, imageBlock->cameras[camIndex].distortionModel);

						// get residual ID 2016/05/21
						block_id = problem->AddResidualBlock(costFunction, NULL, camera, refcamera, point, bframe);
						residual_block_ids.push_back(block_id);
					}
				}

				else if (flagA == 0 && flagB == 0 && flagI == 1)
				{
					//If it is a line point, add to coplanarity model

					strcpy(GCPIDA, imageBlock->images[i].imagePoints[j].imagePointID);
					strcat(GCPIDA, "A");
					strcpy(GCPIDB, imageBlock->images[i].imagePoints[j].imagePointID);
					strcat(GCPIDB, "B");

					int gcpIndexA, gcpIndexB;

					getGCPIndex(GCPIDA, gcpIndexA);
					getGCPIndex(GCPIDB, gcpIndexB);

					double *pointA = points + gcpIndexA*PointBlockSize;
					double *pointB = points + gcpIndexB*PointBlockSize;

					//Two separate cost functions based on if a camera is reference/non-reference
					if (camIndex == refCamIndex){

						costFunction = CBundleModel::CoplanarityEquationCost::Create(obsx, obsy, sigmax, sigmay, camIndex, imageBlock->cameras[camIndex].distortionModel);
						// get residual ID 2016/05/21
						block_id = problem->AddResidualBlock(costFunction, NULL, camera, pointA, pointB, bframe);
						residual_block_ids.push_back(block_id);
					}

					else{

						costFunction = CBundleModel::CoplanarityEquationReferenceCost::Create(obsx, obsy, sigmax, sigmay, camIndex, imageBlock->cameras[camIndex].distortionModel);
						// get residual ID 2016/05/21
						block_id = problem->AddResidualBlock(costFunction, NULL, camera, refcamera, pointA, pointB, bframe);
						residual_block_ids.push_back(block_id);
					}

				}


			}
			else if (strcmp(imageBlock->cameras[camIndex].cameraType, "LINE") == 0) //Line Cameras
			{

				//Check to see if a point belongs to a point or linear feature
				if ((flagA == 0 && flagB == 0 && flagI == 0) || (flagA == 1 && flagB == 0 && flagI == 0) || (flagA == 0 && flagB == 1 && flagI == 0))
				{
					getGCPIndex(imageBlock->images[i].imagePoints[j].imagePointID, gcpIndex);

					double *point = points + gcpIndex*PointBlockSize;

					if (camIndex == refCamIndex)
					{

						num_residuals = 2; //Collinearity equation reprojector error

						//prepare parameters!!This is the standard dynamic implementation of ceres
						vector<double*> parameter_blocks;

						CBundleModel::CollinearityEquationCostDynamicLineCamera::CollinearityEquationDynamicLineCameraCostFunction  *dynamicCostFunction = CBundleModel::CollinearityEquationCostDynamicLineCamera::Create(obsx, obsy, sigmax, sigmay, camIndex, time, oritime, zeroOriTime, interPolationOrder, imageBlock->cameras[camIndex].distortionModel);

						//set number of residuals
						dynamicCostFunction->SetNumResiduals(num_residuals);

						//Add observations to the cost function

						//Add parameter blocks and prepare parameters for the dynamic cost function
						dynamicCostFunction->AddParameterBlock(CameraBlockSize);
						parameter_blocks.push_back(camera);

						dynamicCostFunction->AddParameterBlock(PointBlockSize);
						parameter_blocks.push_back(point);

						for (int k = 0; k < interPolationOrder + 1; k++)
						{
							dynamicCostFunction->AddParameterBlock(BFrameBlockSize);

							getGPSTimeTagIndex(gpsTimes[zeroTimeIndex + lowerOriImage + k], gpsIndex);
							parameter_blocks.push_back(bframes + gpsIndex*BFrameBlockSize);
						}

						// get residual id 2016/05/21
						block_id = problem->AddResidualBlock(dynamicCostFunction, NULL, parameter_blocks);
						residual_block_ids.push_back(block_id);

					}
					else
					{
						num_residuals = 2; //Collinearity equation reprojector error

						//prepare parameters!!This is the standard dynamic implementation of ceres
						vector<double*> parameter_blocks;

						CBundleModel::CollinearityEquationReferenceCostDynamicLineCamera::CollinearityEquationReferenceCostDynamicLineCameraCostFunction  *dynamicReferenceCostFunction = CBundleModel::CollinearityEquationReferenceCostDynamicLineCamera::Create(obsx, obsy, sigmax, sigmay, camIndex, time, oritime, zeroOriTime, interPolationOrder, imageBlock->cameras[camIndex].distortionModel);

						//set number of residuals
						dynamicReferenceCostFunction->SetNumResiduals(num_residuals);

						//Add observations to the cost function

						//Add parameter blocks and prepare parameters for the dynamic cost function
						dynamicReferenceCostFunction->AddParameterBlock(CameraBlockSize);
						parameter_blocks.push_back(camera);

						dynamicReferenceCostFunction->AddParameterBlock(CameraBlockSize);
						parameter_blocks.push_back(refcamera);

						dynamicReferenceCostFunction->AddParameterBlock(PointBlockSize);
						parameter_blocks.push_back(point);

						for (int k = 0; k < interPolationOrder + 1; k++)
						{
							dynamicReferenceCostFunction->AddParameterBlock(BFrameBlockSize);

							getGPSTimeTagIndex(gpsTimes[zeroTimeIndex + lowerOriImage + k], gpsIndex);
							parameter_blocks.push_back(bframes + gpsIndex*BFrameBlockSize);
						}
						// get residual id 2016/05/21
						block_id = problem->AddResidualBlock(dynamicReferenceCostFunction, NULL, parameter_blocks);
						residual_block_ids.push_back(block_id);
					}
				}

				else if (flagA == 0 && flagB == 0 && flagI == 1)
				{

					strcpy(GCPIDA, imageBlock->images[i].imagePoints[j].imagePointID);
					strcat(GCPIDA, "A");
					strcpy(GCPIDB, imageBlock->images[i].imagePoints[j].imagePointID);
					strcat(GCPIDB, "B");

					int gcpIndexA, gcpIndexB;

					getGCPIndex(GCPIDA, gcpIndexA);
					getGCPIndex(GCPIDB, gcpIndexB);

					double *pointA = points + gcpIndexA*PointBlockSize;
					double *pointB = points + gcpIndexB*PointBlockSize;


					if (camIndex == refCamIndex){

						num_residuals = 1; //Coplanarity equation box constraint

						//prepare parameters!!This is the standard dynamic implementation of ceres
						vector<double*> parameter_blocks;


						CBundleModel::CoplanarityEquationCostDynamicLineCamera::CoplanarityEquationCostDynamicLineCameraCostFunction  *dynamicLinearCostFunction = CBundleModel::CoplanarityEquationCostDynamicLineCamera::Create(obsx, obsy, sigmax, sigmay, camIndex, time, oritime, zeroOriTime, interPolationOrder, imageBlock->cameras[camIndex].distortionModel);

						//set number of residuals
						dynamicLinearCostFunction->SetNumResiduals(num_residuals);

						//Add observations to the cost function

						//Add parameter blocks and prepare parameters for the dynamic cost function
						dynamicLinearCostFunction->AddParameterBlock(CameraBlockSize);
						parameter_blocks.push_back(camera);

						dynamicLinearCostFunction->AddParameterBlock(PointBlockSize);
						parameter_blocks.push_back(pointA);

						dynamicLinearCostFunction->AddParameterBlock(PointBlockSize);
						parameter_blocks.push_back(pointB);

						for (int k = 0; k < interPolationOrder + 1; k++)
						{
							dynamicLinearCostFunction->AddParameterBlock(BFrameBlockSize);

							getGPSTimeTagIndex(gpsTimes[zeroTimeIndex + lowerOriImage + k], gpsIndex);
							parameter_blocks.push_back(bframes + gpsIndex*BFrameBlockSize);
						}

						// get residual id 2016/05/21
						block_id = problem->AddResidualBlock(dynamicLinearCostFunction, NULL, parameter_blocks);
						residual_block_ids.push_back(block_id);
					}

					else{

						num_residuals = 1; //Coplanarity equation box constraint

						//prepare parameters!!This is the standard dynamic implementation of ceres
						vector<double*> parameter_blocks;


						CBundleModel::CoplanarityEquationCostDynamicLineCamera::CoplanarityEquationCostDynamicLineCameraCostFunction  *dynamicLinearCostFunction = CBundleModel::CoplanarityEquationCostDynamicLineCamera::Create(obsx, obsy, sigmax, sigmay, camIndex, time, oritime, zeroOriTime, interPolationOrder, imageBlock->cameras[camIndex].distortionModel);

						//set number of residuals
						dynamicLinearCostFunction->SetNumResiduals(num_residuals);

						//Add observations to the cost function

						//Add parameter blocks and prepare parameters for the dynamic cost function
						dynamicLinearCostFunction->AddParameterBlock(CameraBlockSize);
						parameter_blocks.push_back(camera);

						dynamicLinearCostFunction->AddParameterBlock(CameraBlockSize);
						parameter_blocks.push_back(refcamera);

						dynamicLinearCostFunction->AddParameterBlock(PointBlockSize);
						parameter_blocks.push_back(pointA);

						dynamicLinearCostFunction->AddParameterBlock(PointBlockSize);
						parameter_blocks.push_back(pointB);

						for (int k = 0; k < interPolationOrder + 1; k++)
						{
							dynamicLinearCostFunction->AddParameterBlock(BFrameBlockSize);

							getGPSTimeTagIndex(gpsTimes[zeroTimeIndex + lowerOriImage + k], gpsIndex);
							parameter_blocks.push_back(bframes + gpsIndex*BFrameBlockSize);
						}

						// get residual id 2016/05/21
						block_id = problem->AddResidualBlock(dynamicLinearCostFunction, NULL, parameter_blocks);
						residual_block_ids.push_back(block_id);
					}

				}
			}

			pointNum++;
		}//End Image Points Iteration



	}//End Images Iteration


	//Including distance observations to the ceres problem if distance file is available
	if (distanceAvailable)
	{
		//Distances Observation
		for (int i = 0; i < numberOfDistances; i++)//distance observations
		{
			//Scale distance variance based on benchmark sigma
			double distSigma = imageBlock->distances[i].sigma / (stdSigma*stdSigma);

			costFunction = CBundleModel::DistanceCost::Create(imageBlock->distances[i].distance, distSigma);

			int gcpIndex1, gcpIndex2;

			//Get the index of GCP's associated with the distance observation
			getGCPIndex(imageBlock->distances[i].startID, gcpIndex1);
			getGCPIndex(imageBlock->distances[i].endID, gcpIndex2);

			// If enabled use Huber's loss function.
			//  ceres::LossFunction* loss_function = options->use_loss_function ? new ceres::HuberLoss(1.0) : NULL;


			double* point1 = points + gcpIndex1*PointBlockSize;
			double* point2 = points + gcpIndex2*PointBlockSize;


			// get residual id 2016/05/21
			block_id = problem->AddResidualBlock(costFunction, NULL, point1, point2);
			residual_block_ids.push_back(block_id);
		}

		///Distances
	}

	//Including Patch observations to the problem

	if (patchAvailable)
	{
		for (int i = 0; i < imageBlock->patches.size(); i++)
		{
			int gcpIndexA, gcpIndexB, gcpIndexC;

			//Get the index of GCP/tie-points associated with the patch observation
			getGCPIndex(imageBlock->patches[i].patchA, gcpIndexA);
			getGCPIndex(imageBlock->patches[i].patchB, gcpIndexB);
			getGCPIndex(imageBlock->patches[i].patchC, gcpIndexC);



			//Eliminate Patches if any of the GCP's associated with the patch do not exist;

			if (gcpIndexA != -1 && gcpIndexB != -1 && gcpIndexC != -1)
			{
				double *pointA = points + gcpIndexA*PointBlockSize;
				double *pointB = points + gcpIndexB*PointBlockSize;
				double *pointC = points + gcpIndexC*PointBlockSize;


				double obsX, obsY, obsZ, sigmaX, sigmaY, sigmaZ;

				//Scale patch variances
				sigmaX = imageBlock->patches[i].patchCovariance(0, 0) / (stdSigma*stdSigma);
				sigmaY = imageBlock->patches[i].patchCovariance(1, 1) / (stdSigma*stdSigma);
				sigmaZ = imageBlock->patches[i].patchCovariance(2, 2) / (stdSigma*stdSigma);

				for (int j = 0; j < imageBlock->patches[i].patchPoints.size(); j++)
				{
					obsX = imageBlock->patches[i].patchPoints[j].X;
					obsY = imageBlock->patches[i].patchPoints[j].Y;
					obsZ = imageBlock->patches[i].patchPoints[j].Z;

					costFunction = CBundleModel::PatchCoordinateCost::Create(obsX, obsY, obsZ, sigmaX, sigmaY, sigmaZ); //Patch Constraint

					// get residual id 2016/05/21
					block_id = problem->AddResidualBlock(costFunction, NULL, pointA, pointB, pointC);
					residual_block_ids.push_back(block_id);
				}
			}
		}
	}

	// vectors to identify fixed and weighted parameters
	vector<int> camParaCheck;
	vector<int> ptsParaCheck;
	vector<int> bfrParaCheck;

	//Check for constant and weighted parameters done here
	Checkforconstantparams(problem, cameras, cameraStd, CameraBlockSize, numberOfCameras, stdSigma, camParaCheck);
	Checkforconstantparams(problem, points, pointStd, PointBlockSize, numberOfPoints, stdSigma, ptsParaCheck);
	Checkforconstantparams(problem, bframes, bframeStd, BFrameBlockSize, numberOfBodyFrames, stdSigma, bfrParaCheck);

	// output the file
	ofstream fod_pchk("UMSAT_Fixed&Weighted_Paramters_OUT.txt");
	fod_pchk << "Camera Parameters:" << endl << endl;
	for (int i = 0; i < numberOfCameras*CameraBlockSize; i++)
	{
		if (camParaCheck[i] != 2)
		{
			int cameraIndex = i / CameraBlockSize;
			int numDistPara = imageBlock->cameras[cameraIndex].numParameters;
			fod_pchk << "CameraID: " << imageBlock->cameras[cameraIndex].cameraID << "\t\t";
			if (i%CameraBlockSize == 0)
				fod_pchk << "xp\t\t";
			else if (i%CameraBlockSize == 1)
				fod_pchk << "yp\t\t";
			else if (i%CameraBlockSize == 2)
				fod_pchk << "c\t\t";
			else if (i%CameraBlockSize == 3)
				fod_pchk << "X\t\t";
			else if (i%CameraBlockSize == 4)
				fod_pchk << "Y\t\t";
			else if (i%CameraBlockSize == 5)
				fod_pchk << "Z\t\t";
			else if (i%CameraBlockSize == 6)
				fod_pchk << "Omega\t\t";
			else if (i%CameraBlockSize == 7)
				fod_pchk << "Phi\t\t";
			else if (i%CameraBlockSize == 8)
				fod_pchk << "Kappa\t\t";
			else
			{
				if (i%CameraBlockSize < 9 + numDistPara)
				fod_pchk << "Distortion Parameter " << i%CameraBlockSize - 8 << "\t\t";
			}
			if (camParaCheck[i] == 0)
				fod_pchk << "Fixed" << endl;
			else
				fod_pchk << "Weighted" << endl;
		}
	}

	fod_pchk << endl << "Object Points" << endl;
	for (int i = 0; i < numberOfPoints*PointBlockSize; i++)
	{
		if (ptsParaCheck[i] != 2)
		{
			int pointIndex = i / PointBlockSize;
			fod_pchk << "PointID: " << imageBlock->objectPoints[pointIndex].GCPID << "\t\t";
			if (i%PointBlockSize == 0)
				fod_pchk << "X\t\t";
			if (i%PointBlockSize == 1)
				fod_pchk << "Y\t\t";
			if (i%PointBlockSize == 2)
				fod_pchk << "Z\t\t";
			if (ptsParaCheck[i] == 0)
				fod_pchk << "Fixed" << endl;
			else
				fod_pchk << "Weighted" << endl;
		}
	}

	fod_pchk << endl << "GPS Data" << endl;
	for (int i = 0; i < numberOfBodyFrames*BFrameBlockSize; i++)
	{
		if (bfrParaCheck[i] != 2)
		{
			int timetagIndex = i / BFrameBlockSize;
			fod_pchk << "GPSTime " << imageBlock->GPSTimeTags[timetagIndex].GPSTime << "\t";
			if (i%BFrameBlockSize == 0)
				fod_pchk << "X\t\t";
			if (i%BFrameBlockSize == 1)
				fod_pchk << "Y\t\t";
			if (i%BFrameBlockSize == 2)
				fod_pchk << "Z\t\t";
			if (i%BFrameBlockSize == 3)
				fod_pchk << "Omega\t\t";
			if (i%BFrameBlockSize == 4)
				fod_pchk << "Phi\t\t";
			if (i%BFrameBlockSize == 5)
				fod_pchk << "Kappa\t\t";
			if (bfrParaCheck[i] == 0)
				fod_pchk << "Fixed" << endl;
			else
				fod_pchk << "Weighted" << endl;
		}
	}

	fod_pchk.close();
	//Create Ceres solver
	ceres::Solver::Options solverOptions;

	//Set solver options
	solverOptions.max_num_iterations = iterationNum;

	//!!!By default, the strategy type used is levenberg. Currently set to Dogleg
	solverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
	solverOptions.parameter_tolerance = sigmaStop;
	solverOptions.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	solverOptions.minimizer_progress_to_stdout = true;
	solverOptions.update_state_every_iteration = true;
	solverOptions.num_threads = 32;

	solverOptions.use_nonmonotonic_steps = true;
	solverOptions.max_num_consecutive_invalid_steps = 20;

	//Create solver summary to see progress during the solving process
	ceres::Solver::Summary summary;

	ceres::Problem::EvaluateOptions evalop;

	// residual blocks output prepare 2016/05/21
	evalop.residual_blocks = residual_block_ids;

	evalop.num_threads = solverOptions.num_threads;
	double initialCost;
	//vector to store Residual Blocks
	std::vector<double> resBlocks;


	problem->Evaluate(evalop, &initialCost, &resBlocks, NULL, 0);
//	double bframeSigma[BFrameBlockSize*BFrameBlockSize];
//	double cameraSigma[CameraBlockSize*CameraBlockSize];
//	double pointSigma[PointBlockSize*PointBlockSize];



	cout << "initial cost: " << initialCost << endl;

	//Now that all the functional models are created, and solver settings have been set; solve the problem
	ceres::Solve(solverOptions, problem, &summary);

	double finalCost;
	problem->Evaluate(evalop, &finalCost, &resBlocks, NULL, 0);

	cout << summary.FullReport() << "\n";

	///////////////Sigma0 computation here //////////////////

	double aposteriorivariance = (summary.final_cost * 2) / (summary.num_residuals - n_extra_residuals - summary.num_effective_parameters_reduced);
	double initvariance = (initialCost * 2) / (summary.num_residuals - n_extra_residuals - summary.num_effective_parameters_reduced);
	double aposterioristd = sqrt(aposteriorivariance);
	double initstd = sqrt(initvariance);
	std::cout << "STD " << initstd << " ---> " << aposterioristd << "\n";

	////////////////////////////////////////////////////////

	//Below steps to compute solver summary. Has been disabled for now; since it takes long to perform this computation

	//Variance - CoVariance Computation
	if (completeOutput)
	{
		double **covarianceMatrix = new double*[numberOfUnknowns];
		for (int i = 0; i < numberOfUnknowns; i++)
		{
			covarianceMatrix[i] = new double[numberOfUnknowns];
			memset(covarianceMatrix[i], 0, numberOfUnknowns*sizeof(covarianceMatrix[i]));
		}

		ceres::Covariance::Options covarianceOptions;
        covarianceOptions.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::EIGEN_SPARSE;
		covarianceOptions.algorithm_type = ceres::CovarianceAlgorithmType::SPARSE_QR;
		//Setting adapted from Mehdi's code
		covarianceOptions.min_reciprocal_condition_number = 1e-200;
		covarianceOptions.num_threads = solverOptions.num_threads;


		ceres::Covariance covariance(covarianceOptions);
		vector<pair<const double*, const double*>> covarianceBlocks;

		//double *bframeCov = new double[numberOfBodyFrames*numberOfBodyFrames*BFrameBlockSize*BFrameBlockSize];
		//covarianceBlocks.push_back(make_pair(bframes, bframes));
		//CHECK(covariance.Compute(covarianceBlocks, problem));
		//covariance.GetCovarianceBlock(bframes, bframes, bframeCov);

		// prepare the covariance blocks for computation
		for (int i = 0; i < numberOfBodyFrames; i++)
		{
			for (int j = i; j < numberOfBodyFrames; j++)
				covarianceBlocks.push_back(make_pair(bframes + i*BFrameBlockSize, bframes + j*BFrameBlockSize));
			for (int j = 0; j < numberOfCameras; j++)
				covarianceBlocks.push_back(make_pair(bframes + i*BFrameBlockSize, cameras + j*CameraBlockSize));
			for (int j = 0; j < numberOfPoints; j++)
				covarianceBlocks.push_back(make_pair(bframes + i*BFrameBlockSize, points + j*PointBlockSize));
		}

		for (int i = 0; i < numberOfCameras; i++)
		{
			for (int j = i; j < numberOfCameras; j++)
				covarianceBlocks.push_back(make_pair(cameras + i*CameraBlockSize, cameras + j*CameraBlockSize));
			for (int j = 0; j < numberOfPoints; j++)
				covarianceBlocks.push_back(make_pair(cameras + i*CameraBlockSize, points + j*PointBlockSize));
		}

		for (int i = 0; i < numberOfPoints; i++)
		{
			for (int j = i; j < numberOfPoints; j++)
				covarianceBlocks.push_back(make_pair(points + i*PointBlockSize, points + j*PointBlockSize));
		}


		CHECK(covariance.Compute(covarianceBlocks, problem));

		double covBFrameBFrame[BFrameBlockSize*BFrameBlockSize];
		double covBFrameCamera[BFrameBlockSize*CameraBlockSize];
		double covBFramePoint[BFrameBlockSize*PointBlockSize];
		double covCameraCamera[CameraBlockSize*CameraBlockSize];
		double covCameraPoint[CameraBlockSize*PointBlockSize];
		double covPointPoint[PointBlockSize*PointBlockSize];

		// fill the covariance matrix
		for (int i = 0; i < numberOfBodyFrames; i++)
		{
			for (int j = i; j < numberOfBodyFrames; j++)
			{
				covariance.GetCovarianceBlock(bframes + i*BFrameBlockSize, bframes + j*BFrameBlockSize, covBFrameBFrame);
				fillCovarianceMatrix(covarianceMatrix, BFrameBlockSize, BFrameBlockSize, i*BFrameBlockSize, j*BFrameBlockSize, covBFrameBFrame);
			}
			for (int j = 0; j < numberOfCameras; j++)
			{
				covariance.GetCovarianceBlock(bframes + i*BFrameBlockSize, cameras + j*CameraBlockSize, covBFrameCamera);
				fillCovarianceMatrix(covarianceMatrix, BFrameBlockSize, CameraBlockSize, i*BFrameBlockSize, j*CameraBlockSize + numberOfBodyFrames*BFrameBlockSize, covBFrameCamera);
			}
			for (int j = 0; j < numberOfPoints; j++)
			{
				covariance.GetCovarianceBlock(bframes + i*BFrameBlockSize, points + j*PointBlockSize, covBFramePoint);
				fillCovarianceMatrix(covarianceMatrix, BFrameBlockSize, PointBlockSize, i*BFrameBlockSize, j*PointBlockSize + numberOfBodyFrames*BFrameBlockSize + numberOfCameras*CameraBlockSize, covBFramePoint);
			}
		}

		for (int i = 0; i < numberOfCameras; i++)
		{
			for (int j = i; j < numberOfCameras; j++)
			{
				covariance.GetCovarianceBlock(cameras + i*CameraBlockSize, cameras + j*CameraBlockSize, covCameraCamera);
				fillCovarianceMatrix(covarianceMatrix, CameraBlockSize, CameraBlockSize, i*CameraBlockSize + numberOfBodyFrames*BFrameBlockSize, j*CameraBlockSize + numberOfBodyFrames*BFrameBlockSize, covCameraCamera);
			}
			for (int j = 0; j < numberOfPoints; j++)
			{
				covariance.GetCovarianceBlock(cameras + i*CameraBlockSize, points + j*PointBlockSize, covCameraPoint);
				fillCovarianceMatrix(covarianceMatrix, CameraBlockSize, PointBlockSize, i*CameraBlockSize + numberOfBodyFrames*BFrameBlockSize, j*PointBlockSize + numberOfBodyFrames*BFrameBlockSize + numberOfCameras*CameraBlockSize, covCameraPoint);
			}
		}

		for (int i = 0; i < numberOfPoints; i++)
		{
			for (int j = i; j < numberOfPoints; j++)
			{
				covariance.GetCovarianceBlock(points + i*PointBlockSize, points + j*PointBlockSize, covPointPoint);
				fillCovarianceMatrix(covarianceMatrix, PointBlockSize, PointBlockSize, i*PointBlockSize + numberOfBodyFrames*BFrameBlockSize + numberOfCameras*CameraBlockSize, j*PointBlockSize + numberOfBodyFrames*BFrameBlockSize + numberOfCameras*CameraBlockSize, covPointPoint);
			}
		}

		// output the covariance matrix

		ofstream fout_cov("UMSAT_Covariance_OUT.txt");
		for (int i = 0; i < numberOfUnknowns; i++)
		{
			for (int j = 0; j < numberOfUnknowns; j++)
			{
				fout_cov << sqrt(fabs(covarianceMatrix[i][j] * aposteriorivariance)) << "\t";
			}
			fout_cov << endl;
		}

		fout_cov.close();

		// output the sigmas
		ofstream fout_var("UMSAT_Variance_OUT.txt");
		fout_var << "GPS Timetag\t\t" << "X\t\t" << "Y\t\t\t" << "Z\t\t" << "Omega\t\t" << "Phi\t\t" << "Kappa\t\t" << endl;
		for (int i = 0; i < numberOfBodyFrames; i++)
		{
			fout_var << imageBlock->GPSTimeTags[i].GPSTime << "\t\t";
			for (int j = 0; j < BFrameBlockSize; j++)
			{
				fout_var << sqrt(fabs(covarianceMatrix[i*BFrameBlockSize + j][i*BFrameBlockSize + j] * aposteriorivariance)) << "\t";
			}
			fout_var << endl;
		}
		
		for (int i = 0; i < numberOfCameras; i++)
		{
			fout_var << "CameraID\t" << imageBlock->cameras[i].cameraID << endl;
			for (int j = 0; j < CameraBlockSize; j++)
			{
				if (j == 0)
					fout_var << "xp:\t";
				else if (j == 1)	   
					fout_var << "yp:\t";
				else if (j == 2)	   
					fout_var << "c:\t";
				else if (j == 3)	   
					fout_var << "X:\t";
				else if (j == 4)	   
					fout_var << "Y:\t";
				else if (j == 5)
					fout_var << "Z:\t";
				else if (j == 6)
					fout_var << "Omega:\t";
				else if (j == 7)
					fout_var << "Phi:\t";
				else if (j == 8)
					fout_var << "Kappa:\t";
				else
					fout_var << "Distortion Parameter:\t";
				fout_var << sqrt(fabs(covarianceMatrix[i*CameraBlockSize + j + numberOfBodyFrames*BFrameBlockSize][i*CameraBlockSize + j + numberOfBodyFrames*BFrameBlockSize] * aposteriorivariance)) << endl;
			}
			fout_var << endl;
		}

		fout_var << "PointID\t\t" << "X\t\t" << "Y\t\t\t" << "Z\t\t" << endl;
		for (int i = 0; i < numberOfPoints; i++)
		{
			fout_var << imageBlock->objectPoints[i].GCPID << "\t\t";
			for (int j = 0; j < PointBlockSize; j++)
			{
				fout_var << sqrt(fabs(covarianceMatrix[i*PointBlockSize + j + numberOfBodyFrames*BFrameBlockSize + numberOfCameras*CameraBlockSize][i*PointBlockSize + j + numberOfBodyFrames*BFrameBlockSize + numberOfCameras*CameraBlockSize] * aposteriorivariance)) << "\t";
			}
			fout_var << endl;
		}

		fout_var.close();
		ofstream corMat("correlation.txt");
		for (int i = 0; i < numberOfUnknowns; i++)
		{
			for (int j = 0; j < numberOfUnknowns; j++)
			{
				if (i > j)
				{
					double correlationCoef = covarianceMatrix[j][i] / (sqrt(covarianceMatrix[i][i] * covarianceMatrix[j][j]));
					corMat << correlationCoef << "\t";
				}
				else if (i == j)
					corMat << 1 << "\t";
				else
				{
					double correlationCoef = covarianceMatrix[i][j] / (sqrt(covarianceMatrix[i][i] * covarianceMatrix[j][j]));
					corMat << correlationCoef << "\t";
				}
			}
			corMat << endl;
		}

		corMat.close();

		// compute the correlations & draw the correlation map
		ofstream fout_corr("UMSAT_Correlation_OUT.txt");
		const char* corrImgDir = "UMSAT_Correlation_Image.jpg";
		Mat corrImg(numberOfUnknowns, numberOfUnknowns, CV_8UC1);

		double correlationThreshold = 0.99;
		double sum = 0.0;
		for (int i = 0; i < numberOfUnknowns; i++)
		{
			for (int j = i; j < numberOfUnknowns; j++)
			{
				if (i != j)
				{
					double correlationCoef = covarianceMatrix[i][j] / (sqrt(covarianceMatrix[i][i] * covarianceMatrix[j][j]));
					corrImg.at<uchar>(i, j) = 255 - uchar(fabs(correlationCoef * 255));
					//int temp = corrImg.at<uchar>(i, j);
					//sum += correlationCoef;
					if (fabs(correlationCoef) >= correlationThreshold)
					{
						fout_corr << i << "\t" << j << "\t" << correlationCoef << "\t";
						if (i < numberOfBodyFrames*BFrameBlockSize)
						{
							int timetagIndex = i / BFrameBlockSize;
							fout_corr << "GPSTime " << imageBlock->GPSTimeTags[timetagIndex].GPSTime << "\t";
							if (i%BFrameBlockSize == 0)
								fout_corr << "X\t\t";
							if (i%BFrameBlockSize == 1)
								fout_corr << "Y\t\t";
							if (i%BFrameBlockSize == 2)
								fout_corr << "Z\t\t";
							if (i%BFrameBlockSize == 3)
								fout_corr << "Omega\t\t";
							if (i%BFrameBlockSize == 4)
								fout_corr << "Phi\t\t";
							if (i%BFrameBlockSize == 5)
								fout_corr << "Kappa\t\t";
						}
						else if (i < (numberOfCameras*CameraBlockSize + numberOfBodyFrames*BFrameBlockSize) && i >= numberOfBodyFrames*BFrameBlockSize)
						{
							int cameraIndex = (i - (numberOfBodyFrames*BFrameBlockSize)) / CameraBlockSize;
							fout_corr << "CameraID" << imageBlock->cameras[cameraIndex].cameraID << "\t";
							if (i%CameraBlockSize == 0)
								fout_corr << "xp\t\t";
							else if (i%CameraBlockSize == 1)
								fout_corr << "yp\t\t";
							else if (i%CameraBlockSize == 2)
								fout_corr << "c\t\t";
							else if (i%CameraBlockSize == 3)
								fout_corr << "X\t\t";
							else if (i%CameraBlockSize == 4)
								fout_corr << "Y\t\t";
							else if (i%CameraBlockSize == 5)
								fout_corr << "Z\t\t";
							else if (i%CameraBlockSize == 6)
								fout_corr << "Omega\t\t";
							else if (i%CameraBlockSize == 7)
								fout_corr << "Phi\t\t";
							else if (i%CameraBlockSize == 8)
								fout_corr << "Kappa\t\t";
							else
								fout_corr << "Distortion Parameter " << i%CameraBlockSize - 8 << "\t\t";
						}
						else
						{
							int pointIndex = (i - (numberOfCameras*CameraBlockSize + numberOfBodyFrames*BFrameBlockSize)) / PointBlockSize;
							fout_corr << "PointID " << imageBlock->objectPoints[pointIndex].GCPID << "\t";
							if (i%PointBlockSize == 0)
								fout_corr << "X\t\t";
							if (i%PointBlockSize == 1)
								fout_corr << "Y\t\t";
							if (i%PointBlockSize == 2)
								fout_corr << "Z\t\t";
						}

						if (j < numberOfBodyFrames*BFrameBlockSize)
						{
							int timetagIndex = j / BFrameBlockSize;
							fout_corr << "GPSTime " << imageBlock->GPSTimeTags[timetagIndex].GPSTime << "\t";
							if (j%BFrameBlockSize == 0)
								fout_corr << "X" << endl;
							if (j%BFrameBlockSize == 1)
								fout_corr << "Y" << endl;
							if (j%BFrameBlockSize == 2)
								fout_corr << "Z" << endl;
							if (j%BFrameBlockSize == 3)
								fout_corr << "Omega" << endl;
							if (j%BFrameBlockSize == 4)
								fout_corr << "Phi" << endl;
							if (j%BFrameBlockSize == 5)
								fout_corr << "Kappa" << endl;
						}
						else if (j < (numberOfCameras*CameraBlockSize + numberOfBodyFrames*BFrameBlockSize) && j >= numberOfBodyFrames*BFrameBlockSize)
						{
							int cameraIndex = (j - (numberOfBodyFrames*BFrameBlockSize)) / CameraBlockSize;
							fout_corr << "CameraID" << imageBlock->cameras[cameraIndex].cameraID << "\t";
							if (j%CameraBlockSize == 0)
								fout_corr << "xp" << endl;
							else if (j%CameraBlockSize == 1)
								fout_corr << "yp" << endl;
							else if (j%CameraBlockSize == 2)
								fout_corr << "c" << endl;
							else if (j%CameraBlockSize == 3)
								fout_corr << "X" << endl;
							else if (j%CameraBlockSize == 4)
								fout_corr << "Y" << endl;
							else if (j%CameraBlockSize == 5)
								fout_corr << "Z" << endl;
							else if (j%CameraBlockSize == 6)
								fout_corr << "Omega" << endl;
							else if (j%CameraBlockSize == 7)
								fout_corr << "Phi" << endl;
							else if (j%CameraBlockSize == 8)
								fout_corr << "Kappa" << endl;
							else
								fout_corr << "Distortion Parameter " << j%CameraBlockSize - 8 << endl;
						}
						else
						{
							int pointIndex = (j - (numberOfCameras*CameraBlockSize + numberOfBodyFrames*BFrameBlockSize)) / PointBlockSize;
							fout_corr << "PointID " << imageBlock->objectPoints[pointIndex].GCPID << "\t";
							if (j%PointBlockSize == 0)
								fout_corr << "X" << endl;
							if (j%PointBlockSize == 1)
								fout_corr << "Y" << endl;
							if (j%PointBlockSize == 2)
								fout_corr << "Z" << endl;
						}
					}
				}
				else
				{
					double correlationCoef = 1.0;
					corrImg.at<uchar>(i, j) = 255 - uchar(fabs(correlationCoef * 255));
				}
			}
		}

		for (int i = 0; i < numberOfUnknowns; i++)
			for (int j = 0; j < i; j++)
				corrImg.at<uchar>(i, j) = corrImg.at<uchar>(j, i);

		// write img
		imwrite(corrImgDir, corrImg);

		// release the memory
		for (int i = 0; i < numberOfUnknowns; i++)
			delete covarianceMatrix[i];
		delete[] covarianceMatrix;

		//for (int i = 0; i < numberOfBodyFrames; i++)
		//	covarianceBlocks.push_back(make_pair(bframes + i*BFrameBlockSize, bframes + i*BFrameBlockSize));
		//for (int i = 0; i < numberOfCameras; i++)
		//	covarianceBlocks.push_back(make_pair(cameras + i*CameraBlockSize, cameras + i*CameraBlockSize));
		//for (int i = 0; i < numberOfPoints; i++)
		//	covarianceBlocks.push_back(make_pair(points + i*PointBlockSize, points + i*PointBlockSize));

		//cout << "1" << endl;

		//CHECK(covariance.Compute(covarianceBlocks, problem));

		//for (int i = 0; i < numberOfBodyFrames; i++)
		//{
		//	covariance.GetCovarianceBlock(bframes + i*BFrameBlockSize, bframes + i*BFrameBlockSize,bframeSigma);
		//	for (int j = 0; j < BFrameBlockSize; j++)
		//	{
		//		double variance = bframeSigma[j*BFrameBlockSize + j];
		//		bframeStd[i*BFrameBlockSize + j] = aposterioristd*sqrt(variance);
		//	}
		//}
		//for (int i = 0; i < numberOfCameras; i++)
		//{
		//	//cout << "iter " << i << "\n" << endl;
		//	covariance.GetCovarianceBlock(cameras + i*CameraBlockSize, cameras + i*CameraBlockSize, cameraSigma);
		//	for (int j = 0; j < CameraBlockSize; j++)
		//	{
		//		//cout << "computing\n" << endl;
		//		double variance = cameraSigma[j*CameraBlockSize + j];
		//		cameraStd[i*CameraBlockSize + j] = aposterioristd*sqrt(variance);
		//	}
		//}
		//for (int i = 0; i < numberOfPoints; i++)
		//{
		//	covariance.GetCovarianceBlock(points + i*PointBlockSize, points + i*PointBlockSize, pointSigma);
		//	for (int j = 0; j < PointBlockSize; j++)
		//	{
		//		double variance = pointSigma[j*PointBlockSize + j];
		//		pointStd[i*PointBlockSize + j] = aposterioristd*sqrt(variance);
		//	}
		//}

		covarianceBlocks.clear();
	}
	
	////////////////////////////////Output to files here ///////////////////////////
	

	cout << "\n\n CAM OUT \n\n" << endl;
	ofstream fout("UMSAT_CAM_OUT.txt");
	ofstream fout_obs_res("UMSAT_PsedoObs_Residual_OUT.txt");

	if (fout.is_open())
	{

		fout << "\nUMSAT Output - Camera\n\n";
		
		//fout << "sigmaxp sigmayp sigmac:\n";

		//fout << "delx dely delz (leverarm):\n";
		//fout << "sigmadelx sigmadely sigmadelz\n";
		//fout << "om phi kp (boresight):\n";
		//fout << "sigmaom sigmaphi sigmakp:\n"; 

		//fout << "distortion parameters:\n";
		//fout << "sigma dist parameters:\n\n";

		//Output for debuging!! 
		for (int i = 0; i < numberOfCameras; i++)
		{
			fout.precision(6);

			fout << "!cameraID \t type \t xp \t yp \t c\n";
			fout << imageBlock->cameras[i].cameraID << "\t" << imageBlock->cameras[i].cameraType << "\t" << *(cameras + i*CameraBlockSize + 0) << "\t" << *(cameras + i*CameraBlockSize + 1) << "\t" << *(cameras + i*CameraBlockSize + 2) << "\n\n";
			
			// residual
			fout_obs_res << "cameraID:\t" << imageBlock->cameras[i].cameraID << endl;
			fout_obs_res << "xp:\t" << imageBlock->cameras[i].xp - *(cameras + i*CameraBlockSize + 0) << "\typ:\t" << imageBlock->cameras[i].yp - *(cameras + i*CameraBlockSize + 1) << "\tc:\t" << imageBlock->cameras[i].c - *(cameras + i*CameraBlockSize + 2) << endl;

			/*fout << "!		dispersion of xp,yp,c";
			fout << *(cameraStd + i*CameraBlockSize + 0) << " " << "0.0 0.0\n";
			fout << "0.0 " << *(cameraStd + i*CameraBlockSize + 1) << " 0.0\n";
			fout << "0.0 0.0 " << *(cameraStd + i*CameraBlockSize + 2) << "\n";*/

			fout << "! no distortion and array elements\n";
			fout << imageBlock->cameras[i].numParameters <<"\n\n";
			fout << *(cameras + i*CameraBlockSize + 9) << " " << *(cameras + i*CameraBlockSize + 10) << " " << *(cameras + i*CameraBlockSize + 11) << " " << *(cameras + i*CameraBlockSize + 12) << " " << *(cameras + i*CameraBlockSize + 13) << " " << *(cameras + i*CameraBlockSize + 14) << " " << *(cameras + i*CameraBlockSize + 15) << " " << *(cameras + i*CameraBlockSize + 16) << " " << *(cameras + i*CameraBlockSize + 17) << " " << *(cameras + i*CameraBlockSize + 18) << " " << *(cameras + i*CameraBlockSize + 19) << " " << *(cameras + i*CameraBlockSize + 20) << "\n\n";
			
			// residual
			fout_obs_res << "distortion parameters" << endl;
			for (int j = 0; j < imageBlock->cameras[i].numParameters; j++)
			{
				fout_obs_res << imageBlock->cameras[i].distortionParameters[j] - *(cameras + i*CameraBlockSize + 9 + j) << "\t";
			}
			fout_obs_res << endl;

			fout << "! Reference Camera\n";
			fout << imageBlock->cameras[i].refCameraID << "\n\n";

			fout << "!Gps offset: dx, dy, dz and dispersion \n";
			fout << *(cameras + i*CameraBlockSize + 3) << " " << *(cameras + i*CameraBlockSize + 4) << " " << *(cameras + i*CameraBlockSize + 5) << "\n\n";

			// residual
			fout_obs_res << "leverarm" << endl;
			fout_obs_res << imageBlock->cameras[i].leverarm[0] - *(cameras + i*CameraBlockSize + 3) << "\t" << imageBlock->cameras[i].leverarm[1] - *(cameras + i*CameraBlockSize + 4) << "\t" << imageBlock->cameras[i].leverarm[2] - *(cameras + i*CameraBlockSize + 5) << endl;
	
			fout << "!Boresight Angles (Omega,Phi,Kappa) \n";
			fout << *(cameras + i*CameraBlockSize + 6) << " " << *(cameras + i*CameraBlockSize + 7) << " " << *(cameras + i*CameraBlockSize + 8) << "\n\n";

			// residual
			fout_obs_res << "boresight" << endl;
			fout_obs_res << imageBlock->cameras[i].boresight[0] - *(cameras + i*CameraBlockSize + 6) << "\t" << imageBlock->cameras[i].boresight[1] - *(cameras + i*CameraBlockSize + 7) << "\t" << imageBlock->cameras[i].boresight[2] - *(cameras + i*CameraBlockSize + 8) << endl;
			
			//fout << *(cameraStd + i*CameraBlockSize + 3) << " " << *(cameraStd + i*CameraBlockSize + 4) << " " << *(cameraStd + i*CameraBlockSize + 5) << "\n";
			//fout << *(cameraStd + i*CameraBlockSize + 6) << " " << *(cameraStd + i*CameraBlockSize + 7) << " " << *(cameraStd + i*CameraBlockSize + 8) << "\n";			
			//fout <<  *(cameraStd + i*CameraBlockSize + 9) << " " << *(cameraStd + i*CameraBlockSize + 10) << " " << *(cameraStd + i*CameraBlockSize + 11) << " " << *(cameraStd + i*CameraBlockSize + 12) << " " << *(cameraStd + i*CameraBlockSize + 13) << " " << *(cameraStd + i*CameraBlockSize + 14) << " " << *(cameraStd + i*CameraBlockSize + 15) << " " << *(cameraStd + i*CameraBlockSize + 16) << " " << *(cameraStd + i*CameraBlockSize + 17) << " " << *(cameraStd + i*CameraBlockSize + 18) << " " << *(cameraStd + i*CameraBlockSize + 19) << " " << *(cameraStd + i*CameraBlockSize + 20) << "\n";

		}
	}
	fout.close();

	cout << "\n\n GPS OUT \n\n" << endl;
	ofstream fout2("UMSAT_GPS_OUT.txt");

	if (fout2.is_open())
	{
		fout2 << "\nUMSAT Output - GPS \n\n";
		fout_obs_res << endl << "GPS info" << endl;

		for (int i = 0; i < numberOfBodyFrames; i++)
		{
		
			//GPS B-Frame position and orientation parameters
			fout2.precision(6);
			fout2 << imageBlock->GPSTimeTags[i].GPSTime << " " << *(bframes + i*BFrameBlockSize + 0) << " " << *(bframes + i*BFrameBlockSize + 1) << " " << *(bframes + i*BFrameBlockSize + 2) << "\n";
			//fout2 << *(bframeStd + i*BFrameBlockSize + 0) << " " << *(bframeStd + i*BFrameBlockSize + 1) << " " << *(bframeStd + i*BFrameBlockSize + 2) << "\n\n";
			fout2 << *(bframes + i*BFrameBlockSize + 3) << " " << *(bframes + i*BFrameBlockSize + 4) << " " << *(bframes + i*BFrameBlockSize + 5) << "\n";
			//fout2 <<*(bframeStd + i*BFrameBlockSize + 3) << " " << *(bframeStd + i*BFrameBlockSize + 4) << " " << *(bframeStd + i*BFrameBlockSize + 5) << "\n\n";

			// residual
			fout_obs_res << imageBlock->GPSTimeTags[i].GPSTime << endl;
			fout_obs_res << "position:" << endl;
			fout_obs_res << imageBlock->GPSTimeTags[i].navigationData.position[0] - *(bframes + i*BFrameBlockSize + 0) << "\t" << imageBlock->GPSTimeTags[i].navigationData.position[1] - *(bframes + i*BFrameBlockSize + 1) << "\t" << imageBlock->GPSTimeTags[i].navigationData.position[2] - *(bframes + i*BFrameBlockSize + 2) << endl;
			fout_obs_res << "orientation:" << endl;
			fout_obs_res << imageBlock->GPSTimeTags[i].navigationData.orientation[0] - *(bframes + i*BFrameBlockSize + 3) << "\t" << imageBlock->GPSTimeTags[i].navigationData.orientation[1] - *(bframes + i*BFrameBlockSize + 4) << "\t" << imageBlock->GPSTimeTags[i].navigationData.orientation[2] - *(bframes + i*BFrameBlockSize + 5) << endl;
		}
	}
	fout2.close();


	cout << "\n\n GCP OUT \n\n" << endl;
	ofstream fout3("UMSAT_GCP_OUT.txt");

	if (fout3.is_open())
	{
		fout3 << "\nUMSAT Output - GCP \n\n";

		fout_obs_res << endl << "GCPs" << endl;

		for (int i = 0; i < numberOfPoints; i++)
		{
			fout3.setf(ios::fixed);
			fout3.precision(6);
			fout3 <<imageBlock->objectPoints[i].GCPID << " " << *(points + i*PointBlockSize + 0) << " " << *(points + i*PointBlockSize + 1) << " " << *(points + i*PointBlockSize + 2) << "\n";
			//fout3 <<  *(pointStd + i*PointBlockSize + 0) << " " << *(pointStd + i*PointBlockSize + 1) << " " << *(pointStd + i*PointBlockSize + 2) << "\n\n";
			// residual
			fout_obs_res << imageBlock->objectPoints[i].GCPID << "\t";
			fout_obs_res << imageBlock->objectPoints[i].X - *(points + i*PointBlockSize + 0) << "\t" << imageBlock->objectPoints[i].Y - *(points + i*PointBlockSize + 1) << "\t" << imageBlock->objectPoints[i].Z - *(points + i*PointBlockSize + 2) << endl;
		}
	}
	fout3.close();
	fout_obs_res.close();

	cout << "\n\n SIGMA OUT \n\n" << endl;
	ofstream fout4("UMSAT_Sigma_OUT.txt");

	if (fout4.is_open())
	{
		fout << "\nUMSAT Output - Sigma \n\n";


		fout4 << aposterioristd << endl;
	
	}
	fout4.close();

	if (completeOutput)
	{
		cout << "\n\n RESIDUALS OUT \n\n" << endl;
		ofstream fout5("UMSAT__Point_Residuals_OUT.txt");
		ofstream fout6("UMSAT__Line_Residuals_OUT.txt");
		ofstream fout8("UMSAT__Distance_Residuals_OUT.txt");
		ofstream fout7("UMSAT__Patch_Residuals_OUT.txt");


		if (fout5.is_open() || fout6.is_open() || fout7.is_open())
		{
			fout << "\nUMSAT Output - Residuals \n";
			fout5 << "PhotoID \t PointID \t X \t Y \t Residual_X \t Residual_Y \n" << endl;
			fout6 << "PhotoID \t PointID \t Residual\n" << endl;
			fout8 << "PointA \t PointB \t Residual\n" << endl;
			fout7 << "PatchA \t PatchB \t PatchC \t Residual\n" << endl;


			int icount = 0;
			int icount_l = 0;
			int icount_d = 0;
			int icount_p = 0;


			//for (int i = 0; i < resBlocks.size(); i++)
			//{
			//	fout5 << resBlocks[icount] << endl;
			//	icount++;
			//}

			for (int i = 0; i < imageBlock->images.size(); i++)
			{
				for (int j = 0; j < imageBlock->images[i].imagePoints.size(); j++)
				{
					bool bSinglePoint = false;
					for (int k = 0; k < imageBlock->singlePointID.size(); k++)
					{
						if (strcmp(imageBlock->images[i].imagePoints[j].imagePointID, imageBlock->singlePointID[k]) == 0)
						{
							bSinglePoint = true;
						}
					}

					if (bSinglePoint)
						continue;
					if (imageBlock->images[i].imagePoints[j].flagI == 0)
					{
						fout5 << imageBlock->images[i].imageID << "\t" << imageBlock->images[i].imagePoints[j].imagePointID << "\t" << imageBlock->images[i].imagePoints[j].x << "\t" << imageBlock->images[i].imagePoints[j].y << "\t" << resBlocks[icount * 2 + icount_l + 0] << "\t" << resBlocks[icount * 2 + icount_l + 1] << "\n";
						icount++;
					}
					else
					{
						fout6 << imageBlock->images[i].imageID << "\t" << imageBlock->images[i].imagePoints[j].imagePointID << "\t" << resBlocks[icount * 2 + icount_l] << endl;
						icount_l++;
					}
				}

			}

			for (int i = 0; i < imageBlock->distances.size(); i++)
			{
				fout8 << imageBlock->distances[i].startID << "\t" << imageBlock->distances[i].endID << "\t" << resBlocks[icount * 2 + icount_l + icount_d] << endl;;
				icount_d++;
			}

//			cout << icount * 2 << endl;
//			cout << icount_l << endl << icount_d << endl << icount_p << endl;

			for (int i = 0; i < imageBlock->patches.size(); i++)
			{
				fout7 << imageBlock->patches[i].patchA << "\t" << imageBlock->patches[i].patchB << "\t" << imageBlock->patches[i].patchC << "\t" << endl;
				for (int j = 0; j < imageBlock->patches[i].patchPoints.size(); j++)
				{
					fout7 << resBlocks[icount * 2 + icount_l + icount_d + icount_p] << endl;
					icount_p++;
				}
			}

		}
		fout5.close();
		fout6.close();
		fout7.close();
		fout8.close();
	}
	
}//End of Ceres solve function



void CBundleAdjustment::getGCPIndex(char* imagePointID, int &gcpIndex){

	int indexFound = -1;

	for (int i = 0; i < imageBlock->objectPoints.size(); i++)
	{

		if (strcmp(imagePointID, imageBlock->objectPoints[i].GCPID) == 0)
		{
			gcpIndex = i;//returning GCP Index
			indexFound = 0;
			break;
		}

	}

	if (indexFound == -1)
	{
		gcpIndex = -1;
		cout << "GCP point not found" << endl;
	}


}

void CBundleAdjustment::getReferenceCameraIndex(const char* refCameraID, int &camIndex){

	int indexFound = -1;
	for (int i = 0; i < imageBlock->cameras.size(); i++)
	{
		if (strcmp(imageBlock->cameras[i].cameraID, refCameraID) == 0)
		{
			camIndex = i; // Return Reference camera index
			indexFound = 0;
			//cout << "i: " << camIndex << endl;
			break;
		}
	}

	if (indexFound == -1){

		cout << "Error: Reference Cam not found." << endl;
		camIndex = -1;

	}

}

void CBundleAdjustment::getGPSTimeTagIndex(double GPSTimeTag, int &gpsIndex){

	int indexFound = -1;
	for (int i = 0; i < imageBlock->GPSTimeTags.size(); i++)
	{
		if (imageBlock->GPSTimeTags[i].GPSTime == GPSTimeTag)
		{
			gpsIndex = i; // Return Reference camera index
			indexFound = 0;
			//cout << "i: " << camIndex << endl;
			break;
		}
	}

	if (indexFound == -1){

		cout << "Error: GPS Time Tag not found." << endl;
		gpsIndex = -1;

	}

}

void CBundleAdjustment::getGPSTimeListIndex(double GPSTimeTag, double *gpsTimes, int &gpsIndex){

	int indexFound = -1;
	for (int i = 0; i < imageBlock->GPSTimeList.size(); i++)
	{
		if (gpsTimes[i] == GPSTimeTag)
		{
			gpsIndex = i; // Return Reference camera index
			indexFound = 0;
			//cout << "i: " << camIndex << endl;
			break;
		}
	}

	if (indexFound == -1){

		cout << "Error: GPS Time List Index not found." << endl;
		gpsIndex = -1;

	}


}

void CBundleAdjustment::AddParameterBlocks(ceres::Problem * problem, double * X, const int blocksize, const int nblocks)
{
	for (int i = 0; i < nblocks; i++)
	{
		double * block = X + i*blocksize;
		problem->AddParameterBlock(block, blocksize);
	}
}

//Adapted from Mehdi's code
void CBundleAdjustment::Checkforconstantparams(ceres::Problem * problem, double * X, double *stdX, const int blocksize, const int nblocks, const double scaleSigma, vector<int> &check)
{

	std::vector<int> constindices;

	ceres::SubsetParameterization * subsetparam;
	for (int i = 0; i < nblocks; i++)
	{
		double * block = X + i*blocksize;
		double * stdblock = stdX + i*blocksize;


		int n_weighted_param_inblock = 0;
		for (int j = 0; j < blocksize; j++)
		{
			
			if (stdblock[j] <=sigmaFixed)//fixed
			{

				constindices.push_back(j);
				n_fixed_params++;
				check.push_back(0);
			}
			else if (stdblock[j]>sigmaFixed&&stdblock[j] < sigmaFree)//weighted
			{
				n_weighted_params++;
				n_weighted_param_inblock++;
				check.push_back(1);
			}
			else
				check.push_back(2);

		}
		//n_fixed_params += constindices.size();
		if (constindices.size() == blocksize)// all of the block is constant
		{
			problem->SetParameterBlockConstant(block);

		}
		else if (constindices.size()>0 && constindices.size() < blocksize) // a subset of block is constant
		{
			subsetparam = new ceres::SubsetParameterization(blocksize, constindices);
			problem->SetParameterization(block, subsetparam);

		}
		constindices.clear();

		
		if (n_weighted_param_inblock>0)
		{

			if (blocksize == 21)//Scaling and Weighting for IOPs done separately to allow for distortion scaling
			{
				int distortionModel = imageBlock->cameras[i].distortionModel;

				CBundleModel::ScaleAndWeightCameraParameters *cameraObservation = new CBundleModel::ScaleAndWeightCameraParameters(blocksize, stdblock, scaleSigma, distortionModel);
				
				for(int j = 0; j < blocksize; j++)
					cameraObservation->Observed_param.push_back(block[j]);//saving original observation parameters


				problem->AddResidualBlock(cameraObservation, NULL, block);

			}
			else//Weighting for all Parameters other than IOPs done here
			{
				CBundleModel::PseudoObservation * pseudoobservation = new CBundleModel::PseudoObservation(blocksize, stdblock, scaleSigma);

				for (int j = 0; j < blocksize; j++)
					pseudoobservation->Observed_param.push_back(block[j]);//saving original observation parameters



				problem->AddResidualBlock(pseudoobservation, NULL, block);

			}
			

			n_extra_residuals += (blocksize - n_weighted_param_inblock);// number of extra residuals we add for a psuedo observations
			//the extra residuals are absurd, but we have to add it due to limitations of ceres
		}
	}
}


//Function adapted from MSAT
void CBundleAdjustment::findImagePointPosition(int nOriImages, int order, double time, double *oriTimes, int &startPosition){//order of interpolation, time associated with image point, image index


	int midPosition;

	double time1, time2;
	int noparam = order+1 ;

	startPosition = 0;

	for (int i = 0; i<(nOriImages - 1); i++)
	{	

		time1 = oriTimes[i] - oriTimes[0];
		time2 = oriTimes[i + 1] - oriTimes[0];

		if ((time >= time1) && (time <= time2)) //if point inbetween the two OI
		{
			double diff1 = time - time1; //time difference b/n ori image and point 
			double diff2 = time - time2;

			if (fabs(diff1) <= fabs(diff2)) //select the closer one
				midPosition = i;
			else
				midPosition = i + 1;

			if ((noparam % 2) != 0)	//odd number of parameters
			{
				startPosition = midPosition - noparam / 2;
			}
			else //even number of parameters
			{
				if (midPosition == i) //close to ith ori image
				{
					startPosition = midPosition - noparam / 2; //move one to accommodate the mid pos
				}
				else
				{
					startPosition = midPosition - noparam / 2 + 1; //move one to accommodate the mid pos					
				}
			}

			break;
		}
	}

	//check the start and end position
	if ((startPosition + noparam) > nOriImages)
	{
		if ((nOriImages - noparam) > 0) //set the start position 
			startPosition = (nOriImages - noparam);
		else
			startPosition = 0;
	}
	else if (startPosition < 0)
	{
		startPosition = 0;
	}

}

// function to fill the covariance matrix
void CBundleAdjustment::fillCovarianceMatrix(double **covarianceMatrix, int rowBlock, int colBlock, int rowIndex, int colIndex, double *covarianceBlock)
{
	for (int i = rowIndex; i < rowBlock + rowIndex; i++)
	{
		for (int j = colIndex; j < colBlock + colIndex; j++)
		{
			covarianceMatrix[i][j] = *(covarianceBlock + colBlock*(i - rowIndex) + (j - colIndex));
		}
	}
}



