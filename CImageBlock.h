#pragma once

#ifndef UMSAT_CIMAGEBLOCK_H
#define UMSAT_CIMAGEBLOCK_H

#include <Eigen/Dense>
#include <vector>
#include <set>
#include "stdlib.h"

#include "CCamera.h"
#include "CImage.h"
#include "CImagePoint.h"
#include "CObjectPoint.h"
#include "CDistance.h"
#include "CPatch.h"
#include "UtilityRoutine.h"


using namespace std;

class CImageBlock
{
public:
	CImageBlock();			// constructor function
	~CImageBlock();			// destructor function

public:

	struct cmpstr //Define custom comparator for string comparison within a char* set
	{
		bool operator()(const char* s1, const char* s2) const
		{
			return strcmp(s1, s2) < 0;
		}
	};




	vector<CCamera> cameras;					// a vector which storing the camera instances
	vector<CImage> images;						// a vector which storing the image instances
	vector<CObjectPoint> objectPoints;			// a vector which storing the object points
	vector<CDistance> distances;				// a vector which storing the distances
	set<char*, cmpstr> GCPList;					// a set which storing the object points' IDs (non-repeatable)
	set<double, less<double>> GPSTimeList;			// a set which storing the GPSTimeTag struct (non-repeatable)
	vector<GPSTimeTag> GPSTimeTags;				// a set which storing the GPSTimeTag IDs (non-repeatable)
	int cameraIndex;							// an int variable which shows the current camera index
	set<char*, cmpstr> cameraIDList;			// a set which storing the camera IDs (non-repeatable)
	int imageCounter;							// a counter to count the current image number
	vector<CPatch> patches;
	int patchIndex;
	vector<char*> pointID;						// a vector storing the unique point id 
	vector<int> pointCounter;					// a corresponded vector for image point to count how many time it appears
	vector<char*> singlePointID;				// a vector storing the unique point id which only appeared once
	vector<char*> repIDs;

public:
	int createCamera(char* cameraID);			// function to create a camera instance, return the camera index
	void createImage(char* imageID);			// function to create a image instance
	void createImagePoint(char *imagePointID);	// function to create a image point instance
	void addGCPList(char* imagePointID);		// function to add a object point ID to the object point ID set
	void addGPSTimeList(char* GPSTime);			// function to add a time tag ID to the time tag ID set
	void createPatch(char* patchID);
	void getSinglePointID();
	void writeSummary();
};

#endif