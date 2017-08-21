#pragma once

#ifndef UMSAT_CIMAGE_H
#define UMSAT_CIMAGE_H

#include<iostream>
#include<stdio.h>
//#include<ceres/internal/eigen.h>
#include<vector>
#include"CImagePoint.h"
#include"CNavigation.h"
using namespace std;

#pragma once
class CImage
{
public:
	CImage();								// constructor function
	~CImage();								// destructor function

public:
	int cameraIndex;						// an int type variable of camera index (the camera which captured this image)
	char *imageID;							// a character pointer type variable of the image ID
	int imageIndex;							// the image index associated with the index in image vector
	int imagePointCounter;					// a counter for counting the point number in current image
	vector<CImagePoint> imagePoints;		// a vector to store the image point instances
	vector<char*> timeTags;					// a vector to store the time tags associate with this image
	double measurementAccuracy;				// the measurement accuracy
	vector<CNavigation> navigationData;		// a vector to store the navigation data associate with time tags
	int interpolationOrder;					// the order of interpolation (mostly for line camera image)
};

#endif
