#pragma once

#ifndef UMSAT_MAIN_H
#define UMSAT_MAIN_H

#include <vector>
#include "stdlib.h"
#include <iostream>
#include <iomanip>
#include "CImageBlock.h"

using namespace std;

/*struct GPSTimeTag
{
	char * GPSTime;
	vector<int> imageIndex;
};*/

// these variables are global
extern int iterationNum;
extern double sigmaFree;
extern double sigmaFixed;
extern double sigmaStop; //Updated~Ron
extern bool distanceAvailable; 
extern bool patchAvailable;
extern bool completeOutput;
extern CImageBlock *imageBlock;

// function of splitting the message and fill them into the matrix


#endif