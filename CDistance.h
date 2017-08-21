#pragma once

#ifndef UMSAT_CDISTANCE_H
#define UMSAT_CDISTANCE_H

class CDistance
{
public:
	CDistance();				// constructor function
	~CDistance();				// destructor function

public:
	char* startID;				// start point ID
	char* endID;				// end point ID
	int startIndex;				// start point index
	int endIndex;				// end point index
	double distance;			// distance measurement between start point and end point
	double sigma;				// sigma value
};



#endif