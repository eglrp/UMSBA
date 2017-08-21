#include "CDistance.h"
#include "main.h"
#include "UtilityRoutine.h"

using namespace std;

CDistance::CDistance()			// constructor function
{
	startIndex = 0;				// initialize start point index
	endIndex = 0;				// initialize end point index
	distance = 0.0;				// initialize distance measurement
	sigma = 0.0;				// initialize sigma value
}

CDistance::~CDistance()			// destructor function
{
}