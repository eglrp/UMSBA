#include "CImage.h"
#include "main.h"


CImage::CImage()
{
	// initialize the variables
	cameraIndex = 0;			
	imageIndex = 0;
	imagePointCounter = -1;		// the counter will ++, therefore start from -1
	measurementAccuracy = 0.0;	
	interpolationOrder = 0;		// if this is a frame camera the order should always equals to zero
}


CImage::~CImage()
{
}
