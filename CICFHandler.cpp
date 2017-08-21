#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>

#include "CICFHandler.h"
#include "main.h"
#include "UtilityRoutine.h"


XERCES_CPP_NAMESPACE_BEGIN

ICFHandler::ICFHandler()
{
	// initialize the variables
	bImageID = false;
	bCameraID = false;
	bPointID = false;
	bPoint = false;
	bLinePoint = false;
	bPointX = false;
	bPointY = false;

	bMeasurementAccuracy = false;
	bTimeBegin = false;
	bTimeEnd = false;
	bNumberOrientationImages = false;
	bOrientationImages = false;
	bTimeTag = false;
	bInterpolation = false;

	bPointCovariance = false;
	bFlagA = false;
	bFlagB = false;
	bFlagI = false;
	bRepMeasure = false;

}

ICFHandler::~ICFHandler()
{
}

void ICFHandler::startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs)
{
	char* message = XMLString::transcode(localname);
	// compare the message from tag and the image ID character, if they are the same, set bImageID as true
	if (strcmp(message, "ImageID") == 0)
	{
		bImageID = true;
	}
	// compare the message from tag and the camera ID character, if they are the same, set bCameraID as true
	if (strcmp(message, "CameraID") == 0)
	{
		bCameraID = true;
	}
	// compare the message from tag and the point ID character, if they are the same, set bPointID as true
	if (strcmp(message, "ID") == 0)
	{
		bPointID = true;
	}
	// compare the message from tag and the point character, if they are the same, set bPoint as true
	if (strcmp(message, "Point") == 0)
	{
		bPoint = true;
	}
	// compare the message from tag and the line point character, if they are the same, set bPoint as true
	if (strcmp(message, "LinePoint") == 0)
	{
		bLinePoint = true;
	}
	// compare the message from tag and the x coordinate character, if they are the same, set bPointX as true
	if (strcmp(message, "x") == 0)
	{
		bPointX = true;
	}
	// compare the message from tag and the y coordinate character, if they are the same, set bPointY as true
	if (strcmp(message, "y") == 0)
	{
		bPointY = true;
	}
	// compare the message from tag and the measurement accuracy character, if they are the same, set bMeasurementAccuracy as true
	if (strcmp(message, "MeasurementAccuracy") == 0)
	{
		bMeasurementAccuracy = true;

	}
	// compare the message from tag and the time begin character, if they are the same, set bTimeBegin as true
	if (strcmp(message, "TimeBegin") == 0)
	{
		bTimeBegin = true;
	}
	// compare the message from tag and the time end character, if they are the same, set bTimeEnd as true
	if (strcmp(message, "TimeEnd") == 0)
	{
		bTimeEnd = true;
	}
	// compare the message from tag and the number of orientation images, if they are the same, set bNumberOrientaionImages as true
	if (strcmp(message, "NumberOrientationImages") == 0)
	{
		bNumberOrientationImages = true;
	}
	// compare the message from tag and the orientation images character, if they are the same, set bOrientationImages as true
	if (strcmp(message, "OrientationImages") == 0)
	{
		bOrientationImages = true;
	}
	// compare the message from tag and the time tag character, if they are the same, set bTimeTag as true
	if (strcmp(message, "TimeTag") == 0)
	{
		bTimeTag = true;
	}
	// compare the message from tag and the image ID character, if they are the same, set bImageID as true
	if (strcmp(message, "Interpolation") == 0)
	{
		bInterpolation = true;
	}
	// compare the message from tag and the xy covariance matrix character, if they are the same, set bPointCovariance as true
	if (strcmp(message, "PointCovariance") == 0)
	{
		bPointCovariance = true;
	}
	// compare the message from tag and the flag A character, if they are the same, set bFlagA as true
	if (strcmp(message, "flagA") == 0)
	{
		bFlagA = true;
	}
	// compare the message from tag and the flag B character, if they are the same, set bFlagB as true
	if (strcmp(message, "flagB") == 0)
	{
		bFlagB = true;
	}
	// compare the message from tag and the flag I character, if they are the same, set bFlagI as true
	if (strcmp(message, "flagI") == 0)
	{
		bFlagI = true;
	}
}

void ICFHandler::characters(const XMLCh *const chars, const XMLSize_t length)
{
	char* message = XMLString::transcode(chars);

	//cout << message << endl;
	// if the tag is image ID, creat an image instance
	if (bImageID)
	{
		imageBlock->createImage(message);
		bImageID = false;
		pointIDList.clear();
	}
	// if the tag is camera ID, creat and camera instance and save the camera index
	if (bCameraID)
	{		
		imageBlock->images[imageBlock->imageCounter].cameraIndex = imageBlock->createCamera(message);
		bCameraID = false;
	}
	// if the tag is omega, assign omega to the corresponded camera
	if (bPointID)
	{
		if (!bLinePoint)
		{
			int listSize = pointIDList.size();
			pointIDList.insert(message);
			int currentSize = pointIDList.size();
			if (listSize == currentSize)
			{
				bRepMeasure = true;
				imageBlock->repIDs.push_back(message);
			}
		}
		if (!bRepMeasure)
		{
			imageBlock->createImagePoint(message);
			if (!bLinePoint){
				imageBlock->addGCPList(message);
			}
			bPointID = false;
			bLinePoint = false;
		}
	}
	// if the tag is measurement accuracy, assign the measurement accuracy to the image instance
	if (bMeasurementAccuracy){
		imageBlock->images[imageBlock->imageCounter].measurementAccuracy = atof(message);
		bMeasurementAccuracy = false;

	}
	
	if (bTimeBegin){

		bTimeBegin = false;
		//Nothing to do here
	}

	if (bTimeEnd){
		
		bTimeEnd = false;
		//Nothing to do here
	}

	if (bNumberOrientationImages){

		bNumberOrientationImages = false;
		//Nothing to do here

	}
	if (bOrientationImages){

		bOrientationImages = false;
		//Nothing to do here

	}
	// if the tag is time tag, assign the time tag to the image, and adding the time tag list
	if (bTimeTag){

		imageBlock->images[imageBlock->imageCounter].timeTags.push_back(message);
		imageBlock->addGPSTimeList(message);
		bTimeTag = false;

	}
	// if the tag is interpolation order, assign the interpolation order to the image
	if (bInterpolation){
		
		imageBlock->images[imageBlock->imageCounter].interpolationOrder = atoi(message);
		bInterpolation = false;

	}
	if (bPoint){

		//Nothing to do here
		bPoint = false;
	}
	// if the tag is x coordinate of image point, assign the x to the image point
	if (bPointX)
	{	
		if (!bRepMeasure)
		{
			imageBlock->images[imageBlock->imageCounter].imagePoints[imageBlock->images[imageBlock->imageCounter].imagePointCounter].x = atof(message);
		}
			bPointX = false;
		

	}
	// if the tag is y coordinate of image point, assign the y to the image instance
	if (bPointY)
	{
		if (!bRepMeasure)
		{
			imageBlock->images[imageBlock->imageCounter].imagePoints[imageBlock->images[imageBlock->imageCounter].imagePointCounter].y = atof(message);
		}
			bPointY = false;
		
	}
	// if the tag is xy coordinate covariance matrix, assign the covariance matrix to the image
	if (bPointCovariance)
	{
		if (!bRepMeasure)
		{
			split(message, imageBlock->images[imageBlock->imageCounter].imagePoints[imageBlock->images[imageBlock->imageCounter].imagePointCounter].xyCovariance);
		}
			bPointCovariance = false;
		

	}
	// if the tag is flag A, assign the flag A to the image
	if (bFlagA)
	{
		if (!bRepMeasure)
		{
			if (strcmp(message, "1") == 0){
				imageBlock->images[imageBlock->imageCounter].imagePoints[imageBlock->images[imageBlock->imageCounter].imagePointCounter].flagA = true;
			}
		}
			bFlagA = false;
		
	}
	// if the tag is flag B, assign the flag B to the image
	if (bFlagB)
	{
		if (!bRepMeasure)
		{
			if (strcmp(message, "1") == 0){
				imageBlock->images[imageBlock->imageCounter].imagePoints[imageBlock->images[imageBlock->imageCounter].imagePointCounter].flagB = true;
			}
		}
		bFlagB = false;
	}
	// if the tag is flag I, assign the flag I to the image
	if (bFlagI)
	{
		if (!bRepMeasure)
		{
			if (strcmp(message, "1") == 0){
				imageBlock->images[imageBlock->imageCounter].imagePoints[imageBlock->images[imageBlock->imageCounter].imagePointCounter].flagI = true;
			}
		}
		bFlagI = false;

	}
}

void ICFHandler::endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname)
{
	char* message = XMLString::transcode(localname);
	// compare the message from end tag and the image ID character, if they are the same, set bImageID as false
	if (strcmp(message, "ImageID") == 0)
	{
		bImageID = false;
	}
	// compare the message from end tag and the camera ID character, if they are the same, set bCameraID as false
	if (strcmp(message, "CameraID") == 0)
	{
		bCameraID = false;
	}
	// compare the message from end tag and the point ID character, if they are the same, set bPointID as false
	if (strcmp(message, "ID") == 0)
	{
		bPointID = false;
	}
	// compare the message from end tag and the point character, if they are the same, set bPoint as false
	if (strcmp(message, "Point") == 0)
	{
		bPoint = false;
		bRepMeasure = false;
	}
	// compare the message from end tag and the x coordinate character, if they are the same, set bPointX as false
	if (strcmp(message, "x") == 0)
	{
		bPointX = false;
	}
	// compare the message from end tag and the y coordinate character, if they are the same, set bPointY as false
	if (strcmp(message, "y") == 0)
	{
		bPointY = false;
	}
	// compare the message from end tag and the measurement accuracy character, if they are the same, set bMeasurementAccuracy as false
	if (strcmp(message, "MeasurementAccuracy") == 0)
	{
		bMeasurementAccuracy = false;

	}
	// compare the message from end tag and the time begin character, if they are the same, set bTimeBegin as false
	if (strcmp(message, "TimeBegin") == 0)
	{
		bTimeBegin = false;
	}
	// compare the message from end tag and the time end character, if they are the same, set bTimeEnd as false
	if (strcmp(message, "TimeEnd") == 0)
	{
		bTimeEnd = false;
	}
	// compare the message from end tag and the number of orientation images, if they are the same, set bNumberOrientaionImages as false
	if (strcmp(message, "NumberOrientationImages") == 0)
	{
		bNumberOrientationImages = false;
	}
	// compare the message from end tag and the orientation images character, if they are the same, set bOrientationImages as false
	if (strcmp(message, "OrientationImages") == 0)
	{
		bOrientationImages = false;
	}
	// compare the message from tag and the time tag character, if they are the same, set bTimeTag as false
	if (strcmp(message, "TimeTag") == 0)
	{
		bTimeTag = false;
	}
	// compare the message from end tag and the image ID character, if they are the same, set bImageID as false
	if (strcmp(message, "Interpolation") == 0)
	{
		bInterpolation = false;
	}
	// compare the message from end tag and the xy covariance matrix character, if they are the same, set bPointCovariance as false
	if (strcmp(message, "PointCovariance") == 0)
	{
		bPointCovariance = false;
	}
	// compare the message from end tag and the flag A character, if they are the same, set bFlagA as false
	if (strcmp(message, "flagA") == 0)
	{
		bFlagA = false;
	}
	// compare the message from end tag and the flag B character, if they are the same, set bFlagB as false
	if (strcmp(message, "flagB") == 0)
	{
		bFlagB = false;
	}
	// compare the message from end tag and the flag I character, if they are the same, set bFlagI as false
	if (strcmp(message, "flagI") == 0)
	{
		bFlagI = false;
	}
}

XERCES_CPP_NAMESPACE_END