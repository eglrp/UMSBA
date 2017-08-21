#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>
#include "CDISHandler.h"
#include "main.h"
#include "UtilityRoutine.h"


XERCES_CPP_NAMESPACE_BEGIN

DISHandler::DISHandler()
{
	// initialize the boolean variables to false
	bDistance = false;
	bStartID = false;
	bEndID = false;
	bMeasurement = false;
	bSigma = false;

	distanceID = -1;
}

DISHandler::~DISHandler()
{
}

void DISHandler::startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs)
{
	char* message = XMLString::transcode(localname);

	// compare the message from tag and the character of distance, if they are the same, set bDistance as true 
	if (strcmp(message, cDistance) == 0)
	{
		bDistance = true;
	}
	// compare the message from tag and the character of start point ID, if they are the same, set bStartID as true 
	if (strcmp(message, cStartID) == 0)
	{
		bStartID = true;
	}
	// compare the message from tag and the character of end point ID, if they are the same, set bEndID as true 
	if (strcmp(message, cEndID) == 0)
	{
		bEndID = true;
	}
	// compare the message from tag and the character of distance measurement, if they are the same, set bMeasurement as true 
	if (strcmp(message, cMeasurement) == 0)
	{
		bMeasurement = true;
	}
	// compare the message from tag and the character of sigma value, if they are the same, set bSigma as true 
	if (strcmp(message, cSigma) == 0)
	{
		bSigma = true;
	}
}

void DISHandler::characters(const XMLCh *const chars, const XMLSize_t length)
{
	char* message = XMLString::transcode(chars);
	// if the tag is distance, create a distance instance, and ID add 1
	if (bDistance)
	{
		CDistance distance;
		imageBlock->distances.push_back(distance);
		distanceID++;

		bDistance = false;
	}
	// if the tag is start point ID, assign the assign it to corresponded distance instance
	if (bStartID)
	{
		imageBlock->distances[distanceID].startID = message;
	}
	// if the tag is end point ID, assign the assign it to corresponded distance instance
	if (bEndID)
	{
		imageBlock->distances[distanceID].endID = message;
	}
	// if the tag is distance measurement, assign the assign it to corresponded distance instance
	if (bMeasurement)
	{
		imageBlock->distances[distanceID].distance = atof(message);
	}
	// if the tag is sigma, assign the assign it to corresponded distance instance
	if (bSigma)
	{
		imageBlock->distances[distanceID].sigma = atof(message);
	}
}

void DISHandler::endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname)
{
	char* message = XMLString::transcode(localname);
	// compare the message from tag and the character of distance, if they are the same, set bDistance as false 
	if (strcmp(message, cDistance) == 0)
	{
		bDistance = false;
	}
	// compare the message from tag and the character of start point ID, if they are the same, set bStartID as false
	if (strcmp(message, cStartID) == 0)
	{
		bStartID = false;
	}
	// compare the message from tag and the character of end point ID, if they are the same, set bEndID as false
	if (strcmp(message, cEndID) == 0)
	{
		bEndID = false;
	}
	// compare the message from tag and the character of distance measurement, if they are the same, set bMeasurement as false 
	if (strcmp(message, cMeasurement) == 0)
	{
		bMeasurement = false;
	}
	// compare the message from tag and the character of sigma value, if they are the same, set bSigma as false
	if (strcmp(message, cSigma) == 0)
	{
		bSigma = false;
	}
}

XERCES_CPP_NAMESPACE_END