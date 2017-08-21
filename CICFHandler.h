#ifndef UMSAT_CICFHANDLER_H
#define UMSAT_CICFHANDLER_H

#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>
#include "main.h"

XERCES_CPP_NAMESPACE_BEGIN

class ICFHandler : public DefaultHandler
{
public:
	ICFHandler();		// constructor function
	~ICFHandler();		// destructor function

public:
	// boolean variables to decide whether get the right string
	bool bImageID; //Unique ImageID
	bool bCameraID; //Unique CameraID
	bool bPointID; //Unique PointID
	bool bMeasurementAccuracy; // Image measurement accuracy
	bool bTimeBegin, bTimeEnd; //Begin and end of Image capture
	bool bNumberOrientationImages; //Number of orientation images
	bool bOrientationImages; // List of orientation images
	bool bTimeTag; //Time tags associated with each orientation image
	bool bInterpolation;
	bool bPoint;
	bool bLinePoint;
	bool bPointX, bPointY; //Image x,y measurement
	bool bPointCovariance; //x,y measurement variance-covariance matrix
	bool bFlagA; //Linear feature begin point
	bool bFlagB; //Linear feature end point
	bool bFlagI; //Linear feature intermediate point
	bool bRepMeasure;

	// multi-observation on one image check
public:
	struct cmpstr //Define custom comparator for string comparison within a char* set
	{
		bool operator()(const char* s1, const char* s2) const
		{
			return strcmp(s1, s2) < 0;
		}
	};
	set<char*, cmpstr> pointIDList;


public:
	// define the virtual functions for reading from SAX2
	void startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs);		// overload the pure virtual function startElement in Default Handler class
	void characters(const XMLCh *const chars, const XMLSize_t length);																	// overload the pure virtual function characters in Default Handler class
	void endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname);									// overload the pure virtual function endElement in Default Handler class

};

XERCES_CPP_NAMESPACE_END

#endif