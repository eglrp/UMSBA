#pragma once
#ifndef UMSAT_CDISHANDLER_H
#define UMSAT_CDISHANDLER_H

#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>

#include <vector>
#include "stdlib.h"
#include "Eigen/Core"
#include <iostream>

using namespace std;

XERCES_CPP_NAMESPACE_BEGIN

class DISHandler : public DefaultHandler
{
public:
	DISHandler();							// constructor function
	~DISHandler();							// destructor function

public:
	bool bDistance;							// a boolean variable for identify the tag of distance in XML
	bool bStartID;							// a boolean variable for identify the tag of start point iD in XML
	bool bEndID;							// a boolean variable for identify the tag of end point iD in XML
	bool bMeasurement;						// a boolean variable for identify the tag of distance measurement in XML
	bool bSigma;							// a boolean variable for identify the tag of sigma value in XML

public:
	char* cDistance = "Distance";			// a character pointer variable of distance for comparing with the characters from XML
	char* cStartID = "StartID";				// a character pointer variable of start point ID for comparing with the characters from XML
	char* cEndID = "EndID";					// a character pointer variable of end point ID for comparing with the characters from XML
	char* cMeasurement = "Measurement";		// a character pointer variable of distance measurement for comparing with the characters from XML
	char* cSigma = "Sigma";					// a character pointer variable of sigma value for comparing with the characters from XML

public:
	int distanceID;							// an ID to identify the element in vector

public:
	void startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs);		// override the pure virtual function startElement in Default Handler class
	void characters(const XMLCh *const chars, const XMLSize_t length);																	// override the pure virtual function characters in Default Handler class
	void endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname);									// override the pure virtual function endElement in Default Handler class
};


XERCES_CPP_NAMESPACE_END

#endif