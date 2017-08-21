#pragma once
#ifndef UMSAT_CGPSHANDLER_H
#define UMSAT_CGPSHANDLER_H

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

class GPSHandler : public DefaultHandler
{
public:
	GPSHandler();				// constructor function
	~GPSHandler();				// destructor function

public:
	bool bTime;					// a boolean variable for identify the tag of time in XML
	bool bOmega;				// a boolean variable for identify the tag of omega in XML
	bool bPhi;					// a boolean variable for identify the tag of phi in XML
	bool bKappa;				// a boolean variable for identify the tag of kappa in XML
	bool bOriCov;				// a boolean variable for identify the tag of orientation covariance matrix in XML
	bool bX;					// a boolean variable for identify the tag of X in XML
	bool bY;					// a boolean variable for identify the tag of Y in XML
	bool bZ;					// a boolean variable for identify the tag of Z in XML
	bool bPosCov;				// a boolean variable for identify the tag of position covariance matrix in XML

public:
	char* cTime = "Time";						// a character pointer variable of time tag for comparing with the characters from XML
	char* cOmega = "Omega";						// a character pointer variable of omega for comparing with the characters from XML
	char* cPhi = "Phi";							// a character pointer variable of phi for comparing with the characters from XML
	char* cKappa = "Kappa";						// a character pointer variable of kappa for comparing with the characters from XML
	char* cOriCov = "OrientationCovariance";	// a character pointer variable of orientation covariance matrix for comparing with the characters from XML
	char* cX = "X";								// a character pointer variable of X for comparing with the characters from XML
	char* cY = "Y";								// a character pointer variable of Y for comparing with the characters from XML
	char* cZ = "Z";								// a character pointer variable of Z for comparing with the characters from XML
	char* cPosCov = "PositionCovariance";		// a character pointer variable of position covariance matrix for comparing with the characters from XML

public:
	char* timeTag;				// current time tag
	vector<int> imageIndices;	// image indices corresponded to the current time tag
	int index;					//Index corresponding to currnet time tag ~Debug

public:
	// define the virtual functions for reading from SAX2
	void startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs);		// override the pure virtual function startElement in Default Handler class
	void characters(const XMLCh *const chars, const XMLSize_t length);																	// override the pure virtual function characters in Default Handler class
	void endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname);									// override the pure virtual function endElement in Default Handler class
};


XERCES_CPP_NAMESPACE_END

#endif