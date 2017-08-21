#pragma once

#ifndef UMSAT_CGCPHANDLER_H
#define UMSAT_CGCPHANDLER_H

#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>

#include "main.h"

XERCES_CPP_NAMESPACE_BEGIN

class GCPHandler : public DefaultHandler
{
public:
	GCPHandler();			// constructor function
	~GCPHandler();			// destructor function

public:
	// boolean variables to decide whether get the right string
	bool bPoint;			// a boolean variable for identify the tag of point in XML
	bool bPointID;			// a boolean variable for identify the tag of point ID in XML
	bool bPointX;			// a boolean variable for identify the tag of X coordinate of point in XML
	bool bPointY;			// a boolean variable for identify the tag of Y coordinate of point in XML
	bool bPointZ;			// a boolean variable for identify the tag of Z coordinate of point in XML
	bool bXYZCovariance;	// a boolean variable for identify the tag of XYZ covariance matrix in XML

public:
	int GCPIndex;			// integer type variable to relate the vector index with the point name
	bool bGCPExists;		// a boolean variable to verify if the GCP is existed or not


public:
	// define the virtual functions for reading from SAX2
	void startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs);	// override the pure virtual function startElement in Default Handler class
	void characters(const XMLCh *const chars, const XMLSize_t length);																// override the pure virtual function characters in Default Handler class
	void endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname);								// override the pure virtual function endElement in Default Handler class

};

XERCES_CPP_NAMESPACE_END

#endif