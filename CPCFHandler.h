#pragma once
#ifndef UMSAT_CPCFHANDLER_H
#define UMSAT_CPCFHANDLER_H

#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>


XERCES_CPP_NAMESPACE_BEGIN

class PCFHandler : public DefaultHandler
{
public:
	PCFHandler();						// constructor function
	~PCFHandler();						// destructor function

public:
	bool bPatchID;						
	bool bPatchA;						
	bool bPatchB;						
	bool bPatchC;					
	bool bPatchCovariance;
	bool bPoint;
	bool bX, bY, bZ;

public:
	char* cPatchID = "PatchID";
	char* cPatchA = "PatchA";
	char* cPatchB = "PatchB";
	char* cPatchC = "PatchC";
	char* cPatchCovariance = "PatchCovariance";
	char* cPoint = "Point";
	char* cX = "X"; char* cY = "Y"; char* cZ = "Z";

public:
	int index;
	int pointIndex;
												
public:
	// define the virtual functions for reading from SAX2
	void startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs); // override the pure virtual function startElement in Default Handler class
	void characters(const XMLCh *const chars, const XMLSize_t length);															  // override the pure virtual function characters in Default Handler class
	void endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname);							  // override the pure virtual function endElement in Default Handler class
};

XERCES_CPP_NAMESPACE_END
#endif