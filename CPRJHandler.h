#pragma once
#ifndef UMSAT_CPRJHANDLER_H
#define UMSAT_CPRJHANDLER_H

#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>

XERCES_CPP_NAMESPACE_BEGIN

class PRJHandler : public DefaultHandler
{
public:
	PRJHandler();			// constructor function
	~PRJHandler();			// destructor function

public:
	bool bIterNum;			//a boolean variable for identify the tag of iteration number in XML
	bool bSigmaFree;		//a boolean variable for identify the tag of sigma free value in XML
	bool bSigmaFixed;		//a boolean variable for identify the tag of sigma fixed value in XML
	bool bSigmaStop;		//a boolean variable for identify the tag of sigma stop value in XML
	bool bDistanceAvailable;		//a boolean variable for identify the tag of sigma stop value in XML
	bool bPatchAvailable;
	bool bCompleteOutput;

public:
	char* cIterNum = "NumberOfIterations";	// a character pointer variable of number of iterations for comparing with the characters from XML
	char* cSigmaFree = "sigmaFree";			// a character pointer variable of sigma free for comparing with the characters from XML
	char* cSigmaFixed = "sigmaFixed";		// a character pointer variable of sigma fixed for comparing with the characters from XML
	char* cSigmaStop = "sigmaStop";			// a character pointer variable of sigma stop for comparing with the characters from XML
	char* cDistanceAvailable = "DistanceAvailable"; // a character pointer variable of distance available for comparing with the characters from XML
	char* cPatchAvailable = "PatchAvailable";
	char* cCompleteOutput = "CompleteOutput";

public:
	// define the virtual functions for reading from SAX2
	void startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs);		// override the pure virtual function startElement in Default Handler class
	void characters(const XMLCh *const chars, const XMLSize_t length);																	// override the pure virtual function characters in Default Handler class
	void endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname);									// override the pure virtual function endElement in Default Handler class
};

XERCES_CPP_NAMESPACE_END

#endif