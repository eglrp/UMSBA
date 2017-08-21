#pragma once
#ifndef UMSAT_CCAMHANDLER_H
#define UMSAT_CCAMHANDLER_H

#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>


XERCES_CPP_NAMESPACE_BEGIN

class CAMHandler : public DefaultHandler
{
public:
	CAMHandler();						// constructor function
	~CAMHandler();						// destructor function

public:
	bool bCamID;						// a boolean variable for identify the tag of cameraID in XML 
	bool bCamType;						// a boolean variable for identify the tag of camera type in XML
	bool bRefCamID;						// a boolean variable for identify the tag of reference camera ID in XML
	bool bxp, byp, bc;					// boolean variables for identify the tags of xp, yp, and c in XML
	bool bxycCov;						// a boolean variable for identify the tag of xyc covariance matrix of in XML
	bool bDistModel;					// a boolean variable for identify the tag of distortion model in XML
	bool bNumPara;						// a boolean variable for identify the tag of number of distortion parameters in XML
	bool bParameters;					// a boolean variable for identify the tag of distortion parameters in XML
	bool bDistCov;						// a boolean variable for identify the tag of distortion covariance matrix in XML
	bool bX, bY, bZ;					// boolean variables for identify the tags of lever arm parameters in XML
	bool bOmega, bPhi, bKappa;			// boolean variables for identify the tags of bore sight parameters in XML
	bool bLeverCov;						// a boolean variable for identify the tag of levr arm covariance matrix in XML
	bool bBoreCov;						// a boolean variable for identify the tag of bore sight covariance matrix in XML
	bool bxMin, bxMax, byMin, byMax;	// boolean variables for identify the tag of scene range in XML
	bool bLineRate;						// a boolean variable for identify the tag of scan rate in XML
	bool bPixelSize;					// a boolean variable for identify the tag of pixel size in XML
	bool bLineOffset;					// a boolean variable for identify the tag of line offset in XML

public:
	char* cCamID = "CameraID";																	// a character pointer variable of camera ID for comparing with the characters from XML
	char* cCamType = "CameraType";																// a character pointer variable of camera type for comparing with the characters from XML
	char* cRefCamID = "ReferenceCameraID";														// a character pointer variable of reference camera ID for comparing with the characters from XML
	char* cxp = "xp"; char* cyp = "yp"; char* cc = "c";											// character pointer variables of xp, yp, c for comparing with the characters from XML
	char* cxycCov = "xycCovariance";															// a character pointer variable of xyc covariance matrix for comparing with the characters from XML
	char* cDistModel = "DistortionModel";														// a character pointer variable of distortion model for comparing with the characters from XML
	char* cNumPara = "NumberOfParameters";														// a character pointer variable of number of paramters for comparing with the characters from XML
	char* cParameters = "DistortionParameters";													// a character pointer variable of distortion parameters for comparing with the characters from XML
	char* cDistCov = "DistortionCovariance";													// a character pointer variable of distortion covariance matrix for comparing with the characters from XML
	char* cX = "X"; char* cY = "Y"; char* cZ = "Z";												// character pointer variables of lever arm parameters for comparing with the characters from XML
	char* cOmega = "Omega"; char* cPhi = "Phi"; char* cKappa = "Kappa";							// character pointer variables of bore sight parameters for comparing with the characters from XML
	char* cLeverCov = "LeverArmCovariance";														// a character pointer variable of lever arm covariance matrix for comparing with the characters from XML
	char* cBoreCov = "BoreSightCovariance";														// a character pointer variable of bore sight covariance matrix for comparing with the characters from XML
	char* cxMin = "XMin"; char* cxMax = "XMax"; char* cyMin = "YMin"; char* cyMax = "YMax";		// boolean variables for identify the tag of scene range in XML
	char* cLineRate = "LineRate";																// a boolean variable for identify the tag of scan rate in XML
	char* cPixelSize = "PixelSize";																// a boolean variable for identify the tag of pixel size in XML
	char* cLineOffset = "LineOffset";															// a boolean variable for identify the tag of line offset in XML

public:
	int index;


public:
	// define the virtual functions for reading from SAX2
	void startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs); // override the pure virtual function startElement in Default Handler class
	void characters(const XMLCh *const chars, const XMLSize_t length);															  // override the pure virtual function characters in Default Handler class
	void endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname);							  // override the pure virtual function endElement in Default Handler class
};

XERCES_CPP_NAMESPACE_END
#endif