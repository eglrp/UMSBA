#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>

#include "CPRJHandler.h"
#include "main.h"
#include "UtilityRoutine.h"


XERCES_CPP_NAMESPACE_BEGIN

PRJHandler::PRJHandler()
{
	// initialize the variables
	bIterNum = false;
	bSigmaFree = false;
	bSigmaFixed = false;
	bSigmaStop = false;
	bDistanceAvailable = false;
	bPatchAvailable = false;
	bCompleteOutput = false;
}

PRJHandler::~PRJHandler()
{
}

void PRJHandler::startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs)
{
	char* message = XMLString::transcode(localname);
	// compare the message from tag and the number of iterations character, if they are the same, set bIterNum as true
	if (strcmp(message, cIterNum) == 0)
	{
		bIterNum = true;
	}
	// compare the message from tag and the sigma free character, if they are the same, set bSigmaFree as true
	if (strcmp(message, cSigmaFree) == 0)
	{
		bSigmaFree = true;
	}
	// compare the message from tag and the sigma fixed character, if they are the same, set bSigmaFixed as true
	if (strcmp(message, cSigmaFixed) == 0)
	{
		bSigmaFixed = true;
	}
	// compare the message from tag and the sigma stop character, if they are the same, set bSigmaStop as true
	if (strcmp(message, cSigmaStop) == 0)
	{
		bSigmaStop = true;
	}
	// compare the message from tag and the distance available character, if they are the same, set bSigmaStop as true
	if (strcmp(message, cDistanceAvailable) == 0)
	{
		bDistanceAvailable = true;
	}
	if (strcmp(message, cPatchAvailable) == 0)
	{
		bPatchAvailable = true;
	}
	if (strcmp(message, cCompleteOutput) == 0)
	{
		bCompleteOutput = true;
	}

}

void PRJHandler::characters(const XMLCh *const chars, const XMLSize_t length)
{
	char* message = XMLString::transcode(chars);
	// if the tag is number of iterations, assign the value
	if (bIterNum)
	{
		iterationNum = atoi(message);
	}
	// if the tag is sigma free, assign the value
	if (bSigmaFree)
	{
		sigmaFree = atof(message);
	}
	// if the tag is sigma fixed, assign the value
	if (bSigmaFixed)
	{
		sigmaFixed = atof(message);
	}
	// if the tag is sigma stop, assign the value
	if (bSigmaStop)
	{
		sigmaStop = atof(message);
	}
	if (bDistanceAvailable)
	{
		if (strcmp("1", message) == 0)
		{
			distanceAvailable = true;
		}
		else
		{
			distanceAvailable = false;
		}
	}

	if (bPatchAvailable)
	{
		if (strcmp("1", message) == 0)
		{
			patchAvailable = true;
		}
		else
		{
			patchAvailable = false;
		}
	}

	if (bCompleteOutput)
	{
		if (strcmp("1", message) == 0)
		{
			completeOutput = true;
		}
		else
		{
			completeOutput = false;
		}
	}


}

void PRJHandler::endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname)
{
	char* message = XMLString::transcode(localname);
	// compare the message from end tag and the number of iterations character, if they are the same, set bIterNum as false
	if (strcmp(message, cIterNum) == 0)
	{
		bIterNum = false;
	}
	// compare the message from end tag and the sigma free character, if they are the same, set bSigmaFree as false
	if (strcmp(message, cSigmaFree) == 0)
	{
		bSigmaFree = false;
	}
	// compare the message from end tag and the sigma fixed character, if they are the same, set bSigmaFixed as false
	if (strcmp(message, cSigmaFixed) == 0)
	{
		bSigmaFixed = false;
	}
	// compare the message from end tag and the sigma stop character, if they are the same, set bSigmaStop as false
	if (strcmp(message, cSigmaStop) == 0)
	{
		bSigmaStop = false;
	}
	// compare the message from end tag and the distance available character, if they are the same, set bDistanceAvilable as false
	if (strcmp(message, cDistanceAvailable) == 0)
	{
		bDistanceAvailable = false;
	}

	if (strcmp(message, cPatchAvailable) == 0)
	{
		bPatchAvailable = false;
	}

	if (strcmp(message, cCompleteOutput) == 0)
	{
		bCompleteOutput = false;
	}
}

XERCES_CPP_NAMESPACE_END