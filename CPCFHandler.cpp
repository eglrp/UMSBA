#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>

#include "CPCFHandler.h"
#include "main.h"
#include "UtilityRoutine.h"

XERCES_CPP_NAMESPACE_BEGIN

PCFHandler::PCFHandler()
{
	bPatchID = false;
	bPatchA = false;
	bPatchB = false;
	bPatchC = false;
	bPatchCovariance = false;
	bPoint = false;
	bX = false;
	bY = false;
	bZ = false;

	index = 0;
	pointIndex = 0;
}

PCFHandler::~PCFHandler()
{

}

void PCFHandler::startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs)
{
	char* message = XMLString::transcode(localname);

	if (strcmp(message, cPatchID) == 0)
	{
		bPatchID = true;
	}
	if (strcmp(message, cPatchA) == 0)
	{
		bPatchA = true;
	}
	if (strcmp(message, cPatchB) == 0)
	{
		bPatchB = true;
	}
	if (strcmp(message, cPatchC) == 0)
	{
		bPatchC = true;
	}
	if (strcmp(message, cPatchCovariance) == 0)
	{
		bPatchCovariance = true;
	}
	if (strcmp(message, cPoint) == 0)
	{
		bPoint = true;
	}
	if (strcmp(message, cX) == 0)
	{
		bX = true;
	}
	if (strcmp(message, cY) == 0)
	{
		bY = true;
	}
	if (strcmp(message, cZ) == 0)
	{
		bZ = true;
	}
}

void PCFHandler::characters(const XMLCh *const chars, const XMLSize_t length)
{
	char* message = XMLString::transcode(chars);

	if (bPatchID)
	{
		imageBlock->createPatch(message);
		index = imageBlock->patchIndex;
		pointIndex = -1;
	}

	if (bPatchA)
	{
		imageBlock->patches[index].patchA = message;
	}

	if (bPatchB)
	{
		imageBlock->patches[index].patchB = message;
	}

	if (bPatchC)
	{
		imageBlock->patches[index].patchC = message;
	}

	if (bPatchCovariance)
	{
		split(message, imageBlock->patches[index].patchCovariance);
	}

	if (bPoint)
	{
		pointIndex++;
		point tempPoint;
		imageBlock->patches[index].patchPoints.push_back(tempPoint);
		bPoint = false;
	}

	if (bX)
	{
		imageBlock->patches[index].patchPoints[pointIndex].X = atof(message);
	}
	if (bY)
	{
		imageBlock->patches[index].patchPoints[pointIndex].Y = atof(message);
	}
	if (bZ)
	{
		imageBlock->patches[index].patchPoints[pointIndex].Z = atof(message);
	}

}

void PCFHandler::endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname)
{
	char* message = XMLString::transcode(localname);

	if (strcmp(message, cPatchID) == 0)
	{
		bPatchID = false;
	}
	if (strcmp(message, cPatchA) == 0)
	{
		bPatchA = false;
	}
	if (strcmp(message, cPatchB) == 0)
	{
		bPatchB = false;
	}
	if (strcmp(message, cPatchC) == 0)
	{
		bPatchC = false;
	}
	if (strcmp(message, cPatchCovariance) == 0)
	{
		bPatchCovariance = false;
	}
	if (strcmp(message, cPoint) == 0)
	{
		bPoint = false;
	}
	if (strcmp(message, cX) == 0)
	{
		bX = false;
	}
	if (strcmp(message, cY) == 0)
	{
		bY = false;
	}
	if (strcmp(message, cZ) == 0)
	{
		bZ = false;
	}
}

XERCES_CPP_NAMESPACE_END