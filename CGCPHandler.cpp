#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>
#include "CGCPHandler.h"
#include "main.h"
#include "UtilityRoutine.h"


XERCES_CPP_NAMESPACE_BEGIN

GCPHandler::GCPHandler()
{
	// initialize the boolean variable to false
	bPoint = false;
	bPointID = false;
	bPointX = false;
	bPointY = false;
	bPointZ = false;
	bXYZCovariance = false;
	bGCPExists = false;
}

GCPHandler::~GCPHandler()
{
}

void GCPHandler::startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs)
{
	char* message = XMLString::transcode(localname);

	// compare the message from tag and the character of point, if they are the same, set bPoint as true 
	if (strcmp(message, "Point") == 0)
	{
		bPoint = true;
	}
	// compare the message from tag and the character of point ID, if they are the same, set bPointID as true
	if (strcmp(message, "PointID") == 0)
	{
		bPointID = true;
	}
	// compare the message from tag and the character of X coordinate, if they are the same, set bPointX as true
	if (strcmp(message, "X") == 0)
	{
		bPointX = true;
	}
	// compare the message from tag and the character of Y coordinate, if they are the same, set bPointY as true
	if (strcmp(message, "Y") == 0)
	{
		bPointY = true;
	}
	// compare the message from tag and the character of Z coordinate, if they are the same, set bPointZ as true
	if (strcmp(message, "Z") == 0)
	{
		bPointZ = true;
	}
	// compare the message from tag and the character of XYZ covariance matrix, if they are the same, set bXYZCovariance as true
	if (strcmp(message, "XYZCovariance") == 0)
	{
		bXYZCovariance = true;
	}

}

void GCPHandler::characters(const XMLCh *const chars, const XMLSize_t length)
{
	char* message = XMLString::transcode(chars);

	// if the tag is point, nothing to do here

	if (bPoint)
	{
		//Nothing to do here
		bGCPExists = false;
		bPoint = false;
		
	}
	// if the tag is point ID, get the GCP index 
	if (bPointID)
	{
		if (imageBlock->GCPList.count(message))
		{
			bGCPExists = true;
			// Return the index of the GCP
			for (int i = 0; i < imageBlock->objectPoints.size(); i++)
			{

				if (strcmp(message, imageBlock->objectPoints[i].GCPID) == 0)
				{
					GCPIndex = i;//returning GCP Index
				}

			}


			//cout << GCPIndex << endl;
		}
		
	}
	// if the tag is X coordinate of point, assign it to the object point
	if ((bPointX) && (bGCPExists))
	{

		imageBlock->objectPoints[GCPIndex].X = atof(message);
		
	}
	// if the tag is Y coordinate of point, assign it to the object point
	if ((bPointY) && (bGCPExists))
	{
		
		imageBlock->objectPoints[GCPIndex].Y = atof(message);
	}
	// if the tag is Z coordinate of point, assign it to the object point
	if ((bPointZ) && (bGCPExists))
	{
		
		imageBlock->objectPoints[GCPIndex].Z = atof(message);
	}
	// if the tag is XYZ covariance matrix, assign it to the object point
	if ((bXYZCovariance) && (bGCPExists))
	{
		split(message, imageBlock->objectPoints[GCPIndex].XYZCovariance);


	/*	for (int i = 0; i < 3; i++)
=======
		/*for (int i = 0; i < 3; i++)
>>>>>>> 6350555ea21029172b6b33da271a6e6ea84b3a84
		{
			for (int j = 0; j < 3; j++)
			{
				cout << imageBlock->objectPoints[GCPIndex].XYZCovariance(i,j)<< endl;
			}
		}*/
	}
}

void GCPHandler::endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname)
{
	char* message = XMLString::transcode(localname);
	// compare the message from end tag and the character of point, if they are the same, set bPoint as false
	if (strcmp(message, "Point") == 0)
	{
		bPoint = false;
	}
	// compare the message from end tag and the character of point ID, if they are the same, set bPointID as false
	if (strcmp(message, "PointID") == 0)
	{
		bPointID = false;
	}
	// compare the message from end tag and the character of X coordinate, if they are the same, set bPointX as false
	if (strcmp(message, "X") == 0)
	{
		bPointX = false;
	}
	// compare the message from end tag and the character of Y coordinate, if they are the same, set bPointY as false
	if (strcmp(message, "Y") == 0)
	{
		bPointY = false;
	}
	// compare the message from end tag and the character of Z coordinate, if they are the same, set bPointZ as false
	if (strcmp(message, "Z") == 0)
	{
		bPointZ = false;
	}
	// compare the message from end tag and the character of XYZ covariance matrix, if they are the same, set bXYZCovariance as false
	if (strcmp(message, "XYZCovariance") == 0)
	{
		bXYZCovariance = false;
	}
	
}

XERCES_CPP_NAMESPACE_END