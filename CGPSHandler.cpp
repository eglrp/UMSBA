#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>

#include "CGPSHandler.h"
#include "main.h"
#include "UtilityRoutine.h"


XERCES_CPP_NAMESPACE_BEGIN

GPSHandler::GPSHandler()
{
	// initialize the boolean variable to false
	bTime = false;
	bOmega = false;
	bPhi = false;
	bKappa = false;
	bOriCov = false;
	bX = false;
	bY = false;
	bZ = false;
	bPosCov = false;
}

GPSHandler::~GPSHandler()
{
}

void GPSHandler::startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs)
{
	char* message = XMLString::transcode(localname);


	// compare the message from tag and the character of time tag, if they are the same, set bTime as true
	if (strcmp(message, cTime) == 0)
	{
		bTime = true;
	}
	// compare the message from tag and the character of omega, if they are the same, set bOmega as true
	if (strcmp(message, cOmega) == 0)
	{
		bOmega = true;
	}
	// compare the message from tag and the character of phi, if they are the same, set bPhi as true
	if (strcmp(message, cPhi) == 0)
	{
		bPhi = true;
	}
	// compare the message from tag and the character of kappa, if they are the same, set bKappa as true
	if (strcmp(message, cKappa) == 0)
	{
		bKappa = true;
	}
	// compare the message from tag and the character of orientation covariance matrix, if they are the same, set bOriCov as true
	if (strcmp(message, cOriCov) == 0)
	{
		bOriCov = true;
	}
	// compare the message from tag and the character of X, if they are the same, set bX as true
	if (strcmp(message, cX) == 0)
	{
		bX = true;
	}
	// compare the message from tag and the character of Y, if they are the same, set bY as true
	if (strcmp(message, cY) == 0)
	{
		bY = true;
	}
	// compare the message from tag and the character of Z, if they are the same, set bZ as true
	if (strcmp(message, cZ) == 0)
	{
		bZ = true;
	}
	// compare the message from tag and the character of position covariance matrix, if they are the same, set bPosCov as true
	if (strcmp(message, cPosCov) == 0)
	{
		bPosCov = true;
	}

}

void GPSHandler::characters(const XMLCh *const chars, const XMLSize_t length)
{
	char* message = XMLString::transcode(chars);

	// if the tag is time tag, find the index of the corresponded time tag, and find the image indices corresponded to this time tag
	if (bTime)
	{
		double time = atof(message);

		if (imageBlock->GPSTimeList.count(time))
		{
			timeTag = message;
			set<double>::iterator loc = imageBlock->GPSTimeList.find(time);
			// find the time tag index
			//int index;// = distance(imageBlock->GPSTimeList.begin(), loc);
			// find the image indices
			

			for (int i = 0; i < imageBlock->GPSTimeTags.size(); i++)
			{

				if (atof(message) == imageBlock->GPSTimeTags[i].GPSTime)
				{
					index = i;//returning GCP Index
				}

			}

			imageIndices = imageBlock->GPSTimeTags[index].imageIndex;

			//Store position Y data in GPS Time tag

			imageBlock->GPSTimeTags[index].navigationData.position(1) = atof(message);

		}
	}
	// if the tag is omega, assign omega to corresponded time tag
	if (bOmega)
	{
		//Store omega data in GPS Time tag
		if (index >=0 && index < imageBlock->GPSTimeTags.size())
		{
			imageBlock->GPSTimeTags[index].navigationData.orientation(0) = atof(message);
		}

		for (int i = 0; i < imageIndices.size(); i++)
		{
			int id = imageIndices[i];
			if (imageBlock->images[id].timeTags.size() > 0)
			{ 
				// if it's a frame camera with only one time tag
				if (imageBlock->images[id].timeTags.size() == 1)
				{
					imageBlock->images[id].navigationData[0].orientation(0) = atof(message);
				}
				// if it's a line camera with multiple time tags
				else
				{
					// find which one is the corresponded time tag
					for (int j = 0; j < imageBlock->images[id].timeTags.size(); j++)
					{
						if (strcmp(imageBlock->images[id].timeTags[j], timeTag) == 0)
						{
							imageBlock->images[id].navigationData[j].orientation(0) = atof(message);
						}
					}
				}
			}
		}
	}
	// if the tag is phi, assign phi to corresponded time tag
	if (bPhi)
	{

		//Store phi data in GPS Time tag
		if (index >= 0 && index < imageBlock->GPSTimeTags.size())
		{
			imageBlock->GPSTimeTags[index].navigationData.orientation(1) = atof(message);
		}

		for (int i = 0; i < imageIndices.size(); i++)
		{
			int id = imageIndices[i];
			if (imageBlock->images[id].timeTags.size() > 0)
			{
				// if it's a frame camera with only one time tag
				if (imageBlock->images[id].timeTags.size() == 1)
				{
					imageBlock->images[id].navigationData[0].orientation(1) = atof(message);
				}
				// if it's a line camera with multiple time tags
				else
				{
					// find which one is the corresponded time tag
					for (int j = 0; j < imageBlock->images[id].timeTags.size(); j++)
					{
						if (strcmp(imageBlock->images[id].timeTags[j], timeTag) == 0)
						{
							imageBlock->images[id].navigationData[j].orientation(1) = atof(message);
						}
					}
				}
			}
		}
	}
	// if the tag is kappa, assign kappa to corresponded time tag
	if (bKappa)
	{
		//Store omega data in GPS Time tag
		if (index >= 0 && index < imageBlock->GPSTimeTags.size())
		{
			imageBlock->GPSTimeTags[index].navigationData.orientation(2) = atof(message);
		}

		for (int i = 0; i < imageIndices.size(); i++)
		{
			int id = imageIndices[i];
			if (imageBlock->images[id].timeTags.size() > 0)
			{
				// if it's a frame camera with only one time tag
				if (imageBlock->images[id].timeTags.size() == 1)
				{
					imageBlock->images[id].navigationData[0].orientation(2) = atof(message);
				}
				// if it's a line camera with multiple time tags
				else
				{
					// find which one is the corresponded time tag
					for (int j = 0; j < imageBlock->images[id].timeTags.size(); j++)
					{
						if (strcmp(imageBlock->images[id].timeTags[j], timeTag) == 0)
						{
							imageBlock->images[id].navigationData[j].orientation(2) = atof(message);
						}
					}
				}
			}
		}
	}
	// if the tag is orientation covariance matrix, assign the orientation covariance matrix to corresponded time tag
	if (bOriCov)
	{
		//Need to be updated
		if (index >= 0 && index < imageBlock->GPSTimeTags.size())
		{
			split(message, imageBlock->GPSTimeTags[index].navigationData.orientationCovariance); //~~Need to be updated!!
		}

		for (int i = 0; i < imageIndices.size(); i++)
		{
			int id = imageIndices[i];
			//cout << "Timetags: " << imageBlock->images[id].timeTags.size() << endl;
			
			if (imageBlock->images[id].timeTags.size() > 0)
			{
				
				// if it's a frame camera with only one time tag
				if (imageBlock->images[id].timeTags.size() == 1)
				{
					split(message, imageBlock->images[id].navigationData[0].orientationCovariance);
				}
				// if it's a line camera with multiple time tags
				else
				{
					// find which one is the corresponded time tag
					for (int j = 0; j < imageBlock->images[id].timeTags.size(); j++)
					{
						
						if (strcmp(imageBlock->images[id].timeTags[j], timeTag) == 0)
						{
							split(message, imageBlock->images[id].navigationData[j].orientationCovariance);
						}
					}
				}
			}
		}
	}
	// if the tag is X, assign X to corresponded time tag
	if (bX)
	{
		//Store position X data in GPS Time tag
		if (index >= 0 && index < imageBlock->GPSTimeTags.size())
		{
			imageBlock->GPSTimeTags[index].navigationData.position(0) = atof(message);
		}

		for (int i = 0; i < imageIndices.size(); i++)
		{
			int id = imageIndices[i];
			if (imageBlock->images[id].timeTags.size() > 0)
			{
				// if it's a frame camera with only one time tag
				if (imageBlock->images[id].timeTags.size() == 1)
				{
					imageBlock->images[id].navigationData[0].position(0) = atof(message);
				}
				// if it's a line camera with multiple time tags
				else
				{
					// find which one is the corresponded time tag
					for (int j = 0; j < imageBlock->images[id].timeTags.size(); j++)
					{
						if (strcmp(imageBlock->images[id].timeTags[j], timeTag) == 0)
						{
							imageBlock->images[id].navigationData[j].position(0) = atof(message);
						}
					}
				}
			}
		}
	}
	// if the tag is Y, assign Y to corresponded time tag
	if (bY)
	{
		//Store position Y data in GPS Time tag
		if (index >= 0 && index < imageBlock->GPSTimeTags.size())
		{
			imageBlock->GPSTimeTags[index].navigationData.position(1) = atof(message);
		}

		for (int i = 0; i < imageIndices.size(); i++)
		{
			int id = imageIndices[i];
			if (imageBlock->images[id].timeTags.size() > 0)
			{
				// if it's a frame camera with only one time tag
				if (imageBlock->images[id].timeTags.size() == 1)
				{
					imageBlock->images[id].navigationData[0].position(1) = atof(message);
				}
				// if it's a line camera with multiple time tags
				else
				{
					// find which one is the corresponded time tag
					for (int j = 0; j < imageBlock->images[id].timeTags.size(); j++)
					{
						if (strcmp(imageBlock->images[id].timeTags[j], timeTag) == 0)
						{
							imageBlock->images[id].navigationData[j].position(1) = atof(message);
						}
					}
				}
			}
		}
	}
	// if the tag is Z, assign Z to corresponded time tag
	if (bZ)
	{
		//Store position Z data in GPS Time tag
		if (index >= 0 && index < imageBlock->GPSTimeTags.size())
		{
			imageBlock->GPSTimeTags[index].navigationData.position(2) = atof(message);
		}

		for (int i = 0; i < imageIndices.size(); i++)
		{
			int id = imageIndices[i];
			if (imageBlock->images[id].timeTags.size() > 0)
			{
				// if it's a frame camera with only one time tag
				if (imageBlock->images[id].timeTags.size() == 1)
				{
					imageBlock->images[id].navigationData[0].position(2) = atof(message);
				}
				// if it's a line camera with multiple time tags
				else
				{
					// find which one is the corresponded time tag
					for (int j = 0; j < imageBlock->images[id].timeTags.size(); j++)
					{
						if (strcmp(imageBlock->images[id].timeTags[j], timeTag) == 0)
						{
							imageBlock->images[id].navigationData[j].position(2) = atof(message);
						}
					}
				}
			}
		}
	}
	// if the tag is position covariance matrix, assign the position covariance matrix to corresponded time tag
	if (bPosCov)
	{
		//Store position covariance data in GPS Time tag
		if (index >= 0 && index < imageBlock->GPSTimeTags.size())
		{
			split(message, imageBlock->GPSTimeTags[index].navigationData.positionCovariance);
		}

		for (int i = 0; i < imageIndices.size(); i++)
		{
			int id = imageIndices[i];
			if (imageBlock->images[id].timeTags.size() > 0)
			{
				// if it's a frame camera with only one time tag
				if (imageBlock->images[id].timeTags.size() == 1)
				{
					split(message, imageBlock->images[id].navigationData[0].positionCovariance);
				}
				// if it's a line camera with multiple time tags
				else
				{
					// find which one is the corresponded time tag
					for (int j = 0; j < imageBlock->images[id].timeTags.size(); j++)
					{
						if (strcmp(imageBlock->images[id].timeTags[j], timeTag) == 0)
						{
							split(message, imageBlock->images[id].navigationData[j].positionCovariance);
						}
					}
				}
			}
		}
	}
}

void GPSHandler::endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname)
{
	char* message = XMLString::transcode(localname);

	// compare the message from end tag and the character of time tag, if they are the same, set bTime as false
	if (strcmp(message, cTime) == 0)
	{
		bTime = false;
	}
	// compare the message from end tag and the character of omega, if they are the same, set bOmega as false
	if (strcmp(message, cOmega) == 0)
	{
		bOmega = false;
	}
	// compare the message from end tag and the character of phi, if they are the same, set bPhi as false
	if (strcmp(message, cPhi) == 0)
	{
		bPhi = false;
	}
	// compare the message from end tag and the character of kappa, if they are the same, set bKappa as false
	if (strcmp(message, cKappa) == 0)
	{
		bKappa = false;
	}
	// compare the message from end tag and the character of orientation covariance matrix, if they are the same, set bOriCov as false
	if (strcmp(message, cOriCov) == 0)
	{
		bOriCov = false;
	}
	// compare the message from end tag and the character of X, if they are the same, set bX as false
	if (strcmp(message, cX) == 0)
	{
		bX = false;
	}
	// compare the message from end tag and the character of Y, if they are the same, set bY as false
	if (strcmp(message, cY) == 0)
	{
		bY = false;
	}
	// compare the message from end tag and the character of Z, if they are the same, set bZ as false
	if (strcmp(message, cZ) == 0)
	{
		bZ = false;
	}
	// compare the message from end tag and the character of position covariance matrix, if they are the same, set bPosCov as false
	if (strcmp(message, cPosCov) == 0)
	{
		bPosCov = false;
	}

}



XERCES_CPP_NAMESPACE_END