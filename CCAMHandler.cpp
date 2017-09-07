#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>

#include "CCAMHandler.h"
#include "main.h"
#include "UtilityRoutine.h"

XERCES_CPP_NAMESPACE_BEGIN

CAMHandler::CAMHandler()
{
	// initialize the boolean variable to false
	bCamID = false;										 
	bCamType = false;
	bRefCamID = false;
	bxp = false; byp = false; bc = false;
	bxycCov = false;
	bDistModel = false;
	bNumPara = false;
	bParameters = false;
	bDistCov = false;
	bX = false; bY = false; bZ = false;
	bOmega = false; bPhi = false; bKappa = false;
	bLeverCov = false;
	bBoreCov = false;
	bxMax = false; bxMin = false; byMax = false; byMin = false;
	bLineRate = false;
	bPixelSize = false;
	bLineOffset = false;

	index = -1;		// initialize the index
}

CAMHandler::~CAMHandler()
{
}

void CAMHandler::startElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const	qname, const Attributes & attrs)
{

	char* message = XMLString::transcode(localname);

	cout << message << endl;
	
	// compare the message from tag and the character of camera ID, if they are the same, set bCamID as true
	if (strcmp(message, cCamID) == 0)
	{
		bCamID = true;
	}

	// compare the message from tag and the character of camera type , if they are the same, set bCamType as true
	if (strcmp(message, cCamType) == 0)
	{
		bCamType = true;
	}

	// compare the message from tag and the character of reference camera ID, if they are the same, set bRefCamID as true
	if (strcmp(message, cRefCamID) == 0)
	{
		bRefCamID = true;
	}

	// compare the message from tag and the character of xp, if they are the same, set bxp as true
	if (strcmp(message, cxp) == 0)
	{
		bxp = true;
	}

	// compare the message from tag and the character of yp, if they are the same, set byp as true
	if (strcmp(message, cyp) == 0)
	{
		byp = true;
	}

	// compare the message from tag and the character of c, if they are the same, set bc as true
	if (strcmp(message, cc) == 0)
	{
		bc = true;
	}

	// compare the message from tag and the character of xyc covariance matrix, if they are the same, set bxycCov as true
	if (strcmp(message, cxycCov) == 0)
	{
		bxycCov = true;
		
	}

	// compare the message from tag and the character of distortion model, if they are the same, set bDistModel as true
	if (strcmp(message, cDistModel) == 0)
	{
		bDistModel = true;
	}

	// compare the message from tag and the character of number of parameters, if they are the same, set bNumPara as true
	if (strcmp(message, cNumPara) == 0)
	{
		bNumPara = true;
	}

	// compare the message from tag and the character of distortion parameters, if they are the same, set bParameters as true
	if (strcmp(message, cParameters) == 0)
	{
		bParameters = true;
	}

	// compare the message from tag and the character of distortion covariance matrix, if they are the same, set bDistCov as true
	if (strcmp(message, cDistCov) == 0)
	{
		bDistCov = true;
	}

	//compare the message from tag and the character of X, if they are the same, set bX as true
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

	// compare the message from tag and the character of lever arm covariance matrix, if they are the same, set bLeverCov as true
	if (strcmp(message, cLeverCov) == 0)
	{
		bLeverCov = true;
	}

	// compare the message from tag and the character of bore sight covariance matrix, if they are the same, set bBoreCov as true
	if (strcmp(message, cBoreCov) == 0)
	{
		bBoreCov = true;
	}

	// compare the message from tag and the character of x min, if they are the same, set bxMin as true
	if (strcmp(message, cxMin) == 0)
	{
		bxMin = true;
	}

	// compare the message from tag and the character of x max, if they are the same, set bxMax as true
	if (strcmp(message, cxMax) == 0)
	{
		bxMax = true;
	}

	// compare the message from tag and the character of y min, if they are the same, set byMin as true
	if (strcmp(message, cyMin) == 0)
	{
		byMin = true;
	}

	// compare the message from tag and the character of y max, if they are the same, set byMax as true
	if (strcmp(message, cyMax) == 0)
	{
		byMax = true;
	}

	// compare the message from tag and the character of line rate if they are the same, set bLineRate as true
	if (strcmp(message, cLineRate) == 0)
	{
		bLineRate = true;
	}

	// compare the message from tag and the character of pixel size, if they are the same, set bPixelSize as true
	if (strcmp(message, cPixelSize) == 0)
	{
		bPixelSize = true;
	}

	// compare the message from tag and the character of line offset if they are the same, set bLineOffset as true
	if (strcmp(message, cLineOffset) == 0)
	{
		bLineOffset = true;
	}
}

void CAMHandler::characters(const XMLCh *const chars, const XMLSize_t length)
{
	char* message = XMLString::transcode(chars);


	// get the CamID
	if (bCamID)
	{
		// find if the current CamID in cam file is existed in our list or not
		if (imageBlock->cameraIDList.count(message))
		{
			//set<char*>::iterator loc = imageBlock->cameraIDList.find(message);
			// if existed, return the index
			//index = distance(imageBlock->cameraIDList.begin(), loc);

			for (int i = 0; i < imageBlock->cameras.size(); i++)
			{

				if (strcmp(message, imageBlock->cameras[i].cameraID) == 0)
				{
					index = i;//returning GCP Index
				}

			}
		}
	}
	// the camera index should be non-negative
	if (!(index <0))
	{
		// if the tag is camera type, assign the camera type to the corresponded camera
		if (bCamType)
		{
			imageBlock->cameras[index].cameraType = message;
		}
		// if the tag is reference camera ID, assign the reference camera ID to the corresponded camera
		if (bRefCamID)
		{
			imageBlock->cameras[index].refCameraID = message;
		}
		// if the tag is xp, assign xp to the corresponded camera
		if (bxp)
		{
			imageBlock->cameras[index].xp = atof(message);
		}
		// if the tag is yp, assign yp to the corresponded camera
		if (byp)
		{
			imageBlock->cameras[index].yp = atof(message);
		}
		// if the tag is c, assign c to the corresponded camera
		if (bc)
		{
			imageBlock->cameras[index].c = atof(message);
		}
		// if the tag is xyc covariance matrix, assign the xyc covariance matrix to the corresponded camera
		if (bxycCov)
		{
			// call the split function
			split(message, imageBlock->cameras[index].xycCovariance);
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					cout << imageBlock->cameras[index].xycCovariance(i, j)<<endl;
				}
			}			
		}
		// if the tag is distortion model, assign the distortion model to the corresponded camera
		if (bDistModel)
		{
			imageBlock->cameras[index].distortionModel = atoi(message);
		}
		// if the tag is number of parameters, assign the number of parameters to the corresponded camera
		if (bNumPara)
		{
			imageBlock->cameras[index].numParameters = atoi(message);
			// according to the number of parameters, define the dimension of the parameter vector and covariance matrix
			imageBlock->cameras[index].distortionCovariance = Eigen::MatrixXd(imageBlock->cameras[index].numParameters, imageBlock->cameras[index].numParameters);
			imageBlock->cameras[index].distortionParameters = Eigen::VectorXd::Zero(imageBlock->cameras[index].numParameters);
		}
		// if the tag is distortion parameters, assign the distortion parameters to the corresponded camera
		if (bParameters)
		{
			// call the split_vector function
			split_vector(message, imageBlock->cameras[index].distortionParameters);
		}
		// if the tag is distortion covariance matrix, assign the distortion covariance matrix to the corresponded camera
		if (bDistCov)
		{
			// call the split function
			split(message, imageBlock->cameras[index].distortionCovariance);
			for (int i = 0; i < imageBlock->cameras[index].numParameters; i++)
			{
				for (int j = 0; j < imageBlock->cameras[index].numParameters; j++)
				{
					cout << imageBlock->cameras[index].distortionCovariance(i, j) << endl;
				}
			}

		}
		// if the tag is X, assigh X to the corresponded camera
		if (bX)
		{
			imageBlock->cameras[index].leverarm(0) = atof(message);
		}
		// if the tag is Y, assign Y to the corresponded camera
		if (bY)
		{
			imageBlock->cameras[index].leverarm(1) = atof(message);
		}
		// if the tag is Z, assign Z to the corresponded camera
		if (bZ)
		{
			imageBlock->cameras[index].leverarm(2) = atof(message);
		}
		// if the tag is lever arm covariance matrix, assign the lever arm covariance matrix to the corresponded camera
		if (bLeverCov)
		{
			split(message, imageBlock->cameras[index].leverarmCovariance);
		}
		// if the tag is omega, assign omega to the corresponded camera
		if (bOmega)
		{
			imageBlock->cameras[index].boresight(0) = atof(message);
		}
		// if the tag is phi, assign phi to the corresponded camera
		if (bPhi)
		{
			imageBlock->cameras[index].boresight(1) = atof(message);
		}
		// if the tag is kappa, assign kappa to the corresponded camera
		if (bKappa)
		{
			imageBlock->cameras[index].boresight(2) = atof(message);
		}
		// if the tag is bore sight covariance matrix, assign the bore sight covariance matrix to the corresponded camera
		if (bBoreCov)
		{
			split(message, imageBlock->cameras[index].boresightCovariance);
		}
		// if the tag is x min, assign the x min to the corresponded camera
		if (bxMin)
		{
			imageBlock->cameras[index].xMin = atoi(message);
		}
		// if the tag is x max, assign the x max to the corresponded camera
		if (bxMax)
		{
			imageBlock->cameras[index].xMax = atoi(message);
		}
		// if the tag is y min, assign the y min to the corresponded camera
		if (byMin)
		{
			imageBlock->cameras[index].yMin = atoi(message);
		}
		// if the tag is y max, assign the y max to the corresponded camera
		if (byMax)
		{
			imageBlock->cameras[index].yMax = atoi(message);
		}
		// if the tag is scan rate, assign the line rate to the corresponded camera
		if (bLineRate)
		{
			imageBlock->cameras[index].lineRate = atoi(message);
		}
		// if the tag is pixel size, assign the pixel size to the corresponded camera
		if (bPixelSize)
		{
			imageBlock->cameras[index].pixelSize = atoi(message);
		}
		// if the tag is line offset, assign the line offset to the corresponded camera
		if (bLineOffset)
		{
			imageBlock->cameras[index].lineOffset = atoi(message);
		}
	}

}

void CAMHandler::endElement(const XMLCh *const uri, const XMLCh *const localname, const XMLCh *const qname)
{
	char* message = XMLString::transcode(localname);

	// compare the message from end tag and the character of camera ID, if they are the same, set bCamID as false
	if (strcmp(message, cCamID) == 0)
	{
		bCamID = false;
	}

	// compare the message from end tag and the character of camera type , if they are the same, set bCamType as false
	if (strcmp(message, cCamType) == 0)
	{
		bCamType = false;
	}

	// compare the message from end tag and the character of reference camera ID, if they are the same, set bRefCamID as false
	if (strcmp(message, cRefCamID) == 0)
	{
		bRefCamID = false;
	}

	// compare the message from end tag and the character of xp, if they are the same, set bxp as false
	if (strcmp(message, cxp) == 0)
	{
		bxp = false;
	}

	// compare the message from end tag and the character of yp, if they are the same, set byp as false
	if (strcmp(message, cyp) == 0)
	{
		byp = false;
	}

	// compare the message from end tag and the character of c, if they are the same, set bc as false
	if (strcmp(message, cc) == 0)
	{
		bc = false;
	}

	// compare the message from end tag and the character of xyc covariance matrix, if they are the same, set bxycCov as false
	if (strcmp(message, cxycCov) == 0)
	{
		bxycCov = false;

	}

	// compare the message from end tag and the character of distortion model, if they are the same, set bDistModel as false
	if (strcmp(message, cDistModel) == 0)
	{
		bDistModel = false;
	}

	// compare the message from end tag and the character of number of parameters, if they are the same, set bNumPara as false
	if (strcmp(message, cNumPara) == 0)
	{
		bNumPara = false;
	}

	// compare the message from end tag and the character of distortion parameters, if they are the same, set bParameters as false
	if (strcmp(message, cParameters) == 0)
	{
		bParameters = false;
	}

	// compare the message from end tag and the character of distortion covariance matrix, if they are the same, set bDistCov as false
	if (strcmp(message, cDistCov) == 0)
	{
		bDistCov = false;
	}

	//compare the message from end tag and the character of X, if they are the same, set bX as false
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

	// compare the message from end tag and the character of lever arm covariance matrix, if they are the same, set bLeverCov as false
	if (strcmp(message, cLeverCov) == 0)
	{
		bLeverCov = false;
	}

	// compare the message from end tag and the character of bore sight covariance matrix, if they are the same, set bBoreCov as false
	if (strcmp(message, cBoreCov) == 0)
	{
		bBoreCov = false;
	}

	// compare the message from tag and the character of x min, if they are the same, set bxMin as false
	if (strcmp(message, cxMin) == 0)
	{
		bxMin = false;
	}

	// compare the message from tag and the character of x max, if they are the same, set bxMax as false
	if (strcmp(message, cxMax) == 0)
	{
		bxMax = false;
	}

	// compare the message from tag and the character of y min, if they are the same, set byMin as false
	if (strcmp(message, cyMin) == 0)
	{
		byMin = false;
	}

	// compare the message from tag and the character of y max, if they are the same, set byMax as false
	if (strcmp(message, cyMax) == 0)
	{
		byMax = false;
	}

	// compare the message from tag and the character of line rate if they are the same, set bLineRate as false
	if (strcmp(message, cLineRate) == 0)
	{
		bLineRate = false;
	}

	// compare the message from tag and the character of pixel size, if they are the same, set bPixelSize as false
	if (strcmp(message, cPixelSize) == 0)
	{
		bPixelSize = false;
	}

	// compare the message from tag and the character of line offset if they are the same, set bLineOffset as false
	if (strcmp(message, cLineOffset) == 0)
	{
		bLineOffset = false;
	}
}



XERCES_CPP_NAMESPACE_END
