#include "main.h"
#include "ReadWriteXML.h"
#include <fstream>
#include "CBundleAdjustment.h"

using namespace xercesc;

CImageBlock *imageBlock = new CImageBlock();

int iterationNum = 0;
double sigmaFree = 0.0;
double sigmaFixed = 0.0;
double sigmaStop = 0.0;
bool distanceAvailable = false;
bool patchAvailable = false;
bool completeOutput = true;


int main(int argc, char *argv[]) {


	//Set number of precision
	setprecision(50);

	std::cout << "This compiles!!!!" << std::endl;

	readXML("prj.xml", "prj");
	std::cout << "PRJ Done" << std::endl;
	readXML("icf.xml", "icf");
	std::cout << "ICF Done" << std::endl;
	readXML("cam.xml", "cam");
	std::cout << "CAM Done" << std::endl;
	readXML("gcp.xml", "gcp");
	std::cout << "GCP Done" << std::endl;
	imageBlock->getSinglePointID();
	readXML("gps.xml", "gps");
	std::cout << "GPS Done" << std::endl;
	if (distanceAvailable)
	{
		readXML("dis.xml", "dis");
		std::cout << "DIS Done" << std::endl;
	}

	if (patchAvailable)
	{
		readXML("pcf.xml", "pcf");
		std::cout << "PCF Done" << std::endl;
	}
	imageBlock->writeSummary();
/*	for (int i = 0; i < imageBlock->objectPoints.size(); i++)
	{
		cout << "GCP Time Tags:  " << imageBlock->objectPoints[i].Y << endl;
	}*/
	CBundleAdjustment bundleAdjustment;
		bundleAdjustment.createAndSolveCeresProblem();

		int testing = 1;

	return 0;
}