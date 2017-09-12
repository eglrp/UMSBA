#include "ReadWriteXML.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

#include "main.h"
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


int main(int argc, char *argv[])
{
    namespace po = boost::program_options;
    // Set number of precision
    setprecision(50);
    // file path
    boost::filesystem::path prjFile;
    boost::filesystem::path icfFile;
    boost::filesystem::path camFile;
    boost::filesystem::path gcpFile;
    boost::filesystem::path gpsFile;
    boost::filesystem::path disFile;
    boost::filesystem::path pcfFile;
    
    po::options_description description("Options");
    description.add_options()("help", "output usage")
    ("prjFile",po::value<boost::filesystem::path>(&prjFile)->required(),
     "Full path of the input project file")
    ("icfFile",po::value<boost::filesystem::path>(&icfFile)->required(),
     "Full path of the input icf file")
    ("camFile",po::value<boost::filesystem::path>(&camFile)->required(),
     "Full path of the input cam file")
    ("gcpFile",po::value<boost::filesystem::path>(&gcpFile)->required(),
     "Full path of the input gcp file")
    ("gpsFile",po::value<boost::filesystem::path>(&gpsFile)->required(),
     "Full path of the input gps file")
    ("disFile",po::value<boost::filesystem::path>(),
     "Full path of the input distance file (optional)")
    ("pcfFile",po::value<boost::filesystem::path>(),
     "Full path of the input patch file (optional)");
    
    boost::program_options::variables_map variableMap;
    po::store(po::parse_command_line(argc,argv,description), variableMap);
    // Output variableMap
    po::notify(variableMap);
    
    std::cout<<prjFile.string()<<std::endl;
    std::cout<<icfFile.string()<<std::endl;
    std::cout<<camFile.string()<<std::endl;
    std::cout<<gcpFile.string()<<std::endl;
    std::cout<<gpsFile.string()<<std::endl;
    
	std::cout << "This compiles!!!!" << std::endl;
    
    // Read prjFile
    readXML(prjFile.string().c_str(),"prj");
    std::cout << "PRJ Done" << std::endl;
	
    // Read icfFile
    readXML(icfFile.string().c_str(), "icf");
	std::cout << "ICF Done" << std::endl;
    
    // Read camFile
    readXML(camFile.string().c_str(), "cam");
	std::cout << "CAM Done" << std::endl;
    
    // Read gcpFile
    readXML(gcpFile.string().c_str(), "gcp");
	std::cout << "GCP Done" << std::endl;
    
    // Read gpsFile
	imageBlock->getSinglePointID();
	readXML(gpsFile.string().c_str(), "gps");
	std::cout << "GPS Done" << std::endl;
    
    // if disFile is available
    if (variableMap.count("disFile")){
        disFile = variableMap["disFile"].as<boost::filesystem::path>();
        readXML(disFile.string().c_str(), "dis");
        std::cout << "DIS Done" << std::endl;
    }
    // if pcfFile is available
    if (variableMap.count("pcfFile")){
        pcfFile = variableMap["pcfFile"].as<boost::filesystem::path>();
        readXML(pcfFile.string().c_str(), "pcf");
        std::cout << "PCF Done" << std::endl;
    }	imageBlock->writeSummary();
/*	for (int i = 0; i < imageBlock->objectPoints.size(); i++)
	{
		cout << "GCP Time Tags:  " << imageBlock->objectPoints[i].Y << endl;
	}*/
	CBundleAdjustment bundleAdjustment;
		bundleAdjustment.createAndSolveCeresProblem();
	return 0;
}
