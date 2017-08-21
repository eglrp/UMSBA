#include <Eigen/Dense>
#include <vector>
#include <set>
#include "stdlib.h"

#include "CImageBlock.h"
#include <fstream>
using namespace std;
//#include "main.h"

CImageBlock::CImageBlock()
{
	// initialize the variables
	cameraIndex = 0;
	// when apply ++ the first element should be 0
	imageCounter = -1;
	patchIndex = -1;
}

CImageBlock::~CImageBlock()
{
}

int CImageBlock::createCamera(char* cameraID)
{
	bool cameraExisted = false; // boolean var for checking if the cameraID existed in current ImgBlk or not
	int ID = 0;
	// if the cameraIDList is empty, push the current ID into the list
	if (cameraIDList.size() == 0)
	{
		//initialize a camera object and push it into the cameras vector
		cameraIndex = 0;
		CCamera tempCam;
		tempCam.cameraID = cameraID; tempCam.cameraIndex = cameraIndex;
		cameras.push_back(tempCam);
		cameraIDList.insert(cameraID);
		
		return cameraIndex;
	}
	// search in the idlist for if there's existed current camera
	cameraExisted = cameraIDList.count(cameraID);

	// if existed return the ID; if not, return a new ID;
	if (cameraExisted)
	{ 
		//set<char*>::iterator loc = cameraIDList.find(cameraID); //Updated Ron 03/13/16
		// if existed, return the index
		//ID = distance(cameraIDList.begin(), loc);

		for (int i = 0; i<cameras.size(); i++)
		{

			if (strcmp(cameraID, cameras[i].cameraID) == 0)
			{
				ID = i;//returning GCP Index
			}

		}
		cameraIndex = ID;
		return cameraIndex;
	}
	else
	{
		//initialize a camera object and push it into the cameras vector
		cameraIndex = cameraIDList.size();
		CCamera tempCam;
		tempCam.cameraID = cameraID; tempCam.cameraIndex = cameraIndex;
		cameras.push_back(tempCam);
		cameraIDList.insert(cameraID);
		return cameraIndex;
	}

}

void CImageBlock::createImage(char *imageID) //~Ron
{
	// create an image instance, assign the camera Index and push it into the images vector
	imageCounter++;
	CImage tempImg;
	tempImg.imageIndex = imageCounter;
	tempImg.imageID = imageID; //~Ron
	images.push_back(tempImg);
}

void CImageBlock::createImagePoint(char *imagePointID)
{
	// creat an imagepoint instance, according to the imagecounter, find the corresponded image, and push it into the imagepoints vector
	CImagePoint tempImgPoint;
	tempImgPoint.imagePointID = imagePointID; //~Ron
	images[imageCounter].imagePoints.push_back(tempImgPoint);
	images[imageCounter].imagePointCounter++;
}

void CImageBlock::addGCPList(char* imagePointID)
{
	// check whether if the element is there or not, if not, we create a new instance of objpoint, assign the id to it and pust it into the vector
	if (!GCPList.count(imagePointID))
	{
		CObjectPoint tempObjectPoint;
		tempObjectPoint.GCPID = imagePointID;
		objectPoints.push_back(tempObjectPoint);
		GCPList.insert(imagePointID);
		//GCPList.insert(imagePointID);
		// **** 05/19/2016 ****
		pointID.push_back(imagePointID);
		int initCount = 1;
		pointCounter.push_back(initCount);
		// **** 05/19/2016 ****
	}
	// **** 05/19/2016 ****
	else
	{
		//vector<char*>::iterator iter = find(pointID.begin(), pointID.end(), cmpstr(imagePointID));
		//int index = distance(pointID.begin(), iter);
		int index;
		for (int i = 0; i < pointID.size(); i++)
		{
			if (strcmp(imagePointID, pointID[i]) == 0)
				index = i;
		}
		pointCounter[index] += 1;
	}
	// **** 05/19/2016 ****
}

void CImageBlock::addGPSTimeList(char* GPSTime)
{
	double time = atof(GPSTime);
	// if the time tag is existed, find the corresponded id, and find the struct push the img id into the vector
	if (GPSTimeList.count(time))
	{
		//set<char*>::iterator loc = GPSTimeList.find(GPSTime);
		int index;// = distance(GPSTimeList.begin(), loc);

		for (int i = 0; i < GPSTimeTags.size(); i++)
		{

			if (time == GPSTimeTags[i].GPSTime)
			{
				index = i;//returning GCP Index
			}

		}
		GPSTimeTags[index].imageIndex.push_back(imageCounter);
	}
	// if the time tag isn't existed, insert to the list, creat a new struct of time tag, initialize it, and push the current img id into the vector in the struct, then push the struct to the tags vector
	else
	{
		GPSTimeList.insert(time);
		GPSTimeTag tempTimeTag;
		tempTimeTag.GPSTime = time;
		tempTimeTag.imageIndex.push_back(imageCounter);
		GPSTimeTags.push_back(tempTimeTag);
		// create a navigation instance, assign the time tag, push it into the navigation data vector in corresponded image
		//CNavigation tempNavData; ~Ron
		//tempNavData.GPSTime = GPSTime; ~Ron
		//images[imageCounter].navigationData.push_back(tempNavData); ~Ron
	}

	// create a navigation instance, assign the time tag, push it into the navigation data vector in corresponded image
	CNavigation tempNavData;
	tempNavData.GPSTime = time;
	images[imageCounter].navigationData.push_back(tempNavData);
}

void CImageBlock::createPatch(char* patchID)
{
	patchIndex++;
	CPatch tempPatch;
	tempPatch.patchID = patchID;
	patches.push_back(tempPatch);
}

void CImageBlock::getSinglePointID()
{
	for (int i = 0; i < pointID.size(); i++)
	{
		// only appeared once and cannot be end point of line
		if (pointCounter[i] == 1 && strstr(pointID[i], "Line") == NULL && strstr(pointID[i], "line") == NULL)
		{
			singlePointID.push_back(pointID[i]);
			for (int j = 0; j < objectPoints.size(); j++)
			{
				if (strcmp(pointID[i], objectPoints[j].GCPID) == 0)
				{
					objectPoints.erase(objectPoints.begin() + j);
				}
			}
		}
	}
}

void CImageBlock::writeSummary()
{
	cout << "\n\n Summary OUT \n\n" << endl;
	ofstream fod("UMSAT_SUM_OUT.txt");
	if (fod.is_open())
	{
		fod << "Summary File" << endl <<endl;
		fod << "Images:" << endl;
		for (int i = 0; i < images.size(); i++)
		{
			fod << "ID:\t" << images[i].imageID;
			fod << "\tCamera:\t\t" << cameras[images[i].cameraIndex].cameraID << endl;
			fod << "Number Of Orientation Images:\t\t" << images[i].timeTags.size() << endl;
			for (int j = 0; j < images[i].timeTags.size(); j++)
			{
				fod << "Time tags:" << images[i].timeTags[j] << endl;
				fod << "Positions:\t\t" << images[i].navigationData[j].position[0] << "\t\t" << images[i].navigationData[j].position[1] << "\t\t" << images[i].navigationData[j].position[2] << endl;
				fod << "Orientation:\t\t" << images[i].navigationData[j].orientation[0] << "\t\t" << images[i].navigationData[j].orientation[1] << "\t\t" << images[i].navigationData[j].orientation[2] << endl;
			}
			fod << endl;
		}
		fod << endl;
		if (singlePointID.size() != 0)
		{
			fod << "WARNING!!! THERE'RE POINTS ONLY APPEARED ONCE" << endl << endl;
		}
		if (repIDs.size() != 0)
		{
			fod << "WARNING!!! THERE'RE POINTS MEASURED MULTIPLE TIMES IN SINGLE IMAGE" << endl << endl;
			for (int i = 0; i < repIDs.size(); i++)
			{
				fod << repIDs[i] << endl;
			}
		}
		fod << "Image Points:" << endl;
		for (int i = 0; i < pointID.size(); i++)
		{
			fod << "ID:\t\t" << pointID[i] << "\t\t Observations:\t\t" << pointCounter[i] << endl;
		}
		fod.close();
		vector<char*>().swap(pointID);
		vector<int>().swap(pointCounter);
	}
}