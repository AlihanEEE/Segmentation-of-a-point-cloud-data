#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "CommonProcesses.h"



using namespace std;
using namespace pcl;
#pragma once

class Segmentation :public CommonProcesses // Segmentation is a CommonProcesses
{
public:

	/// Default Constructor
	Segmentation(const string &);
	/// set functions
	/// Colored Point Cloud set function
	void setColoredCloud(PointCloud<PointXYZRGB>::Ptr);
	

	/// get functions
	/// Colored Point Cloud get function
	PointCloud<PointXYZRGB>::Ptr getColoredCloud(void);
	

	/// Segmentation functions
	/// Regiongrowing segmentation functions
	void colorRegionGrow(vector<PointIndices>& );
	/// RANSAC segmentation functions
	PointXYZRGB colorRANSAC(int , PointXYZRGB );

	/// Default Destructor
	~Segmentation();


private:
	/// Data members
	/// colored cloud data member it stores segmented raw point cloud
	PointCloud<PointXYZRGB>::Ptr coloredCloud;

};
#endif
