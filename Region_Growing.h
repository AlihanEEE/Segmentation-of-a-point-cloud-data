#ifndef REGION_GROWING_H
#define REGION_GROWING_H
/*
#include <string>
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>

using namespace std;
using namespace pcl;
*/
#include "Segmentation.h"
#pragma once
class Region_Growing: public Segmentation // Region_Growing is a Segmentation
{
public:
	
	/// Default Constructor
	Region_Growing(const string&);
	
	/// set functions
	/// cluster set function
	void setClusters(vector <PointIndices>);
	/// clusters' size set function
	void setNumberOfClusters(int);

	/// get functions
	/// cluster get function
	vector <PointIndices> getClusters(void) const;
	/// clusters' size get function
	int getNumberOfClusters(void) const;
	
	/// Region Growing algorithm function
	void extractClusters(void);
	
	/// Default Destructor
	~Region_Growing();



private:

	/// Data members
	/// cluster data member
	vector <PointIndices> clusters;
	/// Data members
	/// number of cluster data member
	int numberOfClusters;

};

#endif