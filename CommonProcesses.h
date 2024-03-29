#ifndef COMMONPROCESSES_H
#define COMMONPROCESSES_H

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
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <Eigen/Core>
#include <stdexcept>

using namespace std;
using namespace pcl;

#pragma once
class CommonProcesses
{
public:
	
	/// Default Constructor
	CommonProcesses(const string &);
	
	/// set functions
	/// savedPcdFileName set Function
	void setCloud(PointCloud<PointXYZRGB>::Ptr);
	/// pccdFileName set Function
	void setSavedPcdFileName(string);
	/// cloud set Function
	void setPcdFileName(string);
	/// choice set function
	void setChoice(int);
	

	/// get functions
	/// savedPcdFileName get Function
	string getSavedPcdFileName(void) const;
	/// pccdFileName get Function
	string getPcdFileName(void)const;
	/// cloud get Function
	PointCloud<PointXYZRGB>::Ptr getCloud(void);
	/// choice get function
	int getChoice(void) const;
	

	////// This function read the file and initialize it to cloud
	void readPcdFromFile(void);



	/// Point Cloud Data viewer function
	void pcdViewer(PointCloud<PointXYZRGB>::Ptr);
	/// This function write the cloud into file
	void savePcdFile(string, PointCloud<PointXYZRGB>::Ptr);

	/// Rotate functions, it rotates the given cloud 
	PointCloud<PointXYZRGB>::Ptr  rotateCloud(PointCloud<PointXYZRGB>::Ptr& cloud);
	/// Downsample function, it reduces the number of point of the given cloud
	void downsample(PointCloud<PointXYZRGB>::Ptr& cloud);
	/// Scale function, it scales the given cloud
	void scalePointCloud(PointCloud<PointXYZRGB>::Ptr& cloud);

	/// Default Destructor
	~CommonProcesses();

private:
	/// Data Members
	/// raw point cloud data
	PointCloud<PointXYZRGB>::Ptr cloud;
	/// pcd file directory 
	string pcdFileName;
	/// saved pcd file directory 
	string savedPcdFileName;
	/// choice data member
	int choice;
};

#endif
