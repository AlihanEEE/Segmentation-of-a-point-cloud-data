#include "CommonProcesses.h"


CommonProcesses::CommonProcesses(const string &fileName)
	:pcdFileName(fileName), cloud(new PointCloud <PointXYZRGB>)
{
	readPcdFromFile();
	cout << "*************************************************" << endl
		<< "1. Rotate and save" << endl
		<< "2. downsampling and save" << endl
		<< "3. scale and save" << endl
		<< "4. just save" << endl
		<< "*************************************************" << endl;
	int temp;
	cin >> temp;
	setChoice(temp);
	
}

CommonProcesses::~CommonProcesses()
{

}

void CommonProcesses::setCloud(PointCloud<PointXYZRGB>::Ptr cl)
{
	cloud = cl;
}

void CommonProcesses::setSavedPcdFileName(string string)
{
	savedPcdFileName = string;
}

void CommonProcesses::setPcdFileName(string file)
{
	pcdFileName = file;
}

void CommonProcesses::setChoice(int c)
{
	if (c > 0 && c < 5) choice = c;
	else
	{
		throw invalid_argument("You've selected invalid choice");
	}
}



string CommonProcesses::getSavedPcdFileName(void) const
{
	return savedPcdFileName;
}

string CommonProcesses::getPcdFileName()const
{
	return pcdFileName;
}

PointCloud<PointXYZRGB>::Ptr CommonProcesses::getCloud(void)
{
	return cloud;
}

int CommonProcesses::getChoice(void) const
{
	return choice;
}

void CommonProcesses::readPcdFromFile()
{

	if (io::loadPCDFile <PointXYZRGB>(getPcdFileName(), *getCloud()) == -1)
	{
		cout << "Cloud reading failed." << endl;
		
	}
}



void CommonProcesses::pcdViewer(PointCloud<PointXYZRGB>::Ptr cloud)
{
	// Create a visualizer
	pcl::visualization::PCLVisualizer viewer;

	// Add the colored point cloud to the viewer
	viewer.addPointCloud(cloud, "colored_cloud");

	// Set the rendering properties of the point cloud
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "colored_cloud");

	// Set the background color
	viewer.setBackgroundColor(0, 0, 0);

	// Start the visualizer
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

}

void CommonProcesses::savePcdFile(string fileName, PointCloud<PointXYZRGB>::Ptr colored)
{
	setSavedPcdFileName(fileName);

	pcl::io::savePCDFileASCII<PointXYZRGB>(getSavedPcdFileName(), *colored);


}

PointCloud<PointXYZRGB>::Ptr CommonProcesses::rotateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
{

	// Define the angle of rotation (in radians) and the axis of rotation
	std::cout << "Enter the angle of rotation (in degrees): ";
	float angle;

	std::cin >> angle ;

	angle = angle * M_PI / 180.0;  // Convert degrees to radians

	
	float x =0;
	float y= 0;
	float z= 1;
	

	Eigen::Vector3f axis(x, y, z);

	// Define the rotation matrix
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	// Set the rotation matrix
	transform.rotate(Eigen::AngleAxisf(angle, axis));

	// Transform the point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*cloud, *rotated_cloud, transform);

	return rotated_cloud;
}

void CommonProcesses::downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	double leaf_size = 0.01;
	// create a voxel grid filter object
	pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
	voxel_grid_filter.setInputCloud(cloud);
	voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

	// apply the filter
	voxel_grid_filter.filter(*cloud);
}

void CommonProcesses::scalePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	float scale = 0.5;

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0, 0) = scale;
	transform(1, 1) = scale;
	transform(2, 2) = scale;
	pcl::transformPointCloud(*cloud, *cloud, transform);
}

