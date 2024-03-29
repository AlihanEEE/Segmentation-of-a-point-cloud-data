#include "RAN_SA_C.h"

RAN_SA_C::RAN_SA_C(const string &fileName)
    :Segmentation(fileName), modelParameter("SACMODEL_PLANE")
{
    
    ransacAlgorithm();
     
    if (getChoice() == 1)
    {
        setColoredCloud(rotateCloud(getColoredCloud()));
        setSavedPcdFileName("Rotated_RANSAC.pcd");
    }
    else if (getChoice() == 2)
    {
        downsample(getColoredCloud());
        setSavedPcdFileName("Downsampled_RANSAC.pcd");
     
    }
    else if (getChoice() == 3)
    {
        scalePointCloud(getColoredCloud());
        setSavedPcdFileName("Scale_RANSAC.pcd");
       
    }
    else if (getChoice() == 4)
    {
        setSavedPcdFileName("Cloud_After_RANSAC_Segmentation.pcd");     
    }

    savePcdFile(getSavedPcdFileName(), getColoredCloud());
    pcdViewer(getColoredCloud());
}

RAN_SA_C::~RAN_SA_C()
{
}

void RAN_SA_C::setModelParameter(string model)
{

	modelParameter = "SACMODEL_PLANE";
}

string RAN_SA_C::getModelParameter(void)
{
	return modelParameter;
}

void RAN_SA_C::ransacAlgorithm()
{
    // Set up the RANSAC algorithm
    pcl::SACSegmentation<pcl::PointXYZRGB> sac;
    sac.setOptimizeCoefficients(true);
    sac.setModelType(pcl::SACMODEL_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold(0.02);

    // Set up the extraction object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // While there are still points in the original cloud
    while (getCloud()->size() > 0)
    {
        extract.setNegative(false);
        static int counter = 1;
        // Create an inlier point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // Create a model coefficient and inlier indices vector
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);


        // Perform segmentation
        sac.setInputCloud(getCloud());
        sac.segment(*inlier_indices, *coefficients);

        // Extract the inliers
        extract.setInputCloud(getCloud());
        extract.setIndices(inlier_indices);
        extract.filter(*cloud_inliers);

        // Check if we found any inliers
        if (cloud_inliers->size() == 0)
        {
            cout << "not found in " << counter + 1 << ". iteration" << endl;
            break;

        }

        // Color the inliers and add them to the segmented cloud
        for (size_t i = 0; i < cloud_inliers->size(); ++i)
        {
            cloud_inliers->points[i] = Segmentation::colorRANSAC(counter, cloud_inliers->points[i]);
        }

        *temp_cloud += *cloud_inliers;
        counter++;
        std::cout << "inlier counter: " << counter << std::endl;

        // Extract the inliers from the original cloud
        extract.setNegative(true);
        extract.filter(*getCloud());
    }

    setColoredCloud(temp_cloud);
}


