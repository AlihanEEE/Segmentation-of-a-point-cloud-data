#include "Region_Growing.h"



Region_Growing::Region_Growing(const string &fileName):Segmentation(fileName)
{

    extractClusters();
    colorRegionGrow(clusters);
    
    
    if (getChoice() == 1)
    {
        setColoredCloud(rotateCloud(getColoredCloud()));
        setSavedPcdFileName("Rotated_Region_Grow.pcd");
    }
    else if (getChoice() == 2)
    {
        downsample(getColoredCloud());
        setSavedPcdFileName("Downsampled_Region_Grow.pcd");

    }
    else if (getChoice() == 3)
    {
        scalePointCloud(getColoredCloud());
        setSavedPcdFileName("Scale_Region_Grow.pcd");

    }
    else if (getChoice() == 4)
    {
        setSavedPcdFileName("Cloud_After_Region_Growing_Segmentation.pcd");
    }

    savePcdFile(getSavedPcdFileName(), getColoredCloud());
    pcdViewer(getColoredCloud());
    
}

Region_Growing::~Region_Growing()
{
}


void Region_Growing::setClusters(vector<PointIndices> clust)
{
    clusters = clust;
}

void Region_Growing::setNumberOfClusters(int size)
{
    numberOfClusters = size;
}

vector<PointIndices> Region_Growing::getClusters() const
{
    return clusters;
}

int Region_Growing::getNumberOfClusters(void) const
{
    return numberOfClusters;
}

void Region_Growing::extractClusters()
{
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(getCloud());
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices(new std::vector <int>);
    pcl::removeNaNFromPointCloud(*getCloud(), *indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(getCloud());
    reg.setIndices(indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    reg.extract(clusters);

    setNumberOfClusters(clusters.size());

    cout << "Number Of Clusters: " << getNumberOfClusters() << endl;

}


