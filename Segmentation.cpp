#include "Segmentation.h"

Segmentation::Segmentation(const string& file) :CommonProcesses(file), coloredCloud(new PointCloud <PointXYZRGB>)
{
    
    
}

void Segmentation::setColoredCloud(PointCloud<PointXYZRGB>::Ptr colored)
{
    coloredCloud = colored;
}

PointCloud<PointXYZRGB>::Ptr Segmentation::getColoredCloud(void)
{
    return coloredCloud;
}



Segmentation::~Segmentation()
{
}



void Segmentation::colorRegionGrow(vector<PointIndices>& clusters_)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud;

    if (!clusters_.empty())
    {
        coloredCloud = (new PointCloud<PointXYZRGB>)->makeShared();

        //srand(static_cast<unsigned int> (time(nullptr)));
        vector<unsigned char> colors;


        for (size_t i_segment = 0; i_segment < clusters_.size(); i_segment++)
        {
            // Generate unique R, G, and B values
            unsigned char R = static_cast<unsigned char>((i_segment * 137) % 256);
            unsigned char G = static_cast<unsigned char>((i_segment * 73) % 256);
            unsigned char B = static_cast<unsigned char>((i_segment * 191) % 256);

            // Add the R, G, and B values to the colors vector
            colors.push_back(R);
            colors.push_back(G);
            colors.push_back(B);
        }

        coloredCloud->width = getCloud()->width;
        coloredCloud->height = getCloud()->height;
        coloredCloud->is_dense = getCloud()->is_dense;
        for (const auto& i_point : *getCloud())
        {
            pcl::PointXYZRGB point;
            point.x = *(i_point.data);
            point.y = *(i_point.data + 1);
            point.z = *(i_point.data + 2);
            point.r = 255;
            point.g = 0;
            point.b = 0;
            coloredCloud->points.push_back(point);
        }

        int next_color = 0;
        for (const auto& i_segment : clusters_)
        {
            for (const auto& index : (i_segment.indices))
            {
                (*coloredCloud)[index].r = colors[3 * next_color];
                (*coloredCloud)[index].g = colors[3 * next_color + 1];
                (*coloredCloud)[index].b = colors[3 * next_color + 2];
            }
            next_color++;
        }
    }

    setColoredCloud(coloredCloud);
}

pcl::PointXYZRGB Segmentation::colorRANSAC(int index, pcl::PointXYZRGB point)
{
    point.r = (index * 30) % 256;
    point.g = (index * 137) % 256;
    point.b = (index * 191) % 256;

    return point;
}



