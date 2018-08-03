// Dane Slattery

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char** argv)
{
    float mean = atof(argv[2]);
    float stddev = atof(argv[3]);
    float startz = atof(argv[4]);
    float endz = atof(argv[5]); //z
    float startx = atof(argv[6]);
    float endx = atof(argv[7]); //x
    float starty = atof(argv[8]); 
    float endy = atof(argv[9]); //y

    std::vector<int> pcd_filename_indices =  pcl::console::parse_file_extension_argument(argc, argv, "pcd");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints (new pcl::PointCloud<pcl::PointWithScale>);
    // load in the point cloud
    if (!pcd_filename_indices.empty())
    {
        std::string filename = argv[pcd_filename_indices[0]];

        if (pcl::io::loadPCDFile(filename, *point_cloud_in) == -1)
        {
            return 0;
        }
    }
    

    // z pass through filter
    pcl::PassThrough<pcl::PointXYZRGB> passz;
    passz.setInputCloud(point_cloud_in);
    passz.setFilterFieldName("z");
    passz.setFilterLimits(startz, endz);
    passz.filter(*point_cloud_out);

    // x pass through filter
    pcl::PassThrough<pcl::PointXYZRGB> passx;
    passx.setInputCloud(point_cloud_out);
    passx.setFilterFieldName("x");
    passx.setFilterLimits(startx, endx);
    passx.filter(*point_cloud_out);

    // y pass through filter
    pcl::PassThrough<pcl::PointXYZRGB> passy;
    passy.setInputCloud(point_cloud_out);
    passy.setFilterFieldName("y");
    passy.setFilterLimits(starty, endy);
    passy.filter(*point_cloud_out);


    // statistical filter the point cloud
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (point_cloud_out);
    sor.setMeanK(mean);
    sor.setStddevMulThresh(stddev);
    sor.filter(*point_cloud_out);

    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;

    sift_detect.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    
    // decreasing the min-scale helps detect more features
    sift_detect.setScales(0.01, 16, 16);
    sift_detect.setMinimumContrast(2.0);

    sift_detect.setInputCloud(point_cloud_out);

    sift_detect.compute(*keypoints);

    pcl::visualization::PCLVisualizer viz;
    // viz.addPointCloud(point_cloud_in, "points");
    viz.addPointCloud(point_cloud_out, "output");
  
    for (size_t i = 0; i < keypoints->size(); ++i)
        {
            const pcl::PointWithScale &p = keypoints->points[i];

            float r = 0.2*p.scale;

            std::stringstream ss ("Keypoint");
            ss << i;

            viz.addSphere(p, r, 1.0, 0.0, 0.0, ss.str());
    }
    viz.spin();

}

