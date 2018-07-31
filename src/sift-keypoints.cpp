// Dane Slattery

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>

int main(int argc, char** argv)
{
    std::vector<int> pcd_filename_indices =  pcl::console::parse_file_extension_argument(argc, argv, "pcd");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointWithScale>::Ptr point_cloud_out_ptr (new pcl::PointCloud<pcl::PointWithScale>);


    if (!pcd_filename_indices.empty())
    {
        std::string filename = argv[pcd_filename_indices[0]];

        if (pcl::io::loadPCDFile(filename, *point_cloud_ptr) == -1)
        {
            return 0;
        }
    }
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;

    sift_detect.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    
    // decreasing the min-scale helps detect more features
    sift_detect.setScales(0.001, 3, 3);
    sift_detect.setMinimumContrast(2.0);

    sift_detect.setInputCloud(point_cloud_ptr);

    sift_detect.compute(*point_cloud_out_ptr);

    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud(point_cloud_ptr, "points");

    for (size_t i = 0; i < point_cloud_out_ptr->size(); ++i)
    {
        const pcl::PointWithScale &p = point_cloud_out_ptr->points[i];

        float r = 0.2*p.scale;

        std::stringstream ss ("Keypoint");
        ss << i;

        viz.addSphere(p, r, 1.0, 0.0, 0.0, ss.str());
    }
    viz.spin();

}

