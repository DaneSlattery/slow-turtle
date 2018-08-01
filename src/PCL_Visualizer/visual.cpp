// Code adapted from Geoffrey Biggs 
// by Nick Raal & Dane Slattery

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h> // Allows for .pcd to be read from disk
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h> // allows for transforms

// The USAGE of the complier
void printUsage (const char* progName)
{
    std::cout << "Usage: " << progName << "[options]\n" << "Options:\n" << "-h for help\n" << "-r followed by [file_name.pcd]" << std::endl; 
}

// Creates a pcl: XYZRGB pointer
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Point Cloud Viewer"));
  viewer->setBackgroundColor (0, 0, 0); // set background to black
  viewer->removeAllPointClouds(); //clear the view
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "Point Cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Point Cloud");
  //viewer->addCoordinateSystem (1.0); 
  viewer->initCameraParameters ();
  //viewer->spinOnce(100);       
  return (viewer);
}


int main (int argc, char** argv)
{ // START of MAIN

  // Check which arguments were displayed by the user
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  bool rgb(false);
  if (pcl::console::find_argument (argc, argv, "-r") >= 0)
  {
    rgb = true;
    std::cout << "RGB point cloud visualization\n";
  }
  else
  {
    printUsage (argv[0]);
    return 0;
  }

// This is where we want to read in our own .pcd files to view on the viewer
// ------------------------------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>); // create the PCL to read
    pcl::PointCloud<pcl::PointXYZRGB>& point_cloud = *point_cloud_ptr;
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");

    if (!pcd_filename_indices.empty ()) // checks to make sure a .pcd has been specified
    {
            // Need to call updatePointCloud() to change the point cloud each iteration
            // Creating for loop to go through each .pcd in the file specified
      for(int i = 1; i <= 24; i++)
      {
          std::string filename = argv[pcd_filename_indices[i]];
          std::cout << "the filename is: " << filename << std::endl;
          // if the file could not be opened then ...
            if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
            {
            std::cout << "Was not able to open file \""<<filename<<"\".\n";
            printUsage (argv[0]);
            return 0;
            }
// Want to rotate the point cloud as it is upside down
           
            float angle = 3.14159;   // angle of rotation, radians = 180 degrees
            // Using Eigen::Affine3f for 3D point cloud
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.translation() << 0.0, 0.0, 0.0; // translate
            transform.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitZ())); // rotation
            //std::cout << transform.matrix() << std::endl; // display the rotation matrix
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ()); // doing the transformation
            
            // applying the transformation to the cloud
            pcl::transformPointCloud (*point_cloud_ptr, *transformed_cloud, transform);


  // This displays the point cloud
          boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
              if (rgb)
              {
                  viewer = rgbVis(transformed_cloud); // show the transformed point cloud
              } else {
                  EXIT_FAILURE;
              }
              while (!viewer->wasStopped ())
              {
                  viewer->spinOnce (100);
                  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
              }
      }

    } else std::cout << "No .pcd file was given. gg" << std::endl;
// ------------------------------------------------------------------------------------------------------

  
} //END of MAIN