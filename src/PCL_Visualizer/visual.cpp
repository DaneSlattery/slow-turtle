// Code adapted from Geoffrey Biggs 
// by Nick Raal & Dane Slattery

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h> // Allows for .pcd to be read from disk
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h> // allows for transforms
#include <pcl/point_cloud.h>
#include <unistd.h> // for delay

// The USAGE of the complier
void printUsage (const char* progName)
{
    std::cout << "Usage: " << progName << "[options]\n" << "Options:\n" << "-h for help\n" << "-r followed by [file_name.pcd]" << std::endl; 
}


int main (int argc, char** argv)
{ // START of MAIN

  // Check which arguments were displayed by the user
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }


// This is where we want to read in our own .pcd files to view on the viewer
// ------------------------------------------------------------------------------------------------------
    
    //pcl::PointCloud<pcl::PointXYZRGB>& point_cloud = *point_cloud_ptr;
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd"); // fetches files from args
    

    //Creating the viewer
    pcl::visualization::PCLVisualizer viewer("Point Cloud Visualizer");
    //setting the background to black
    viewer.setBackgroundColor (0, 0, 0); // set background to black

    if (!pcd_filename_indices.empty ()) // checks to make sure a .pcd has been specified
    {
            // Need to call updatePointCloud() to change the point cloud each iteration
            // Creating for loop to go through each .pcd in the file specified
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ()); // create the PCL to read
      viewer.spinOnce();
      
      for(int i = 0; i < argc-1; i++)
      {
         
         // Load the PCD files from disk
          // if the file could not be opened then ...
          std::cout << "On file: " << argv[pcd_filename_indices[i]] << std::endl;
          if (pcl::io::loadPCDFile (argv[pcd_filename_indices[i]], *point_cloud_ptr) < 0)  
          {
            std::cout << "Error loading point cloud " << argv[pcd_filename_indices[i]] << std::endl << std::endl;
            printUsage (argv[0]);
            return -1;
          }
           pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr); // define the RGB colours
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
            
          if(i != 0)
          {
            viewer.removePointCloud("point_cloud"+(i-1));
           
          }
          
          viewer.addPointCloud(transformed_cloud,rgb,"point_cloud"+i);
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud"+i);

          viewer.spinOnce();
          
          usleep(10000);
          
          //viewer.spinOnce(100, true);
      }

      
    }
    
  return 0;
} //END of MAIN