

#include <iostream>
#include <boost/program_options.hpp>


#include <pcl/io/spectroscan_3d_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

namespace po=boost::program_options;

bool range_coloring = false;

pcl::visualization::PCLVisualizer* visualizer;

void updatePointCloud( const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud ){

}


int main(int argc, char** argv){
  po::options_description desc("./spectroscan3d_simple_viewer \n Real time view of Spectroscan3D point cloud.\n");
  std::string infile;

  desc.add_options()
    ("help", "produce help message")
    ("range,r", "color point cloud by range")
          ;
  po::positional_options_description p;
//  p.add("input",1);

  po::variables_map vm;
 try{
  po::store(po::command_line_parser(argc, argv).
  options(desc).positional(p).run(), vm);
  po::notify(vm);
 }
 catch( const std::exception& e)
 {
     std::cerr << e.what() << std::endl;
     std::cout << desc << std::endl;
     return 1;
 }
 if (vm.count("help") ){
   std::cout << desc << std::endl;
   return 0;
 }

 pcl::Spectroscan3DGrabber camera;

 camera.registerCallback<pcl::Spectroscan3DGrabber::sig_cb_xyzi_cloud>(  updatePointCloud  );
 camera.start();

 if (!camera.isRunning()){
	 std::cout << "Failed to start scanner \n";
 }
 std::cout << "Successfully started scanner \n";

 visualizer = new pcl::visualization::PCLVisualizer();
 visualizer->spin();

return 0;
}
