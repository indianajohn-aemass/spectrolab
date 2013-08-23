/*
 * spectroscan3d_simple_viewer.cpp
 * Simple program to stream point cloud data from a Spectroscan3D to
 * a 3D viewer.
 */
#include <iostream>
#include <boost/program_options.hpp>
#include <pcl/point_types.h>

#include <pcl/io/spectroscan_3d_io.h>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

namespace po=boost::program_options;

bool range_coloring = false;

class SimpleViewer{
private:
	pcl::visualization::PCLVisualizer* visualizer;
	boost::mutex cloud_mutex_;
	pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_;
	bool recieved_first_;

public:

	void run();
	void cloudCB(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
	void updatePointCloud( const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud );

};

void SimpleViewer::updatePointCloud( const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud ){
	typedef pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> CHandlerT;
	typedef pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZI> GHandlerT;

	GHandlerT::Ptr ghandler;
	ghandler.reset(new GHandlerT(cloud));

	CHandlerT::Ptr chandler;
	if (range_coloring) chandler.reset(new CHandlerT(cloud, "z"));
	else chandler.reset(new CHandlerT(cloud, "intensity"));

	visualizer->removeAllPointClouds(0);
	std::string name = "cloud";
	if (!recieved_first_){
		recieved_first_=true;
		return;
	}
	visualizer->addPointCloud<pcl::PointXYZI>(cloud, *chandler,*ghandler, name,0);
	visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud",0);
	visualizer->spinOnce(1, true);
}

void SimpleViewer::run() {

	visualizer= new pcl::visualization::PCLVisualizer("Spectroscan 3D Viewer");
	visualizer->addCoordinateSystem(0.5);
#if ( ( PCL_MAJOR_VERSION >=1) && (  PCL_MINOR_VERSION > 6) )
	visualizer->setCameraPosition (0, 0, -2,
    							   0, 0, 1,
    							   0,-1,0);
#else
visualizer->setCameraPose (0, 0, -2,
    					   0, 0, 1,
    					  0,-1,0);
#endif
	while(!visualizer->wasStopped()){
		if (cloud_!=NULL){
			boost::interprocess::scoped_lock<boost::mutex> lock( cloud_mutex_);
			updatePointCloud(cloud_);
			cloud_.reset();
		}
		if (recieved_first_) visualizer->spinOnce(30,false);
	}
}

void SimpleViewer::cloudCB(
	const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) {
	boost::interprocess::scoped_lock<boost::mutex> lock( cloud_mutex_);
	cloud_ = cloud;
}


int main(int argc, char** argv){
  po::options_description desc("./spectroscan3d_simple_viewer \n Real time view of Spectroscan3D point cloud.\n");
  std::string infile;

  desc.add_options()
    ("help", "produce help message")
    ("range,r", "color point cloud by range")
          ;
  po::positional_options_description p;

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


 pcl::Spectroscan3DGrabber* camera;
 try{
	 camera = new pcl::Spectroscan3DGrabber;
 }
 catch (std::exception& e){
	 std::cout << "Failed to open connection to camera\n";
	 std::cout << e.what() << "\n";
	 return -1;
 }
 
 range_coloring = vm.count("range");
 
 SimpleViewer viewer;

 camera->registerCallback<pcl::Spectroscan3DGrabber::sig_cb_xyzi_cloud>(
		 boost::bind(&SimpleViewer::cloudCB, &viewer, _1));
 camera->start();

 if (!camera->isRunning()){
	 std::cout << "Failed to start scanner \n";
 }
 std::cout << "Successfully started scanner \n";

viewer.run();

return 0;
}

