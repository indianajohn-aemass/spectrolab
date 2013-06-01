

#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/spectroscan_3d_io.h>

namespace po=boost::program_options;

std::string file_root;
size_t number_to_grab=1, count=0;

void savecloud( const spectrolab::Scan::ConstPtr& frame, time_t scan_time ){

	boost::filesystem::path path = file_root;

	char fname[300];
	sprintf(fname, "%s_%04d.bin", path.c_str(),count);
	std::cout << "Saving to " << fname << " \n";
	frame->save(fname);
	count++;
	if (count>= number_to_grab){
		exit(0);
	}
}


int main(int argc, char** argv){
  po::options_description desc("./spectroscan3d_grabframe  frame_root_name \n Save binary scan images.  Each frame is frame_root_name_NUMBER.bin\n");

  desc.add_options()
    ("help", "produce help message")
     ("output,o",po::value<std::string>(&file_root)->required(), "root frame name ")
    ("number,n",po::value<size_t>(&number_to_grab)->default_value(1), "number of frames to grab")
    ;

  po::positional_options_description p;
   p.add("output",1);

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

 pcl::Spectroscan3DGrabber grabber;
 boost::signals2::connection c= grabber.registerCallback<spectrolab::SpectroScan3D::sig_camera_cb>(  savecloud  );
 grabber.start();

 while(1){	boost::this_thread::sleep(boost::posix_time::seconds(1));}

 return 0;
}
