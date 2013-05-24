

#include <iostream>
#include <boost/program_options.hpp>

#include <spectrolab/spectroscan_3d.h>

namespace po=boost::program_options;

int main(int argc, char** argv){
  po::options_description desc("./spectroscan3d_grabframe ");
  std::string infile;
  std::string ofile;

  desc.add_options()
    ("help", "produce help message")
    //("input,i",po::value<std::string>(&infile)->required(), "input point cloud ")
  //  ("output,o",po::value<std::string>(&ofile)->required(), "output pcd ")
       ;

  po::positional_options_description p;
//  p.add("input",1);
 // p.add("output",1);

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


 spectrolab::SpectroScan3D camera;

 std::cout << "About to start \n";
 if (!camera.start()){
	 std::cout << "Failed to start scanner \n";
 }
 std::cout << "Successfully started scanner \n";

 sleep(1);

 std::cout << "saving frame to " << std::endl;


 return 0;
}
