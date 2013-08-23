/*
 * spectrolab3d_frame_to_png.cpp
 * Render a Spectroscan3D frame to as a png image using frame intensity 
 * data.
 */ 
#include <iostream>
#include <boost/program_options.hpp>

#include <spectrolab/spectroscan_3d.h>
#include <pcl/io/png_io.h>

namespace po=boost::program_options;


void cvtToAmplitudeImage(const spectrolab::Scan& scan, std::vector<uint8_t>& img_data){

	img_data.resize(scan.cols()*scan.rows());
	for(size_t i=0; i< img_data.size(); i++){
		float val = scan[i].amplitude;
		val = val/1024.0f*255.0f;
		img_data[i]=val;
	}
}


int main(int argc, char** argv){
  po::options_description desc("./spectroscan3d_frame_to_png frame.bin output.png");
  std::string infile;
  std::string ofile;

  desc.add_options()
    ("help", "produce help message")
    ("input,i",po::value<std::string>(&infile)->required(), "input frame ")
    ("output,o",po::value<std::string>(&ofile)->required(), "output png ")
  ;

  po::positional_options_description p;
  p.add("input",1);
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

 spectrolab::Scan scan(128, 256);

 if (!scan.load(infile)){
	 std::cout << "Failed to load scan \n";
 }

 std::vector<uint8_t> img_data;
 cvtToAmplitudeImage(scan, img_data);
 pcl::io::saveCharPNGFile(ofile, img_data.data(), scan.cols(),scan.rows(),1);

return 0;
}
