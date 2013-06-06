

#include <iostream>
#include <boost/program_options.hpp>

namespace po=boost::program_options;

int main(int argc, char** argv){
  po::options_description desc("./main ");
  std::string infile;
  std::string ofile;

  desc.add_options()
    ("help", "produce help message")
    ("input,i",po::value<std::string>(&infile)->required(), "input point cloud ")
    ("output,o",po::value<std::string>(&ofile)->required(), "output pcd ")
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




return 0;
}
