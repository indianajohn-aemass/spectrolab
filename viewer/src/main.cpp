#include <iostream>
#include <boost/program_options.hpp>

#include "viewer_application.h"
#include <pcl/io/movie_grabber.h>
#include <pcl/visualization/cloud_player_widget.h>

namespace po = boost::program_options;

int main(int argc, char** argv) {
  po::options_description desc("./spectrolab_viewer  ");
  std::string infile;

  desc.add_options()
      ("help", "produce help message")
      ;

  po::positional_options_description p;

  po::variables_map vm;
  try {
    po::store(
        po::command_line_parser(argc, argv).options(desc).positional(p).run(),
        vm);
    po::notify(vm);
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 0;
  }

  QApplication app(argc, argv);

  SpectolabViewer spectrolab_viewer;
  spectrolab_viewer.show();

  return app.exec();;
}
