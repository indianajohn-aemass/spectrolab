/*
 * recorder.cpp
 *
 *  Created on: Jun 10, 2013
 *      Author: asher
 */


#include <pcl/io/recorder.h>
#include <boost/filesystem.hpp>

void pcl::Recorder::setOutput(std::string output_directory,
		std::string root_name, size_t frame_num) {
	if(!boost::filesystem::exists(output_directory)){
		boost::filesystem::create_directories(output_directory);
	}
	frame_idx_=frame_num;
	movie_dir_=output_directory;
	root_name_=root_name;
}

std::string pcl::Recorder::genNextFileName() {
	boost::filesystem::path opath = this->movie_dir_;
	char tmp[20];
	sprintf(tmp, "_%06lu", frame_idx_);
	frame_idx_++;
	return ( (opath/root_name_).string()+tmp);
}
