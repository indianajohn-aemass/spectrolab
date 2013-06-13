/*
 * recorder.h
 *
 *  Created on: Jun 10, 2013
 *      Author: asher
 */

#ifndef RECORDER_H_
#define RECORDER_H_

#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>

namespace pcl{

	class Recorder{
	public:
		Recorder(std::string description="" ): frame_idx_(0),
									description_(description){}
		virtual ~Recorder(){}

		virtual bool setGrabber(const boost::shared_ptr<Grabber>& grabber)=0;
		void setOutput(std::string output_directory,
					   std::string root_name , size_t frame_num=0);

		virtual bool hasValidGrabber()=0;
		virtual bool isRecording()=0;
		virtual void start()=0;
		virtual void stop()=0;
		std::string  getDescription(){return description_;}
	protected:
		std::string description_;
		std::string movie_dir_;
		std::string root_name_;
		size_t frame_idx_;
		std::string genNextFileName();
	};

}

#endif /* RECORDER_H_ */
