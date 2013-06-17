/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef PCL_IO_RECORDER_H_
#define PCL_IO_RECORDER_H_

#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>

namespace pcl{

/** \brief Abstract base class for recording streams of data from a PCL grabber
  * \author Adam Stambler <adasta@gmail.com>
  * \ingroup io
  */
	class PCL_EXPORTS Recorder{
	public:
		Recorder(std::string description="" ): frame_idx_(0),
									description_(description){}
		virtual ~Recorder(){}

    /** \brief Set input grabber.*/
		virtual bool
		setGrabber(const boost::shared_ptr<Grabber>& grabber)=0;

    /** \brief set ouput directory and root name for frames*/
		void
		setOutput(std::string output_directory,
					   std::string root_name , size_t frame_num=0);

    /** \brief returns true if grabber is valid*/
		virtual bool
		hasValidGrabber()=0;

    /** \brief returns true if recording published grabber frames*/
		virtual bool
		isRecording()=0;

    /** \brief starts recording grabber frames if valid grabber is set*/
		virtual void
		start()=0;

    /** \brief stops recording frames*/
		virtual void
		stop()=0;

    /** \brief returns description of recorder*/
		std::string
		getDescription(){return description_;}
	protected:
		std::string description_;
		std::string movie_dir_;
		std::string root_name_;
		size_t frame_idx_;

    /** \brief generates next file path for saving stream frame*/
		std::string genNextFileName();
	};

}

#endif /* RECORDER_H_ */
