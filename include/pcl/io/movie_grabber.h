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


#ifndef PCL_IO_MOVIE_GRABBER_H_
#define PCL_IO_MOVIE_GRABBER_H_

#include <pcl/io/grabber.h>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

namespace pcl{

/** \brief Grabber for playing directories of files
  * \author Adam Stambler <adasta@gmail.com>
  * \ingroup io
  */
class PCL_EXPORTS MovieGrabber : public Grabber{

public:
	MovieGrabber( const boost::filesystem::path move_folder, std::string ext=".pcd");

	virtual ~MovieGrabber() throw(){
		if (running_) stop();
		io_thread_.join();
	}

	typedef boost::shared_ptr<MovieGrabber> Ptr;

	/** \brief For devices that are streaming, the streams are started by calling this method.
      *        Trigger-based devices, just trigger the device once for each call of start.
      */
    virtual void
    start ();

    /** \brief For devices that are streaming, the streams are stopped.
      *        This method has no effect for triggered devices.
      */
    virtual void
    stop ();

    /** \brief returns the name of the concrete subclass.
      * \return the name of the concrete driver.
      */
    virtual std::string
    getName () const {return "MovieGrabber";};

    /** \brief Indicates whether the grabber is streaming or not. This value is not defined for triggered devices.
      * \return true if grabber is running / streaming. False otherwise.
      */
    virtual bool
    isRunning () const;

    /** \brief returns fps. 0 if trigger based. */
    virtual float
    getFramesPerSecond () const;

    /** \brief triggers publishing of one frame*/
    void
    playOneFrame();

    /** \brief Set the playback rate in fps*/
    void
    setFramesPerSecond( float frame_rate_fps);

    /** \brief returns total number of frames in move*/
    size_t
    getFrameCount();

    /** \brief returns current frame idx*/
    size_t
    getCurrentFrame();

    /** \brief set current frame idx*/
    void
    setFrameNumber( size_t frame_number);

    /** \brief returns directory of movie frames */
    std::string
    getMovieDir();

	typedef void (sig_pointcloud_cb) ( const sensor_msgs::PointCloud2::ConstPtr&);
	typedef void (sig_xyz_cb) ( const PointCloud<PointXYZ>::ConstPtr&);
	typedef void (sig_xyzi_cb) ( const PointCloud<PointXYZI>::ConstPtr&);
	typedef void (sig_xyzrgb_cb) ( const PointCloud<PointXYZRGB>::ConstPtr&);
	typedef void (sig_frame_num_cb) (  size_t  frame_num,  size_t total_frames);

private:
	boost::signals2::signal<sig_pointcloud_cb>* cloud_cb_;
	boost::signals2::signal<sig_xyz_cb>* xyz_cb_;
	boost::signals2::signal<sig_xyzi_cb>* xyzi_cb_;
	boost::signals2::signal<sig_xyzrgb_cb>* xyzrgb_cb_;
	boost::signals2::signal<sig_frame_num_cb>* frame_num_cb_;

    bool running_; //flags if movie is running

    boost::thread io_thread_;
    boost::mutex frame_mutex_;

    std::vector<std::string> file_names_;
    size_t frame_idx_;

    float frame_rate_;
    uint32_t sleep_ms_;

    boost::filesystem::path movie_dir_;

    void runIO();

protected:
    virtual void handleFile( const std::string& file);

	/*
	 * must be called by handle file
	 * publishes registered point clouds
	 */
	virtual void handleCloud( const sensor_msgs::PointCloud2ConstPtr& cloud,
				const Eigen::Vector4f& origin, const Eigen::Quaternionf& rot);
};

}


#endif /* MOVIE_GRABBER_H_ */
