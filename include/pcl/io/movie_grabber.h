/*
 * movie_grabber.h
 *
 *  Created on: Jun 8, 2013
 *      Author: asher
 */

#ifndef __PCL_IO_MOVIE_GRABBER__
#define __PCL_IO_MOVIE_GRABBER__

#include <pcl/io/grabber.h>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

namespace pcl{

/** \brief Grabber interface for PCL 1.x device drivers
  * \author Suat Gedikli <gedikli@willowgarage.com>
  * \ingroup io
  */
class PCL_EXPORTS MovieGrabber : public Grabber{

public:
	MovieGrabber( const boost::filesystem::path move_folder, std::string ext=".pcd");

	virtual ~MovieGrabber() throw(){}

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

    void playOneFrame();

    void setFramesPerSecond( float frameframe);

    size_t getFrameCount();
    size_t getCurrentFrame();
    void setFrameNumber( size_t frame_number);

    std::string getMovieDir();

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

    bool running_;

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
	 */
	virtual void handleCloud( const sensor_msgs::PointCloud2ConstPtr& cloud,
				const Eigen::Vector4f& origin, const Eigen::Quaternionf& rot);
};

}


#endif /* MOVIE_GRABBER_H_ */
