/*
 * spectrolab_io.h
 *
 *  Created on: May 5, 2013
 *      Author: Adam Stambler
 */

#ifndef _PCL_SPECTROLAB_IO_H_
#define _PCL_SPECTROLAB_IO_H_

#include <pcl/io/grabber.h>


namespace spectrolab{
	class LidarCamera;
}

namespace pcl{

	struct PointXYZ;
	struct PointXYZI;
	template <typename T> class PointCloud;

	class PCL_EXPORTS SpectrolabGrabber : public Grabber{
	public:

		SpectrolabGrabber();
		SpectrolabGrabber(std::string uri);

		virtual ~SpectrolabGrabber() throw();

		bool open(std::string uri);


		//define callback signature typedefs
		typedef void (sig_cb_xyz_cloud) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
		typedef void (sig_cb_xyzi_cloud) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);

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
		getName () const {return "spectrolab";}

		/** \brief Indicates whether the grabber is streaming or not. This value is not defined for triggered devices.
		* \return true if grabber is running / streaming. False otherwise.
		*/
		virtual bool
		isRunning () const;

		/** \brief returns fps. 0 if trigger based. */
		virtual float
		getFramesPerSecond () const;


	private:
		void init();
		spectrolab::LidarCamera* camera_;
	};
}



#endif /* SPECTROLAB_IO_H_ */
