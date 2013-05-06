/*
 * lidar_driver.h
 *
 *  Created on: Apr 12, 2013
 *      Author: asher
 */

#ifndef _SPECTROLABLIDAR_DRIVER_H_
#define _SPECTROLABLIDAR_DRIVER_H_

#include <boost/multi_array.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>


#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>

namespace spectrolab{


	/*
	 * RangeImage
	 * Range Image in meters for the camera
	 * NaN values are invalid points
	 */
	typedef boost::multi_array<float, 2> RangeImage;
	typedef boost::shared_ptr<RangeImage> RangeImgaePtr;

	typedef boost::multi_array<float, 2> IntensityImage;
	typedef boost::shared_ptr<IntensityImage> IntensityImagePtr;


	class LidarCamera{
		public:
		//define callback signature typedefs
		typedef void (sig_camera_cb) (const RangeImgaePtr&, const IntensityImagePtr&);

		bool open(std::string uri);

		bool isRunning();
		void start();
		void stop();

		boost::signals2::connection
		regsiterCallBack(const boost::function<sig_camera_cb>& cb);

		private:
	      boost::thread grabber_thread_;

	};
}


#endif /* LIDAR_DRIVER_H_ */
