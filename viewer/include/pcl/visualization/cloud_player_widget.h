/*
 * cloud_player_widget.h
 *
 *  Created on: Jun 5, 2013
 *      Author: asher
 */

#ifndef CLOUD_PLAYER_WIDGET_H_
#define CLOUD_PLAYER_WIDGET_H_

#include <pcl/io/grabber.h>

namespace pcl {
namespace visualization{
class CloudPlayerWidget {
	public:
		CloudPlayerWidget();
		virtual ~CloudPlayerWidget();

		void setGrabber(boost::shared_ptr<Grabber>& grabber);

	private:
		boost::shared_ptr<Grabber> grabber_;
		bool is_movie_grabber_;
	};
}

} /* namespace adaptive_pushgrasp */
#endif /* CLOUD_PLAYER_WIDGET_H_ */
