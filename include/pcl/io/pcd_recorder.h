/*
 * pcd_recorder.h
 *
 *  Created on: Jun 11, 2013
 *      Author: asher
 */

#ifndef PCD_RECORDER_H_
#define PCD_RECORDER_H_
#include <pcl/io/recorder.h>
#include <sensor_msgs/PointCloud2.h>

namespace pcl{
class PCDRecorder : public Recorder{
public:
	PCDRecorder();
	virtual bool setGrabber(const boost::shared_ptr<Grabber>& grabber);
	virtual bool hasValidGrabber();
	virtual bool isRecording();
	virtual void start();
	virtual void stop();

protected:
	boost::shared_ptr<Grabber> grabber_;
	boost::signals2::connection connection_;
	bool valid_grabber_;
	void cloudCB(const typename sensor_msgs::PointCloud2ConstPtr& cloud);
};

}


#endif /* PCD_RECORDER_H_ */
