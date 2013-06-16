/*
 * pcd_recorder.cpp
 *
 *  Created on: Jun 11, 2013
 *      Author: asher
 */

#include <pcl/io/pcd_recorder.h>
#include <pcl/io/pcd_io.h>


pcl::PCDRecorder::PCDRecorder( ) :
		Recorder("Record to PCD"),
		valid_grabber_(false){
}

bool pcl::PCDRecorder::hasValidGrabber() {
	return valid_grabber_;
}

bool pcl::PCDRecorder::isRecording() {
	return connection_.connected();
}

 void pcl::PCDRecorder ::start() {
	if (!valid_grabber_) return;
	typedef void (sig_cb_cloud) (const  sensor_msgs::PointCloud2::ConstPtr&);
	connection_ = grabber_->registerCallback<sig_cb_cloud>(boost::bind(&PCDRecorder::cloudCB, this, _1) );
}

void pcl::PCDRecorder::stop() {
	connection_.disconnect();
}

bool  pcl::PCDRecorder::setGrabber(const boost::shared_ptr<Grabber>& grabber) {
	grabber_ = grabber;
	typedef void (sig_cb_cloud) (const  sensor_msgs::PointCloud2::ConstPtr&);
	valid_grabber_ = grabber_->providesCallback<sig_cb_cloud>();
	return valid_grabber_;
}

void pcl::PCDRecorder::cloudCB(
		const sensor_msgs::PointCloud2::ConstPtr& cloud) {
	pcl::PCDWriter writer;
	writer.writeBinaryCompressed(genNextFileName()+".pcd", *cloud);
}
