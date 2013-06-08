/*
 * movie_grabber.cpp
 *
 *  Created on: Jun 8, 2013
 *      Author: asher
 */
#include <pcl/io/movie_grabber.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

pcl::MovieGrabber::MovieGrabber(const boost::filesystem::path movie_folder, std::string ext):
	running_(false), frame_idx_(0){
	setFramesPerSecond(30);

	cloud_cb_ = this->createSignal<sig_pointcloud_cb>();
	xyz_cb_ = this->createSignal<sig_xyz_cb>();
	xyzi_cb_ = this->createSignal<sig_xyzi_cb>();
	xyzrgb_cb_ = this->createSignal<sig_xyzrgb_cb>();

	boost::filesystem::directory_iterator diter(movie_folder), dend;
	for(;diter!= dend; diter++){
		if (diter->path().extension()==ext)
			file_names_.push_back(diter->path().string());
	}
	std::sort(file_names_.begin(), file_names_.end());
}


void pcl::MovieGrabber::start() {
	if (running_) return;


	running_=true;
	io_thread_ = boost::thread( boost::bind(&MovieGrabber::runIO, this) );
}

void pcl::MovieGrabber::stop() {
}

bool pcl::MovieGrabber::isRunning() const {
	return running_;
}

float pcl::MovieGrabber::getFramesPerSecond() const {
	return frame_rate_;
}

void pcl::MovieGrabber::setFramesPerSecond(float frame_rate) {
	frame_rate_ = frame_rate;
	sleep_ms_ = 1/frame_rate*1000;
}

size_t pcl::MovieGrabber::getFrameCount() {
	return this->file_names_.size();
}


void pcl::MovieGrabber::setFrameNumber(size_t frame_number) {
	boost::unique_lock<boost::mutex> lock(frame_mutex_);
	frame_idx_ = frame_number;
}

size_t pcl::MovieGrabber::getCurrentFrame() {
	return frame_idx_;
}

std::string pcl::MovieGrabber::getMovieDir() {
	return movie_dir_.string();
}

void pcl::MovieGrabber::runIO() {
	while( (frame_idx_< file_names_.size()) && running_){
		handleFile(file_names_[frame_idx_]);
		boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_ms_));
		boost::unique_lock<boost::mutex> lock(frame_mutex_);
		frame_idx_++;
	}
}

void pcl::MovieGrabber::handleFile(std::string& file) {
	sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
	Eigen::Vector4f  origin;
	Eigen::Quaternionf rot;
	if (pcl::io::loadPCDFile(file, *cloud, origin, rot) <0){
		pcl::console::print_error("[MovieGrabber] Failed to load frame %s", file.c_str());
		return;
	}
	handleCloud(cloud, origin, rot);
}

void pcl::MovieGrabber::handleCloud(
	const sensor_msgs::PointCloud2ConstPtr& cloud,
	const Eigen::Vector4f& origin, const Eigen::Quaternionf& rot) {
	if (!cloud_cb_->empty()) (*cloud_cb_)(cloud);
	if (!xyz_cb_->empty()){
		pcl::PointCloud<pcl::PointXYZ>::Ptr tcloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*cloud, *tcloud );
		tcloud->sensor_orientation_=rot;
		tcloud->sensor_origin_=origin;
		(*xyz_cb_)(tcloud);
	}
	if (!xyzi_cb_->empty()){
		pcl::PointCloud<pcl::PointXYZI>::Ptr tcloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(*cloud, *tcloud );
		tcloud->sensor_orientation_=rot;
		tcloud->sensor_origin_=origin;
		(*xyzi_cb_)(tcloud);
	}
	if (!xyzrgb_cb_->empty()){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*cloud, *tcloud );
		tcloud->sensor_orientation_=rot;
		tcloud->sensor_origin_=origin;
		(*xyzrgb_cb_)(tcloud);
	}
}
