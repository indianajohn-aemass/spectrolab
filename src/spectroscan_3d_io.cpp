/*
 * spectrolab_io.cpp
 *
 *  Created on: May 5, 2013
 *      Author: asher
 */

#include <pcl/io/spectroscan_3d_io.h>
#include <spectrolab/spectroscan_3d.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/console/print.h>
#include <pcl/ros/conversions.h>
#include <pcl/exceptions.h>

template<typename PointT>
inline void pcl::rangeImageToCloud(
		const spectrolab::Scan& scan, pcl::PointCloud<PointT>& cloud,
		double range_resolution, double x_focal, double y_focal) {

	cloud.resize(scan.cols()*scan.rows());
	cloud.width = scan.cols();
	cloud.height = scan.rows();
	cloud.sensor_origin_ << 0,0,0,1;
	cloud.sensor_orientation_ = Eigen::Quaternionf::Identity();
	cloud.is_dense=true;

	float mx = scan.cols()/2.0f;
	float my = scan.rows()/2.0f;
	x_focal= x_focal*mx;
	y_focal = y_focal*my;

	for(size_t r=0, idx=0; r< scan.rows(); r++){
		for(size_t c=0; c< scan.cols(); c++,idx++){
			double range = (float) scan[idx].range; //scan(r,c).range ;
			range*=range_resolution;
			cloud[idx].z= range;
			float dx = c-mx;
			float dy = r-my;
			cloud[idx].x= dx*range/x_focal;
			cloud[idx].y= dy*range/y_focal;
		}
	}
	PointT& pt_delimeter = cloud.at(0,scan.rows()-1);
	PointT& pt_frame_count = cloud.at(1,scan.rows()-1);
	pt_delimeter.x =pt_delimeter.y = pt_delimeter.z =
			pt_frame_count.x= pt_frame_count.y, pt_frame_count.z= std::numeric_limits<float>::quiet_NaN();
}

void print_debug_output( const std::string& str){
	pcl::console::print_debug("%s", str.c_str());
}

pcl::Spectroscan3DGrabber::Spectroscan3DGrabber(std::string ipaddress) {

	camera_.setDebugOutput(print_debug_output);
	if (! camera_.open( boost::asio::ip::address::from_string(ipaddress) )){
		throw pcl::IOException("Failed to open Spectroscan 3D Camera");
	}
	camera_.registerCallBack( boost::bind(&Spectroscan3DGrabber::frameCB, this, _1, _2) );
	xyzi_cb_ = this->createSignal<sig_cb_xyzi_cloud>();
	img_cb_ = this->createSignal<spectrolab::SpectroScan3D::sig_camera_cb>();
	xyz_cb_ = this->createSignal<sig_cb_xyz_cloud>();
 }


void
pcl::Spectroscan3DGrabber::start(){
	camera_.start();
}

void
pcl::Spectroscan3DGrabber::stop(){
	camera_.stop();
}

void pcl::Spectroscan3DGrabber::frameCB(const spectrolab::Scan::ConstPtr& scan, time_t scan_time) {
	if (!img_cb_->empty()){
		(*img_cb_)(scan, scan_time);
	}
	if (!xyz_cb_->empty()){
		PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
		rangeImageToCloud(*scan, *xyz, settings_.range_resolution, settings_.x_focal_length, settings_.y_focal_length);
		(*xyz_cb_)(xyz);
	}
	if (!xyzi_cb_->empty()){
		PointCloud<pcl::PointXYZI>::Ptr xyzi(new pcl::PointCloud<pcl::PointXYZI>);
		rangeImageToCloud(*scan, *xyzi, settings_.range_resolution, settings_.x_focal_length, settings_.y_focal_length);

		for(size_t r=0, idx=0; r< scan->rows(); r++){
			for(size_t c=0; c< scan->cols(); c++,idx++){
				float amp = ((float) (*scan)[idx].amplitude);
				(*xyzi)[idx].intensity= amp/1024.0f;
			}
		}
		(*xyzi).at(0,scan->rows()-1).intensity=0;
		(*xyzi).at(1,scan->rows()-1).intensity=0;
		(*xyzi_cb_)(xyzi);
	}
}

void pcl::Spectroscan3DGrabber::signalsChanged() {
	xyzi_cb_ = this->find_signal<sig_cb_xyzi_cloud>();
	img_cb_ = this->find_signal<spectrolab::SpectroScan3D::sig_camera_cb>();
	xyz_cb_ = this->find_signal<sig_cb_xyz_cloud>();
}

pcl::SpectroscanSettings::SpectroscanSettings(): range_resolution(0.00625),
			x_focal_length( 1.0f/tan(30.0/180.0f*M_PI)), y_focal_length(1.0f/tan(15.0/180.0f*M_PI)),
			cx(256.0f/2.0f), cy(64) {

}

pcl::Spectroscan3DMovieGrabber::Spectroscan3DMovieGrabber(
		boost::filesystem::path movie_dir) :
					MovieGrabber(movie_dir, ".bin"), img_cb_(NULL)  {
	img_cb_ = this->createSignal<spectrolab::SpectroScan3D::sig_camera_cb>();
}

void pcl::Spectroscan3DMovieGrabber::handleFile(const std::string& file) {

	spectrolab::Scan::Ptr scan(new spectrolab::Scan(spectrolab::SpectroScan3D::IMG_HEIGHT,
													spectrolab::SpectroScan3D::IMG_WIDTH));
	if (!scan->load(file)){
		pcl::console::print_error("[Spectroscan3DMovieGrabber] Cannot read %s", file.c_str());
		return;
	}
	if (!img_cb_->empty()){
		(*img_cb_)(scan, getCurrentFrame());
	}
	PointCloud<pcl::PointXYZI>::Ptr xyzi(new pcl::PointCloud<pcl::PointXYZI>);
	rangeImageToCloud(*scan, *xyzi, settings_.range_resolution, settings_.x_focal_length, settings_.y_focal_length);

	for(size_t r=0, idx=0; r< scan->rows(); r++){
		for(size_t c=0; c< scan->cols(); c++,idx++){
			float amp = ((float) (*scan)[idx].amplitude);
			(*xyzi)[idx].intensity= amp/1024.0f;
		}
	}
	(*xyzi).at(0,scan->rows()-1).intensity=0;
	(*xyzi).at(1,scan->rows()-1).intensity=0;
	sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
	pcl::toROSMsg<pcl::PointXYZI>(*xyzi, *cloud);
	handleCloud(cloud, Eigen::Vector4f(0,0,0,1), Eigen::Quaternionf::Identity() );
}

pcl::Spectroscan3DRecorder::Spectroscan3DRecorder() :
		Recorder("Record Spectroscan 3D Frames"),
		valid_grabber_(false){
}

bool pcl::Spectroscan3DRecorder::setGrabber(
		const boost::shared_ptr<Grabber>& grabber) {
}

bool pcl::Spectroscan3DRecorder::hasValidGrabber() {
	return valid_grabber_;
}

bool pcl::Spectroscan3DRecorder::isRecording() {
	return connection_.connected();
}

void pcl::Spectroscan3DRecorder::start() {
	if (!valid_grabber_) return;
	connection_ = grabber_->registerCallback<spectrolab::SpectroScan3D::sig_camera_cb>(boost::bind(&Spectroscan3DRecorder::frameCB, this, _1, _2) );
}

void pcl::Spectroscan3DRecorder::stop() {
	connection_.disconnect();
}

void pcl::Spectroscan3DRecorder::frameCB(const spectrolab::Scan::ConstPtr& scan,
		time_t t) {
	scan->save(genNextFileName());
}
