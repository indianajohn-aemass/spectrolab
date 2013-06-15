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
#include <pcl/common/io.h>
#include <pcl/exceptions.h>


#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem.hpp>

pcl::SpectroscanSettings::SpectroscanSettings():
		 	range_resolution(0.00625), range_offset(0),
			x_angle_delta(0.1385/180*M_PI), y_angle_delta(.209/180*M_PI),
			min_range(0), max_range(20){
}

bool pcl::SpectroscanSettings::load(std::string fname) {
    using boost::property_tree::ptree;
    ptree pt;

    double t_max_range, t_min_range,t_range_offset,
    	 t_y_ange_delta , t_x_ange_delta;

    // Load the ini file into the property tree. If reading fails
    // (cannot open file, parse error), an exception is thrown.
    try{
        read_ini(fname, pt);
        t_max_range = pt.get<double>("max_range");
        t_min_range = pt.get<double>("min_range");
        t_range_offset = pt.get<double>("range_offset");
        t_y_ange_delta = pt.get<double>("y_angle_delta");
        t_x_ange_delta = pt.get<double>("x_angle_delta");
    }
    catch(std::exception& e){
    	pcl::console::print_error("[SpectroscanSettings::load] Error : %s", e.what());
    	return false;
    }
    range_offset = t_range_offset;
    max_range = t_max_range;
    min_range = t_min_range;
    x_angle_delta = t_x_ange_delta;
    y_angle_delta = t_y_ange_delta;
	return true;
}

void pcl::SpectroscanSettings::save(std::string ofname) {
		boost::filesystem::path opath(ofname);
		opath.replace_extension(".ini");
	   // Create an empty property tree object
	   using boost::property_tree::ptree;
	   ptree pt;

	   pt.put("max_range", max_range);
	   pt.put("min_range", min_range);
	   pt.put("range_offset", range_offset);
	   pt.put("y_angle_delta", y_angle_delta);
	   pt.put("x_angle_delta", x_angle_delta);

	   // Write the property tree to the XML file.
	   write_ini(opath.string(), pt);
}


template<typename PointT>
inline void pcl::rangeImageToCloud(
		const spectrolab::Scan& scan, pcl::PointCloud<PointT>& cloud,
		const SpectroscanSettings& settings) {

	cloud.resize(scan.cols()*scan.rows());
	cloud.width = scan.cols();
	cloud.height = scan.rows();
	cloud.sensor_origin_ << 0,0,0,1;
	cloud.sensor_orientation_ = Eigen::Quaternionf::Identity();
	cloud.is_dense=false;

	float mx = scan.cols()/2.0f;
	float my = scan.rows()/2.0f;


	for(size_t r=0, idx=0; r< scan.rows(); r++){
		for(size_t c=0; c< scan.cols(); c++,idx++){
			double range = (float) scan[idx].range; //scan(r,c).range ;
			range=settings.range_resolution *range + settings.range_offset;
			if ( (range> settings.max_range) || (range< settings.min_range) ){
				cloud[idx].x= cloud[idx].y, cloud[idx].z= std::numeric_limits<float>::quiet_NaN();
				continue;
			}
			cloud[idx].z= range;
			float dx = c-mx;
			cloud[idx].x = sin(dx*settings.x_angle_delta)*range;
			float dy = r-my;
			cloud[idx].y = sin(dy*settings.y_angle_delta)*range;
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

void passthrough_filter( const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >& in,
						       boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >& out){
	out=in;
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
	cloud_cb_ = this->createSignal<sig_cb_cloud>();
	filter_= passthrough_filter;
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
	PointCloud<pcl::PointXYZI>::Ptr xyzi(new pcl::PointCloud<pcl::PointXYZI>);
	PointCloud<pcl::PointXYZI>::Ptr filtered_xyzi(new pcl::PointCloud<pcl::PointXYZI>);

	rangeImageToCloud(*scan, *xyzi, settings_);

	for(size_t r=0, idx=0; r< scan->rows(); r++){
		for(size_t c=0; c< scan->cols(); c++,idx++){
			float amp = ((float) (*scan)[idx].amplitude);
			(*xyzi)[idx].intensity= amp/1024.0f;
		}
	}
	(*xyzi).at(0,scan->rows()-1).intensity=0;
	(*xyzi).at(1,scan->rows()-1).intensity=0;

	filter_(xyzi, filtered_xyzi);

	if (!xyzi_cb_->empty()){
		(*xyzi_cb_)(filtered_xyzi);
	}
	if (!xyz_cb_->empty()){
		PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*filtered_xyzi, *xyz);
		(*xyz_cb_)(xyz);
	}
	if (!cloud_cb_->empty()){
		sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
		pcl::toROSMsg(*filtered_xyzi, *cloud);
		(*cloud_cb_)(cloud);
	}
}

void pcl::Spectroscan3DGrabber::signalsChanged() {
	cloud_cb_ = this->find_signal<sig_cb_cloud>();
	xyz_cb_ = this->find_signal<sig_cb_xyz_cloud>();
	xyzi_cb_ = this->find_signal<sig_cb_xyzi_cloud>();
	img_cb_ = this->find_signal<spectrolab::SpectroScan3D::sig_camera_cb>();
}

pcl::Spectroscan3DMovieGrabber::Spectroscan3DMovieGrabber(
		boost::filesystem::path movie_dir, bool use_extention) :
					MovieGrabber(movie_dir, use_extention ? ".ssi" : ""),
					img_cb_(NULL)  {
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
	rangeImageToCloud(*scan, *xyzi, settings_);

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

bool pcl::Spectroscan3DRecorder::setGrabber( const boost::shared_ptr<Grabber>& grabber) {
	valid_grabber_ =  grabber->providesCallback<spectrolab::SpectroScan3D::sig_camera_cb>();
	grabber_ = grabber;
	return valid_grabber_;
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
	scan->save(genNextFileName()+".ssi");
}

