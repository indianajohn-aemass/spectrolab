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

template<typename PointT>
inline void pcl::Spectroscan3DGrabber::rangeImageToCloud(
		const spectrolab::RangeImage& rimg, pcl::PointCloud<PointT>& cloud,
		double range_resolution, double x_focal, double y_focal) {

	cloud.resize(rimg.shape()[0]*rimg.shape()[1]);
	cloud.width = rimg.shape()[1];
	cloud.height = rimg.shape()[0];
	cloud.sensor_origin_ << 0,0,0,1;
	cloud.sensor_orientation_ = Eigen::Quaternionf::Identity();

	float mx = rimg.shape()[1]/2.0f;
	float my = rimg.shape()[0]/2.0f;
	x_focal= x_focal*mx;
	y_focal = y_focal*my;
	//TODO double check that am the organization of the range image and the point cloud matches
	for(size_t r=0, idx=0; r< rimg.shape()[0]; r++){
		for(size_t c=0; c< rimg.shape()[1]; c++,idx++){
			double range = (float) rimg[r][c] ;
			range*=range_resolution;
			cloud[idx].z= range;
			float dx = c-mx;
			float dy = r-my;
			cloud[idx].x= dx*range/x_focal;
			cloud[idx].y= dy*range/y_focal;
		}
	}
	PointT& pt_delimeter = cloud.at(0,rimg.shape()[0]-1);
	PointT& pt_frame_count = cloud.at(1,rimg.shape()[0]-1);
	pt_delimeter.x =pt_delimeter.y = pt_delimeter.z =
			pt_frame_count.x= pt_frame_count.y, pt_frame_count.z= std::numeric_limits<float>::quiet_NaN();
}

pcl::Spectroscan3DGrabber::Spectroscan3DGrabber(std::string ipaddress) :
		camera_(boost::asio::ip::address::from_string(ipaddress)){
	camera_.regsiterCallBack( boost::bind(&Spectroscan3DGrabber::frameCB, this, _1, _2) );
	xyzi_cb_ = this->createSignal<sig_cb_xyzi_cloud>();
	img_cb_ = this->createSignal<spectrolab::SpectroScan3D::sig_camera_cb>();
	xyz_cb_ = this->createSignal<sig_cb_xyz_cloud>();

	std::cout << "X focal length "<< settings_.x_focal_length << " y focal " << settings_.y_focal_length << " \n";
}


void
pcl::Spectroscan3DGrabber::start(){
	camera_.start();
}

void
pcl::Spectroscan3DGrabber::stop(){
	camera_.stop();
}

bool
pcl::Spectroscan3DGrabber::open(std::string scanner_ip){
	return camera_.open(boost::asio::ip::address::from_string(scanner_ip));
}

void pcl::Spectroscan3DGrabber::frameCB(const spectrolab::RangeImage& rimg,
		const spectrolab::IntensityImage& intensity_img) {
	std::cout << "Frame callback called\n";
	if (!img_cb_->empty()){
		(*img_cb_)(rimg, intensity_img);
	}
	if (!xyz_cb_->empty()){
		PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
		rangeImageToCloud(rimg, *xyz, settings_.range_resolution, settings_.x_focal_length, settings_.y_focal_length);

		(*xyz_cb_)(xyz);
	}
	if (!xyzi_cb_->empty()){
		PointCloud<pcl::PointXYZI>::Ptr xyzi(new pcl::PointCloud<pcl::PointXYZI>);
		rangeImageToCloud(rimg, *xyzi, settings_.range_resolution, settings_.x_focal_length, settings_.y_focal_length);

		for(size_t r=0, idx=0; r< intensity_img.shape()[0]; r++){
			for(size_t c=0; c< intensity_img.shape()[1]; c++,idx++){
				double amp = ((float) intensity_img[r][c]);
				(*xyzi)[idx].intensity= amp/1024.0f;
			}
		}
		(*xyzi).at(0,intensity_img.shape()[0]-1).intensity=0;
		(*xyzi).at(1,intensity_img.shape()[0]-1).intensity=0;
		(*xyzi_cb_)(xyzi);
	}
}

void pcl::Spectroscan3DGrabber::signalsChanged() {
	xyzi_cb_ = this->find_signal<sig_cb_xyzi_cloud>();
	img_cb_ = this->find_signal<spectrolab::SpectroScan3D::sig_camera_cb>();
	xyz_cb_ = this->find_signal<sig_cb_xyz_cloud>();
}

pcl::Spectroscan3DGrabber::Settings::Settings(): range_resolution(0.00625),
			x_focal_length( 1.0f/tan(30.0/180.0f*M_PI)), y_focal_length(1.0f/tan(15.0/180.0f*M_PI)),
			cx(256.0f/2.0f), cy(64) {

}
