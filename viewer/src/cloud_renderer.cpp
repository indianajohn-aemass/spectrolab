/*
 * cloud_renderer.cpp
 *
 *  Created on: Jun 14, 2013
 *      Author: asher
 */

#include <pcl/visualization/cloud_renderer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <pcl/visualization/bw_color_handler.h>

pcl::visualization::CloudRendererRange::CloudRendererRange(std::string field) :
	valid_grabber_(false), field_name_(field){
	description_= "Color by ";
	description_ = description_+field + " field";
}

bool pcl::visualization::CloudRendererRange::setup(
		boost::shared_ptr<Grabber>& grabber) {

	valid_grabber_ =false;
	typedef void (sig_cb) ( const sensor_msgs::PointCloud2::ConstPtr&);

	if ( !grabber->providesCallback<sig_cb>() ) return  false;

	valid_grabber_=true;
	connection_ = this->connection_ = grabber->registerCallback<sig_cb>(
				boost::bind(&CloudRendererRange::grabberCB, this, _1));
	return true;
}

void pcl::visualization::CloudRendererRange::disconnect(){
	connection_.disconnect();
	valid_grabber_=false;
	cloud_.reset();
}
void pcl::visualization::CloudRendererRange::renderNew() {
	if(!valid_grabber_) return;
	boost::unique_lock<boost::mutex> lock( cloud_mutex_);
	if (cloud_==NULL) return;
	pcl::visualization::PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2>::Ptr chand(
			new pcl::visualization::PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2> (cloud_,field_name_) );


	Eigen::Vector4f origin(0,0,0,1);
	Eigen::Quaternionf rot = Eigen::Quaternionf::Identity();
	this->visualizer_->removeAllPointClouds();
	this->visualizer_->addPointCloud(cloud_,chand, origin, rot);
	widget_->update();
	cloud_.reset();
}

void pcl::visualization::CloudRendererRange::grabberCB(
		const sensor_msgs::PointCloud2::ConstPtr& cloud) {
	{
		boost::unique_lock<boost::mutex> lock( cloud_mutex_);
		cloud_ = cloud;
	}
	emit update();
}



/************************************************************************/

pcl::visualization::CloudRendererBW::CloudRendererBW( ) :
	valid_grabber_(false){
	description_= "Black/White intensity coloring";
}

bool pcl::visualization::CloudRendererBW::setup(
		boost::shared_ptr<Grabber>& grabber) {

	valid_grabber_ =false;
	typedef void (sig_cb) ( const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&);

	valid_grabber_ = grabber->providesCallback<sig_cb>() ;
	if (!valid_grabber_) return  false;

	connection_ = this->connection_ = grabber->registerCallback<sig_cb>(
				boost::bind(&CloudRendererBW::grabberCB, this, _1));
	return true;
}

void pcl::visualization::CloudRendererBW::disconnect(){
	connection_.disconnect();
	valid_grabber_=false;
	cloud_.reset();
}
void pcl::visualization::CloudRendererBW::renderNew() {
	if(!valid_grabber_) return;
	boost::unique_lock<boost::mutex> lock( cloud_mutex_);
	if (cloud_==NULL) return;
	pcl::visualization::PointCloudIntensityHandler<PointXYZI>::Ptr chand(
			new pcl::visualization::PointCloudIntensityHandler<PointXYZI> (cloud_) );

	this->visualizer_->removeAllPointClouds();
	this->visualizer_->addPointCloud<pcl::PointXYZI>(cloud_,*chand);
	this->visualizer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4);
	widget_->update();
	cloud_.reset();
}

void pcl::visualization::CloudRendererBW::grabberCB(
		const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) {
	{
		boost::unique_lock<boost::mutex> lock( cloud_mutex_);
		cloud_ = cloud;
	}
	emit update();
}

void pcl::visualization::CloudRendererBW::setCloud(
		const sensor_msgs::PointCloud2ConstPtr& cloud) {
	pcl::PointCloud<pcl::PointXYZI>::Ptr tcloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*cloud, *tcloud);
	cloud_=tcloud;
}
