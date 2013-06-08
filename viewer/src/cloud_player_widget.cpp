/*
 * cloud_player_widget.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: asher
 */

#include <pcl/visualization/cloud_player_widget.h>
#include <pcl/io/movie_grabber.h>

pcl::visualization::CloudPlayerWidget::CloudPlayerWidget():
	current_renderer_idx_(0),
	playing_(false),
	is_movie_grabber_(false),
	pcl_visualizer_(NULL) {
  ui_.setupUi(this);
  pcl_visualizer_ = new pcl::visualization::PCLVisualizer ("", false);
  ui_.qvtkwidget->SetRenderWindow(pcl_visualizer_->getRenderWindow ());
  pcl_visualizer_->setupInteractor (ui_.qvtkwidget->GetInteractor (), ui_.qvtkwidget->GetRenderWindow ());
  ui_.qvtkwidget->update ();

  renderers_.push_back(new CloudRendererRange("x"));
  renderers_.push_back(new CloudRendererRange("y"));
  renderers_.push_back(new CloudRendererRange("z"));

  QObject::connect(ui_.button_play_pause, SIGNAL(triggered(QAction *)),
		  	  this, SLOT(play_pause(QAction*)) );

 }

pcl::visualization::CloudPlayerWidget::~CloudPlayerWidget() {
}

void pcl::visualization::CloudPlayerWidget::setGrabber(
		boost::shared_ptr<Grabber>& grabber) {

	this->grabber_ = grabber;
	this->renderers_[current_renderer_idx_]->setup(grabber, pcl_visualizer_);
	is_movie_grabber_  =  (dynamic_cast<MovieGrabber*>( &(*grabber)) !=NULL);

	this->ui_.button_play_pause->setEnabled(true);
	this->ui_.button_record->setEnabled(true);

	if (is_movie_grabber_) enablePlayback();
	else disablePlayback();

	this->playPause(NULL);
}

void pcl::visualization::CloudPlayerWidget::addCloudRenderer(
		CloudRenderer* renderer) {
	renderers_.push_back(renderer);
	//todo update menu list
}

void pcl::visualization::CloudPlayerWidget::paintEvent(QPaintEvent* e) {
	renderers_[current_renderer_idx_]->renderNew();
	QWidget::paintEvent(e);
}


void pcl::visualization::CloudPlayerWidget::playPause(QAction* act) {
	if (playing_){ //now pause
		QPixmap icon = QPixmap (":/imgs/pause.png");
		this->ui_.button_play_pause->setIcon(icon);
		playing_ = false;
		grabber_->stop();
	}
	else{
		QPixmap icon = QPixmap (":/imgs/play.png");
		this->ui_.button_play_pause->setIcon(icon);
		playing_ = true;
		grabber_->start();
	}
}

void pcl::visualization::CloudPlayerWidget::forward(QAction* act) {
	MovieGrabber* mg = dynamic_cast<MovieGrabber*>( &(*grabber_));
	mg->setFrameNumber(mg->getCurrentFrame()+1);
}

void pcl::visualization::CloudPlayerWidget::backward(QAction* act) {
	MovieGrabber* mg = dynamic_cast<MovieGrabber*>( &(*grabber_));
	if ( mg->getCurrentFrame()==0) return;
	mg->setFrameNumber(mg->getCurrentFrame()-1);
}

void pcl::visualization::CloudPlayerWidget::record(QAction* act) {
}

void pcl::visualization::CloudPlayerWidget::resetView(QAction* act) {
}

void pcl::visualization::CloudPlayerWidget::enablePlayback() {
	this->ui_.button_backward->setEnabled(true);
	this->ui_.button_forward->setEnabled(true);
	this->ui_.progress_bar->setEnabled(true);
 	progress_connection_ = this->grabber_->registerCallback<MovieGrabber::sig_frame_num_cb>(
			boost::bind(&CloudPlayerWidget::progressUpdate, this , _1, _2));
}

void pcl::visualization::CloudPlayerWidget::disablePlayback() {
	this->ui_.button_backward->setEnabled(false);
	this->ui_.button_forward->setEnabled(false);
	this->ui_.button_backward->setEnabled(false);
	progress_connection_.disconnect();
}

void pcl::visualization::CloudPlayerWidget::progressUpdate(size_t frame_num, size_t frame_total) {
	MovieGrabber* mg = dynamic_cast<MovieGrabber*>( &(*grabber_));
	this->ui_.progress_bar->setMinimum(0);
	this->ui_.progress_bar->setMaximum(frame_total);
	this->ui_.progress_bar->setValue(frame_num);
}


#include <pcl/visualization/point_cloud_handlers.h>

pcl::visualization::CloudRendererRange::CloudRendererRange(std::string field) :
	valid_grabber_(false) {
	description_= "Color by ";
	description_ = description_+field + " field";
}

bool pcl::visualization::CloudRendererRange::setup(
		boost::shared_ptr<Grabber>& grabber, PCLVisualizer* visualizer) {

	valid_grabber_ =false;
	if ( !grabber->providesCallback<sensor_msgs::PointCloud2::ConstPtr>() )
		return  false;
	valid_grabber_=true;
	typedef void (sig_cb) ( const sensor_msgs::PointCloud2::ConstPtr&);

	this->connection_ = grabber->registerCallback<sig_cb>(
			boost::bind(&CloudRendererRange::grabberCB, this, _1));

	return true;
}

void pcl::visualization::CloudRendererRange::renderNew() {
	if(!valid_grabber_) return;
	if (cloud_==NULL) return;
	boost::unique_lock<boost::mutex> lock( cloud_mutex_);

	pcl::visualization::PointCloudGeometryHandlerXYZ<sensor_msgs::PointCloud2>::Ptr
		geo(new pcl::visualization::PointCloudGeometryHandlerXYZ<sensor_msgs::PointCloud2>(cloud_) );

	pcl::visualization::PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2>::Ptr chand(
			new pcl::visualization::PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2> (cloud_,field_name_) );

	Eigen::Vector4f origin(0,0,0,1);
	Eigen::Quaternionf rot = Eigen::Quaternionf::Identity();
	this->visualizer_->addPointCloud(cloud_, geo,chand, origin, rot);
}

void pcl::visualization::CloudRendererRange::grabberCB(
		const sensor_msgs::PointCloud2::ConstPtr& cloud) {
	boost::unique_lock<boost::mutex> lock( cloud_mutex_);
	cloud_ = cloud;
}
