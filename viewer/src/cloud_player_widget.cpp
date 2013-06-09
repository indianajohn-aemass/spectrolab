/*
 * cloud_player_widget.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: asher
 */

#include <pcl/visualization/cloud_player_widget.h>
#include <pcl/io/movie_grabber.h>

pcl::visualization::CloudPlayerWidget::CloudPlayerWidget(QWidget* parent, Qt::WindowFlags f):
	QWidget(parent, f),
	current_renderer_idx_(0),
	playing_(false),
	is_movie_grabber_(false),
	pcl_visualizer_(NULL) {
  ui_.setupUi(this);
  pcl_visualizer_ = new pcl::visualization::PCLVisualizer ("", false);
  ui_.qvtkwidget->SetRenderWindow(pcl_visualizer_->getRenderWindow ());
  pcl_visualizer_->setupInteractor (ui_.qvtkwidget->GetInteractor (), ui_.qvtkwidget->GetRenderWindow ());
  ui_.qvtkwidget->update ();


  renderers_.push_back(new CloudRendererRange("x", ui_.qvtkwidget, pcl_visualizer_));
  renderers_.push_back(new CloudRendererRange("y", ui_.qvtkwidget, pcl_visualizer_));
  renderers_.push_back(new CloudRendererRange("z", ui_.qvtkwidget, pcl_visualizer_));

  QObject::connect(ui_.button_play_pause, SIGNAL(clicked()),
		  	  this, SLOT(playPause()) );
  QObject::connect(ui_.button_record, SIGNAL(clicked()),
		  	  this, SLOT(record()) );

  this->setWindowTitle("Cloud Player");

  this->pcl_visualizer_->registerKeyboardCallback(boost::bind(&CloudPlayerWidget::keyCB, this, _1));
  resetView();
 }

pcl::visualization::CloudPlayerWidget::~CloudPlayerWidget() {
}

void pcl::visualization::CloudPlayerWidget::setGrabber(
		boost::shared_ptr<Grabber>& grabber) {

	this->grabber_ = grabber;
	this->renderers_[current_renderer_idx_]->setup(grabber);
	connect(this->renderers_[current_renderer_idx_], SIGNAL(update()), this, SLOT(updateCloud()));

	is_movie_grabber_  =  (dynamic_cast<MovieGrabber*>( &(*grabber)) !=NULL);

	this->ui_.button_play_pause->setEnabled(true);
	this->ui_.button_record->setEnabled(true);

	if (is_movie_grabber_) enablePlayback();
	else disablePlayback();
}

void pcl::visualization::CloudPlayerWidget::addCloudRenderer(
		CloudRenderer* renderer) {
	renderers_.push_back(renderer);
	//todo update menu list
}



void pcl::visualization::CloudPlayerWidget::playPause( ) {
	if (playing_){ //now pause
		this->ui_.button_play_pause->setIcon( QIcon (":/viewer/imgs/play.png"));
		playing_ = false;
		grabber_->stop();
	}
	else{
		this->ui_.button_play_pause->setIcon(QIcon (":/viewer/imgs/pause.png"));
		playing_ = true;
		grabber_->start();
	}
}

void pcl::visualization::CloudPlayerWidget::record( ) {
}

void pcl::visualization::CloudPlayerWidget::resetView( ) {
#if ( ( PCL_MAJOR_VERSION >=1) && (  PCL_MINOR_VERSION > 6) )
	pcl_visualizer_->setCameraPosition (0, 0, -2,
    							   0, 0, 1,
    							   0,-1,0);
#else
	pcl_visualizer_->setCameraPose (0, 0, -2,
    					   0, 0, 1,
    					  0,-1,0);
#endif
}

void pcl::visualization::CloudPlayerWidget::enablePlayback() {
	MovieGrabber* mg = dynamic_cast<MovieGrabber*>( &(*grabber_));
	QObject::connect(ui_.progress_bar, SIGNAL(valueChanged(int)), this, SLOT(sliderValueChanged(int)));
	this->ui_.progress_bar->setEnabled(true);
	this->ui_.progress_bar->setMaximum(mg->getFrameCount());
	progress_connection_ = this->grabber_->registerCallback<MovieGrabber::sig_frame_num_cb>(
			boost::bind(&CloudPlayerWidget::progressUpdate, this , _1, _2));
}

void pcl::visualization::CloudPlayerWidget::disablePlayback() {
	this->ui_.progress_bar->setEnabled(false);
	progress_connection_.disconnect();
}

void pcl::visualization::CloudPlayerWidget::keyCB(
		const pcl::visualization::KeyboardEvent& e) {

}

void pcl::visualization::CloudPlayerWidget::sliderValueChanged(int val) {
	MovieGrabber* mg = dynamic_cast<MovieGrabber*>( &(*grabber_));
	mg->setFrameNumber(val);
	mg->playOneFrame();
}

void pcl::visualization::CloudPlayerWidget::updateCloud() {
	this->renderers_[current_renderer_idx_]->renderNew();
}

void pcl::visualization::CloudPlayerWidget::progressUpdate(size_t frame_num, size_t frame_total) {
	MovieGrabber* mg = dynamic_cast<MovieGrabber*>( &(*grabber_));
	this->ui_.progress_bar->setMinimum(0);
	this->ui_.progress_bar->setMaximum(frame_total);
	this->ui_.progress_bar->setValue(frame_num);
}


#include <pcl/visualization/point_cloud_handlers.h>

pcl::visualization::CloudRendererRange::CloudRendererRange(std::string field, QVTKWidget* widget, PCLVisualizer * visualizer) :
	CloudRenderer(widget, visualizer),
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
	this->connection_ = grabber->registerCallback<sig_cb>(
			boost::bind(&CloudRendererRange::grabberCB, this, _1));

	return true;
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
