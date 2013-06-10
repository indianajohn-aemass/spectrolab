/*
 * viewer_application.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: Adam Stambler
 */

#include "viewer_application.h"
#include <pcl/io/spectroscan_3d_io.h>
#include <pcl/io/file_grabber.h>
#include <boost/filesystem.hpp>
#include <QFileDialog>
#include <QInputDialog>
#include <QErrorMessage>

#include <pcl/io/movie_grabber.h>

SpectolabViewer::SpectolabViewer() :
settings_("Open Perception", "Spectrolab Viewer"),
frame_rate_(5){
	ui_.setupUi(this);
	this->ui_.centralwidget->setLayout(new QVBoxLayout);
	cplayer_= new pcl::visualization::CloudPlayerWidget();
	this->ui_.centralwidget->layout()->addWidget(cplayer_);
	cplayer_->addCloudRenderer(new pcl::visualization::CloudRendererRange("intensity"));
	cplayer_->setRenderer(cplayer_->getNumRenderers()-1);

	QObject::connect(ui_.action_movie_load, SIGNAL(triggered()), this , SLOT(loadMovie()) );
	QObject::connect(ui_.action_movie_frame_rate, SIGNAL(triggered()), this , SLOT(setFrameRate()) );
	QObject::connect(ui_.actionLoad_scan, SIGNAL(triggered()), this , SLOT(loadScan()) );

	QObject::connect(ui_.action_connect, SIGNAL(triggered()), this , SLOT(spectroscan3dConnect()) );
	QObject::connect(ui_.action_settings, SIGNAL(triggered()), this , SLOT(spectroscan3dSettings()) );

	this->setWindowTitle("Spectrolab Viewer");

	if (settings_.contains("Frame Rate")){
		bool ok;
		frame_rate_ = settings_.value("Frame Rate", 5).toFloat(&ok);
	}
}

SpectolabViewer::~SpectolabViewer() {
	delete cplayer_;
	settings_.setValue("Frame Rate", frame_rate_);
}
#include <qlabel.h>

void SpectolabViewer::loadMovie() {
	QString fileName = QFileDialog::getOpenFileName(this, tr("Select a frame of the movie"),
	                                                settings_.value("data_path","").toString(),
	                                                 tr("Files (*.pcd *.bin)"));
	if (fileName.size() ==0) return;
	boost::filesystem::path frame_path = fileName.toAscii().data();
	QString save_path = frame_path.parent_path().c_str();
	settings_.setValue("data_path", save_path);

	if (frame_path.extension()==".pcd"){
		grabber_.reset(new pcl::MovieGrabber( frame_path.parent_path(), frame_path.extension().string() ) );
	}
	else{
		grabber_.reset(new pcl::Spectroscan3DMovieGrabber( frame_path.parent_path() ) );
	}
	dynamic_cast<pcl::MovieGrabber*>( grabber_.get())->setFramesPerSecond(frame_rate_);
	this->cplayer_->setGrabber(grabber_);
}

void SpectolabViewer::loadScan() {
	QString fileName = QFileDialog::getOpenFileName(this, tr("Select PCD or Spectroscan image frame"),
													 settings_.value("data_path","").toString(),
	                                                 tr("Files (*.pcd *.bin)"));
	if (fileName.size() ==0) return;
	boost::filesystem::path frame_path = fileName.toAscii().data();

	QString save_path = frame_path.parent_path().c_str();
	settings_.setValue("data_path", save_path);

	if (frame_path.extension()==".pcd"){
		grabber_.reset(new pcl::MovieGrabber( frame_path.string(), frame_path.extension().string() ) );
	}
	else{
		grabber_.reset(new pcl::Spectroscan3DMovieGrabber( frame_path.string()) );
	}
	this->cplayer_->setGrabber(grabber_);
	this->cplayer_->playPause();
}

void SpectolabViewer::setFrameRate(){
	bool ok;
	int val =QInputDialog::getInteger(NULL, "How many frames per second should movies play?", "FPS: ", frame_rate_, 1, 30, 1, &ok);
	if (ok) frame_rate_=val;

	pcl::MovieGrabber* grabber = 	dynamic_cast<pcl::MovieGrabber*>( grabber_.get());
	if (grabber!= NULL){
		grabber->setFramesPerSecond(frame_rate_);
	}
}

void SpectolabViewer::spectroscan3dConnect(){

	try{
		grabber_.reset(new pcl::Spectroscan3DGrabber);
		dynamic_cast<pcl::Spectroscan3DGrabber*>( grabber_.get())->setSettings(spectroscan_settings_);
		cplayer_->setGrabber(grabber_);
		QString msg="Opened connection to Spectroscan 3D ";
		ui_.statusbar->showMessage(msg,15000);
	}
	catch( std::exception& e){
		QString msg="Failed to connect to Spectroscan 3D : ";
		msg += e.what();
		ui_.statusbar->showMessage(msg,15000);
	}
}

void SpectolabViewer::spectroscan3dSettings(){

}
