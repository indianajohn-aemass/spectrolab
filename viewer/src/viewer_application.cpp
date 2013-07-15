/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "viewer_application.h"
#include <pcl/io/spectroscan_3d_io.h>
#include <pcl/io/movie_grabber.h>
#include "spectroscan_settings_widget.h"

#include <boost/filesystem.hpp>


#include <QFileDialog>
#include <QInputDialog>
#include <QErrorMessage>
#include <qlabel.h>


SpectolabViewer::SpectolabViewer() :
settings_("Open Perception", "Spectrolab Viewer"),
  frame_rate_(5) , interface_widget_(NULL){
	ui_.setupUi(this);
	this->ui_.centralwidget->setLayout(new QVBoxLayout);
	cplayer_= new pcl::visualization::CloudPlayerWidget();
	this->ui_.centralwidget->layout()->addWidget(cplayer_);
	cplayer_->addCloudRenderer(new pcl::visualization::CloudRendererRange("intensity"));
	cplayer_->setCurrentRenderer(cplayer_->getNumRenderers()-1);
	cplayer_->addCloudRenderer(new pcl::visualization::CloudRendererBW);
	cplayer_->addCloudRenderer (new pcl::visualization::CloudRendererIZ ());

	cplayer_->addRecorder(new pcl::Spectroscan3DRecorder);

	bool ok;
	cplayer_->setCurrentRecorder(settings_.value("recorder_idx", 0).toInt(&ok));
	cplayer_->setCurrentRenderer(settings_.value("renderer_idx", 0).toInt(&ok));

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

	settings_widget_ = new SpectroscanSettingsWidget(&spectroscan_settings_);
	connect(settings_widget_, SIGNAL(settingsApplied()), this, SLOT(spectroscan3dSettingsApplied()));

    connect(ui_.action_cmd_interface, SIGNAL(triggered()), this, SLOT(commandInterface()));

}

SpectolabViewer::~SpectolabViewer() {
	settings_.setValue("Frame Rate", frame_rate_);
	settings_.setValue("renderer_idx", cplayer_->currentRendererIDX());
	settings_.setValue("recorder_idx", cplayer_->currentRecorderIDX());
	settings_widget_->close();
	delete cplayer_;
	delete settings_widget_;
}

void SpectolabViewer::loadMovie() {
	QString fileName = QFileDialog::getOpenFileName(this, tr("Select a frame of the movie"),
	                                                settings_.value("data_path","").toString(),
	                                                 tr("Files (*.pcd  Frame* *.ssi)"));
	if (fileName.size() ==0) return;
	boost::filesystem::path frame_path = fileName.toAscii().data();
	QString save_path =  (char*)  frame_path.parent_path().c_str();
	settings_.setValue("data_path", save_path);

	pcl::MovieGrabber* mg;
	if (frame_path.extension()==".pcd"){
		mg = new pcl::MovieGrabber( frame_path.parent_path(), frame_path.extension().string() );
	}
	else if (frame_path.extension()==".ssi"){
		mg = new pcl::Spectroscan3DMovieGrabber( frame_path.parent_path(),true );
	}
	else{
		mg = new pcl::Spectroscan3DMovieGrabber( frame_path.parent_path(),false );
	}
	if (mg->getFrameCount() ==0 ){
		statusBar()->showMessage("Could no play "+ save_path, 2000);
		delete mg;
		return;
	}
	mg->setFramesPerSecond(frame_rate_);
	grabber_.reset(mg);
	this->cplayer_->setGrabber(grabber_);
	this->cplayer_->playPause();
}

void SpectolabViewer::loadScan() {
	QString fileName = QFileDialog::getOpenFileName(this, tr("Select PCD or Spectroscan image frame"),
													 settings_.value("data_path","").toString(),
	                                                 tr("Files (*.pcd Frame* *.ssi)"));
	if (fileName.size() ==0) return;
	boost::filesystem::path frame_path = fileName.toAscii().data();

	QString save_path = (char*) frame_path.parent_path().c_str();
	settings_.setValue("data_path", save_path);

	if (frame_path.extension()==".pcd"){
		grabber_.reset(new pcl::MovieGrabber( frame_path.string(), frame_path.extension().string() ) );
	}
	else if (frame_path.extension()==".ssi"){ //load new file format
		grabber_.reset(new pcl::Spectroscan3DMovieGrabber( frame_path.string(), true) );
	}
	else{ //load original frame that has no extension
		grabber_.reset(new pcl::Spectroscan3DMovieGrabber( frame_path.string(), false) );
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
		this->cplayer_->playPause();
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
	settings_widget_->show();
}

void SpectolabViewer::spectroscan3dSettingsApplied(){
	if ( dynamic_cast<pcl::Spectroscan3DGrabber*>( grabber_.get()) !=NULL){
		dynamic_cast<pcl::Spectroscan3DGrabber*>( grabber_.get())->setSettings(spectroscan_settings_);
	}
	if ( dynamic_cast<pcl::Spectroscan3DMovieGrabber*>( grabber_.get()) !=NULL){
		dynamic_cast<pcl::Spectroscan3DMovieGrabber*>( grabber_.get())->setSettings(spectroscan_settings_);
	}
}

void SpectolabViewer::closeEvent(QCloseEvent* event) {
	this->settings_widget_->close();
    if (interface_widget_!=NULL) {
        interface_widget_->close();
        delete interface_widget_;
    }
	QMainWindow::closeEvent(event);
}


void SpectolabViewer::commandInterface(){
    if (interface_widget_ != NULL){
        delete interface_widget_;
    }
    if (grabber_==NULL){
        this->statusBar()->showMessage("No Spectroscan 3D connected.  Cannot open Command Interface", 30000);
        return;
    }
    if (dynamic_cast<pcl::Spectroscan3DGrabber*>( grabber_.get()) ==NULL){
        this->statusBar()->showMessage("No Spectroscan 3D connected.  Cannot open Command Interface", 30000);
        return;
    }
    interface_widget_ = new CMDInterfaceWidget(boost::dynamic_pointer_cast<pcl::Spectroscan3DGrabber>( grabber_), NULL);
    interface_widget_->show();
}
