/*
 * cloud_player_widget.h
 *
 *  Created on: Jun 5, 2013
 *      Author: asher
 */

#ifndef CLOUD_PLAYER_WIDGET_H_
#define CLOUD_PLAYER_WIDGET_H_

#include "ui_cloud_player.h"
#include <pcl/io/grabber.h>
#include <pcl/io/recorder.h>

#include <pcl/visualization/pcl_visualizer.h>
#include "cloud_renderer.h"
#include <QWidget>

class QErrorMessage;

namespace pcl {
namespace visualization{

class CloudPlayerWidget : public QWidget{
	Q_OBJECT

	public:
		CloudPlayerWidget(QWidget* parent = 0, Qt::WindowFlags f = 0);
		virtual ~CloudPlayerWidget();

		void setGrabber(boost::shared_ptr<Grabber>& grabber);

		void addCloudRenderer(CloudRenderer* renderer);
		size_t getNumRenderers(){return renderers_.size();}

		uint32_t  currentRendererIDX(){return current_renderer_idx_;}
		CloudRenderer* getRenderer(size_t idx);
		void setCurrentRenderer(int idx);


		void addRecorder(Recorder* recorder);
		uint32_t  currentRecorderIDX(){return current_recorder_idx_;}
		void setCurrentRecorder(uint32_t idx);

	private:
		boost::shared_ptr<Grabber> grabber_;
		bool is_movie_grabber_;
		PCLVisualizer* pcl_visualizer_;

		Ui_CloudPlayer ui_;
		std::vector<CloudRenderer*> renderers_;
		std::vector<Recorder*> recorders_;

		uint32_t current_renderer_idx_;
		uint32_t current_recorder_idx_;

		bool recording_;
		bool playing_;
		boost::signals2::connection progress_connection_;

		QErrorMessage* error_msg_;

	public slots:
		void playPause( );
		void sliderValueChanged(int val);
		void record( );

		void resetView( );
		void updateCloud();
	protected :
		void keyCB(const pcl::visualization::KeyboardEvent& e);
		void enablePlayback();
		void disablePlayback();
		void progressUpdate(size_t frame_num, size_t frame_total);
		void startRecording();
		void stopRecording();
		void enableRenderering();

	protected slots:
		void rendererSelectedViaMenu();
};

	}/* namespace visualization */
} /* namespace pcl */
#endif /* CLOUD_PLAYER_WIDGET_H_ */
