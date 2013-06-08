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
#include <pcl/visualization/pcl_visualizer.h>

#include <QWidget>

namespace pcl {
namespace visualization{

class CloudRenderer {
	public:
		virtual ~CloudRenderer(){}
		virtual bool setup(boost::shared_ptr<Grabber>& grabber,
						 PCLVisualizer * visualizer)=0;
		virtual void renderNew()=0;
		std::string description(){return description_;};
	protected:
		std::string description_;
		pcl::visualization::PCLVisualizer* visualizer_;
		boost::signals2::connection connection_;
};

class CloudRendererRange  : public CloudRenderer{
	public:
		CloudRendererRange(std::string field);
		virtual ~CloudRendererRange(){}
		virtual bool setup(boost::shared_ptr<Grabber>& grabber,
						 PCLVisualizer * visualizer);
		virtual void renderNew();
	private:
		std::string field_name_;
		boost::mutex cloud_mutex_;
		sensor_msgs::PointCloud2::ConstPtr cloud_;
		bool valid_grabber_;
		void grabberCB( const sensor_msgs::PointCloud2::ConstPtr& cloud);
};


class CloudPlayerWidget : public QWidget{
	Q_OBJECT

	public:
		CloudPlayerWidget();
		virtual ~CloudPlayerWidget();

		void setGrabber(boost::shared_ptr<Grabber>& grabber);

		void addCloudRenderer(CloudRenderer* renderer);

	private:
		boost::shared_ptr<Grabber> grabber_;
		bool is_movie_grabber_;
		PCLVisualizer* pcl_visualizer_;

		Ui_CloudPlayer ui_;
		std::vector<CloudRenderer*> renderers_;
		uint32_t current_renderer_idx_;

		bool playing_;
		boost::signals2::connection progress_connection_;

	protected:
		virtual void paintEvent(QPaintEvent* e);

	public slots:
	  void playPause(QAction* act);
	  void forward(QAction* act);
	  void backward(QAction* act);
	  void record(QAction* act);
	  void resetView(QAction* act);
	protected :
	  void enablePlayback();
	  void disablePlayback();
	  void progressUpdate(size_t frame_num, size_t frame_total);

	};
}

} /* namespace adaptive_pushgrasp */
#endif /* CLOUD_PLAYER_WIDGET_H_ */
