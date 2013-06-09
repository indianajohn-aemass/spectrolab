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

class CloudRenderer  : public QObject{
	Q_OBJECT
	protected:
		std::string description_;
		QVTKWidget* widget_;
		pcl::visualization::PCLVisualizer* visualizer_;
		boost::signals2::connection connection_;
	public:
		CloudRenderer(QVTKWidget* widget, PCLVisualizer * visualizer): widget_(widget), visualizer_(visualizer){}
		virtual ~CloudRenderer(){}
		virtual bool setup(boost::shared_ptr<Grabber>& grabber)=0;
		virtual void renderNew()=0;
		std::string description(){return description_;};

	signals:
		void update();
};

class CloudRendererRange  : public CloudRenderer{
	public:
		CloudRendererRange(std::string field, QVTKWidget* widget, PCLVisualizer * visualizer);
		virtual ~CloudRendererRange(){}
		virtual bool setup(boost::shared_ptr<Grabber>& grabber);
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
		CloudPlayerWidget(QWidget* parent = 0, Qt::WindowFlags f = 0);
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

	public slots:
		void playPause( );
		void sliderValueChanged(int val);
		void record( );
		void resetView( );
		virtual  void updateCloud();
	protected :
		void keyCB(const pcl::visualization::KeyboardEvent& e);
		void enablePlayback();
		void disablePlayback();
		void progressUpdate(size_t frame_num, size_t frame_total);
	};
}

} /* namespace adaptive_pushgrasp */
#endif /* CLOUD_PLAYER_WIDGET_H_ */
