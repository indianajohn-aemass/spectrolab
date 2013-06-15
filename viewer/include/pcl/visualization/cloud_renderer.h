/*
 * cloud_renderer.h
 *
 *  Created on: Jun 14, 2013
 *      Author: asher
 */

#ifndef CLOUD_RENDERER_H_
#define CLOUD_RENDERER_H_

#include <QVTKWidget.h>
#include <QObject>

#include <pcl/io/grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

#include<pcl/point_types.h>

namespace pcl{

 namespace visualization{

	class CloudRenderer  : public QObject{
		Q_OBJECT
		protected:
			std::string description_;
			QVTKWidget* widget_;
			pcl::visualization::PCLVisualizer* visualizer_;
			boost::signals2::connection connection_;
		public:
			CloudRenderer( ): widget_(NULL), visualizer_(NULL){}
			virtual ~CloudRenderer(){}

			virtual void init(QVTKWidget* widget, PCLVisualizer * visualizer){
				widget_=widget;
				visualizer_=visualizer;
			}
			virtual bool setup(boost::shared_ptr<Grabber>& grabber)=0;
			virtual void disconnect()=0;

			virtual void renderNew()=0;
			std::string description(){return description_;};

		signals:
			void update();
	};

	class CloudRendererRange  : public CloudRenderer{
		public:
			CloudRendererRange(std::string field);
			virtual ~CloudRendererRange(){}
			virtual bool setup(boost::shared_ptr<Grabber>& grabber);
			virtual void disconnect();
			virtual void renderNew();
		private:
			std::string field_name_;
			boost::mutex cloud_mutex_;
			sensor_msgs::PointCloud2::ConstPtr cloud_;
			bool rendered_;
			bool valid_grabber_;
			void grabberCB( const sensor_msgs::PointCloud2::ConstPtr& cloud);
			boost::signals2::connection connection_;
	};

	class CloudRendererBW : public CloudRenderer{
	public:
		CloudRendererBW();
		virtual ~CloudRendererBW(){}
		virtual bool setup(boost::shared_ptr<Grabber>& grabber);
		virtual void disconnect();
		virtual void renderNew();
	private:
		boost::mutex cloud_mutex_;
		pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_;
		bool valid_grabber_;
		void grabberCB( const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
		boost::signals2::connection connection_;
	};
 }
}



#endif /* CLOUD_RENDERER_H_ */
