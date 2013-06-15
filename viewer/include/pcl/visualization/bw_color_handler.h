/*
 * bw_color_handler.h
 *
 *  Created on: Jun 14, 2013
 *      Author: asher
 */

#ifndef _PCL_BW_COLOR_HANDLER_H_
#define _PCL_BW_COLOR_HANDLER_H_

#include <pcl/visualization/point_cloud_color_handlers.h>

namespace pcl{
 namespace visualization{

 template <typename PointT>
     class PointCloudIntensityHandler : public PointCloudColorHandler<PointT>
     {
       public:
         typedef pcl::PointCloud<PointT> PointCloud;
         typedef typename PointCloud::Ptr PointCloudPtr;
         typedef typename PointCloud::ConstPtr PointCloudConstPtr;

         typedef boost::shared_ptr<PointCloudColorHandler<PointT> > Ptr;
         typedef boost::shared_ptr<const PointCloudColorHandler<PointT> > ConstPtr;

         /** \brief Constructor. */
         PointCloudIntensityHandler (const PointCloudConstPtr &cloud) : PointCloudColorHandler<PointT>()
         {
        	 capable_=true;
        	 cloud_=cloud;
         }
         virtual ~PointCloudIntensityHandler(){}


         /** \brief Abstract getName method. */
         virtual std::string
         getName () const { return "PointCloudIntensityHandler";};

         /** \brief Abstract getFieldName method. */
         virtual std::string
         getFieldName () const {return "intensity";};

         /** \brief Obtain the actual color for the input dataset as vtk scalars.
           * \param[out] scalars the output scalars containing the color for the dataset
           * \return true if the operation was successful (the handler is capable and
           * the input cloud was given as a valid pointer), false otherwise
           */
         virtual bool
         getColor (vtkSmartPointer<vtkDataArray> &scalars) const {
        	  if (!capable_ || !cloud_)
        	    return (false);

        	  if (!scalars)
        	    scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        	  scalars->SetNumberOfComponents (3);

        	  vtkIdType nr_points = cloud_->points.size ();
        	  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);

        	  // Get a random color
        	  unsigned char* colors = new unsigned char[nr_points * 3];
        	  unsigned char r, g, b;
        	  // Color every point
        	  for (vtkIdType cp = 0; cp < nr_points; ++cp)
        	  {
            	float val =  cloud_->points[cp].intensity*200+50;
            	if (val>255) val=255;
        	    colors[cp * 3 + 0] = val ;
        	    colors[cp * 3 + 1] = val;
        	    colors[cp * 3 + 2] = val;
        	  }
        	  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, 3 * nr_points, 0);
        	 return capable_;
         }

         /** \brief Set the input cloud to be used.
           * \param[in] cloud the input cloud to be used by the handler
           */
         virtual void
         setInputCloud (const PointCloudConstPtr &cloud)
         {
           cloud_ = cloud;
         }

       protected:
         // Members derived from the base class
         using PointCloudColorHandler<PointT>::cloud_;
         using PointCloudColorHandler<PointT>::capable_;

     };

 }
}


#endif /* BW_COLOR_HANDLER_H_ */
