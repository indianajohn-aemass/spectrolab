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

#ifndef PCL_VISUALIZATION_BW_COLOR_HANDLER_H_
#define PCL_VISUALIZATION_BW_COLOR_HANDLER_H_

#include <pcl/visualization/point_cloud_handlers.h>

namespace pcl {
namespace visualization {

/** \brief Color Handler for rendering intensity in Black & White
 * \author Adam Stambler <adasta@gmail.com>
 * \ingroup visualization
 */
template<typename PointT>
class PCL_EXPORTS PointCloudIntensityHandler : public PointCloudColorHandler<
    PointT> {
 public:
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  typedef boost::shared_ptr<PointCloudColorHandler<PointT> > Ptr;
  typedef boost::shared_ptr<const PointCloudColorHandler<PointT> > ConstPtr;

  /** \brief Constructor. */
  PointCloudIntensityHandler(const PointCloudConstPtr &cloud)
      : PointCloudColorHandler<PointT>(cloud) {
    capable_ = true;
    cloud_ = cloud;
  }
  virtual ~PointCloudIntensityHandler() {
  }

  /** \brief Abstract getName method. */
  virtual std::string getName() const {
    return "PointCloudIntensityHandler";
  }
  ;

  /** \brief Abstract getFieldName method. */
  virtual std::string getFieldName() const {
    return "intensity";
  }

  /** \brief Obtain the actual color for the input dataset as vtk scalars.
   * \param[out] scalars the output scalars containing the color for the dataset
   * \return true if the operation was successful (the handler is capable and
   * the input cloud was given as a valid pointer), false otherwise
   */
#if ( ( PCL_MAJOR_VERSION >=1) && (  PCL_MINOR_VERSION > 6) )
  virtual bool getColor(vtkSmartPointer<vtkDataArray> &scalars) const {
    if (!capable_ || !cloud_)
      return (false);
#else
    virtual void
    getColor (vtkSmartPointer<vtkDataArray> &scalars) const
    {
      if (!capable_ || !cloud_)
      return;
#endif
    if (!scalars)
      scalars = vtkSmartPointer<vtkUnsignedCharArray>::New();
    scalars->SetNumberOfComponents(3);

    vtkIdType nr_points = cloud_->points.size();
    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples(
        nr_points);

    // Get a random color
    unsigned char* colors = new unsigned char[nr_points * 3];
    unsigned char r, g, b;
    // Color every point
    for (vtkIdType cp = 0; cp < nr_points; ++cp) {
      float val = cloud_->points[cp].intensity * 255;  // + 50;
      if (val > 255)
        val = 255;
      colors[cp * 3 + 0] = val;
      colors[cp * 3 + 1] = val;
      colors[cp * 3 + 2] = val;
    }
    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray(
        colors, 3 * nr_points, 0);

#if ( ( PCL_MAJOR_VERSION >=1) && (  PCL_MINOR_VERSION > 6) )
    return true;
#endif
  }

  /** \brief Set the input cloud to be used.
   * \param[in] cloud the input cloud to be used by the handler
   */
  virtual void setInputCloud(const PointCloudConstPtr &cloud) {
    cloud_ = cloud;
  }

 protected:
  // Members derived from the base class
  using PointCloudColorHandler<PointT>::cloud_;
  using PointCloudColorHandler<PointT>::capable_;

};

}
}

#endif /* PCL_VISUALIZATION_BW_COLOR_HANDLER_H_ */
