

#ifndef OBJECTS_H_
#define OBJECTS_H_

//#include "arc_rendering/grasp_editor.h"
#include "arc_registration/object_grasp.h"
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>

//namespace object_grasp_editor{
//  class ObjectGrasp;
//}


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;

struct Frame {

    cv::Mat rgb;
    cv::Mat depth;
    cv::Mat mask;
    std::string path;
    PointCloudTPtr pcloud;
    PointCloudTPtr pcloud_normalized_to_0;
    //+ the orientation
    Eigen::Quaternionf q; //the orientation of the object in the camera frame of reference
    Eigen::Quaternionf transform_to_first_view; //need to store the transformation to the first view 0/0000
    Frame():pcloud(new PointCloudT), pcloud_normalized_to_0(new PointCloudT){}


};
typedef std::shared_ptr<Frame> FramePtr;

struct Object {
  std::string object_name;
  std::string object_model_path;
  std::vector<FramePtr> frames_data;
  object_grasp_editor::ObjectGrasp grasps;
  PointCloudTPtr merged_cloud;
  Object(): merged_cloud(new PointCloudT){}
  double offset_centroid_x,offset_centroid_y,offset_centroid_z;
};
typedef std::shared_ptr<Object> ObjectPtr;

#endif
