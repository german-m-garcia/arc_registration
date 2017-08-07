// example_ros_class.h header file //
// wsn; Feb, 2015
// include this file in "example_ros_class.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef ARC_REGISTRATION_H_
#define ARC_REGISTRATION_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>


#include <image_transport/image_transport.h>

#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp6d.h>


#include <tf/transform_broadcaster.h>

#include "arc_registration/Registration.h"
#include "arc_registration/object_grasp.h"
#include "arc_registration/objects.h"



#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "arc_registration/utils.hpp"

#include <thread>


class ARCRegistration
{
public:  
    ARCRegistration(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    ARCRegistration(const std::string& data_path); //constructor for the use as library
    //returns the score of gicp
    double register_model_cloud(const std::string& object_name, const std::string& camera_frame, Eigen::Affine3f& cam_T_pred_pose, PointCloudTPtr& data_cloud, std::vector<Eigen::Affine3d>& grasping_poses);
    ~ARCRegistration();

private:
    ros::NodeHandle nh_;
    ros::Subscriber minimal_subscriber_;
    ros::ServiceServer registration_service_, mrsregistration_service_;
    ros::ServiceClient mrsmap_reg_client;
    ros::Publisher  minimal_publisher_;
    ros::Publisher registered_cloud_pub_;
    ros::Publisher data_cloud_pub_;
    ros::Publisher model_cloud_pub_;
    //image_transport::ImageTransport it_;
    image_transport::Publisher img_pub_, depth_pub_;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    std::vector<ObjectPtr> objects;
    std::map<std::string, ObjectPtr> objects_map;

    //
    std::string data_path, registration_objects_path;
    std::vector<std::string> image_paths;
    std::vector<FramePtr> frames_data;
    object_grasp_editor::ObjectGrasp grasps_;

    //transforms to be sent
    bool do_publish_tfs;
    tf::StampedTransform object_stamped_transform;
    std::vector<tf::StampedTransform> grasp_tfs;
    std::thread thread;

    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();

    bool loadAllObjects(std::vector<std::string>& registration_objects_paths);
    bool loadAllObjects(const std::string& all_objects_path);
    void loadGrasps(const std::string& object_grasp_path, object_grasp_editor::ObjectGrasp& g);
    void computeObjectCentroid(ObjectPtr& object);
    void loadObjectModelData(ObjectPtr object);
    void loadFrame(FramePtr frame);
    double  cgicp(PointCloudT::Ptr& cloud_1, PointCloudT::Ptr& cloud_2, PointCloudT::Ptr& out, Eigen::Matrix4f& transform);
    double   gicp(PointCloudT::Ptr& cloud_1, PointCloudT::Ptr& cloud_2, PointCloudT::Ptr& out, Eigen::Matrix4f& transform);
    //returns an index to frames_data
    int getClosestFrame(ObjectPtr& object, Eigen::Quaternionf& q_predicted);

    void publishRegisteredView(ObjectPtr& object,int closest_view_index, Eigen::Affine3f& transformation, const  std::string& camera_frame);
    void publishRegisteredView(ObjectPtr& object, Eigen::Affine3f& transformation, const  std::string& camera_frame);

    void subSample(PointCloudTPtr& src_cloud, PointCloudTPtr& dst_cloud);
    bool registrationServiceCallback(arc_registration::RegistrationRequest& request, arc_registration::RegistrationResponse& response);
    bool registrationMRSMapServiceCallback(arc_registration::RegistrationRequest& request, arc_registration::RegistrationResponse& response);
    void publishRegisteredObjectTf(Eigen::Affine3f& pose, const  std::string& camera_frame , const std::string& object_frame_id);
    void publishGrasps(ObjectPtr& object);
    void publisher_thread();

    void publishDataCloud(PointCloudTPtr& data_cloud);
    void publishModelCloud(PointCloudTPtr& model_cloud);
    void constructModelCloud(ObjectPtr& object,int closest_view_index ,Eigen::Affine3f& predicted_object_pose, PointCloudTPtr& model_cloud);
    void getGraspingPose(const std::string& grasp_frame_id, const std::string& camera_frame, Eigen::Affine3d& graspEigenTransform);

    template<typename T>
    std::string to_string ( const T & t )
    {
       std::ostringstream ss;
       ss << t;
       return ss.str();
    }

    template <typename T>
    T getParamElseDefault ( ros::NodeHandle & nh, const std::string & paramName, const T & def )
    {
       T out;

       if ( ! nh.getParam ( paramName, out ) )
       {
          ROS_INFO_STREAM ( paramName << ": Default value used (" << to_string ( def ) << ")" );
          return def;
       };

       ROS_INFO_STREAM ( paramName << ": " << to_string ( out ) );

       return out;
    }

};

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
