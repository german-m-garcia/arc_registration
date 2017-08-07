#ifndef GRASP_EDITOR_H
#define GRASP_EDITOR_H

#include <QWidget>
#include <rviz/display.h>
#include <rviz/properties/string_property.h>
#include <rviz/display_context.h>
#include <rviz/visualization_manager.h>
#include <rviz/display_factory.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>

#include "arc_registration/arc_registration_node.h"
#include "arc_registration/objects.h"
#include "ui_grasp_editor.h"
#include "object_grasp.h"



typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;

/*namespace Ui {
class grasp_editor;
}*/
namespace grasp_editor {

class grasp_editor : public rviz::Display//public QWidget
{
  Q_OBJECT

public:
  grasp_editor();
  ~grasp_editor();
  void markerFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void shapeMarkerFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  virtual void onInitialize();

private Q_SLOTS:
  void on_loadObjectPushButton_clicked();

  void on_addGraspPushButton_clicked();

  void on_saveGraspPushButton_clicked();

  void on_addShapePushButton_clicked();   

  void on_xSizeHorizontalSlider_sliderReleased();

  void on_ySizeHorizontalSlider_sliderReleased();

  void on_zSizeHorizontalSlider_sliderReleased();

  void on_addShapeGraspPushButton_clicked();

  void on_loadGraspsPushButton_clicked();

  void on_sampleGraspPushButton_clicked();

  void on_objectsPathPushButton_clicked();

  void on_objectsComboBox_currentIndexChanged(int index);

  void on_loadViewsPushButton_clicked();

  void on_viewPushButton_clicked();

  void on_normCheckBox_clicked();

  void on_normto0CheckBox_clicked();

  void on_viewMergedPushButton_clicked();


  void on_registrationPushButton_clicked();

  void on_loadDataCloudPushButton_clicked();

  void on_testDataCheckBox_clicked();

  void on_fullModelcheckBox_clicked();

  void on_registerLibPushButton_clicked();

  void on_mrsMapsPushButton_clicked();

private:
  //Ui::grasp_editor *ui;
  Ui::grasp_editor m_ui;
  QWidget* m_w;
  std::vector<std::string> image_paths;
  QString data_path;
  PointCloudTPtr object_cloud_0;
  std::vector<PointCloudTPtr> object_clouds;
  std::map<std::string, PointCloudTPtr> merged_clouds; //contains the merged point clouds mapped by their path in the filesystem

  ros::Publisher cloud_pub;
  ros::Publisher marker_pub;
  ros::Publisher grasp_marker_pub;
  ros::Publisher test_data_cloud_pub;
  interactive_markers::InteractiveMarkerServer server;  
  interactive_markers::MenuHandler menu_handler;
  const std::string model_cloud_frame_id = "model_cloud";
  std::string basic_shape_frame_id;
  geometry_msgs::PoseStamped currentAddedGraspPose;
  geometry_msgs::PoseStamped currentBasicShapePose, currentBasicGraspShapePose;
  bool addedBasicShape;
  float basicShapeScaleX, basicShapeScaleY, basicShapeScaleZ;
  float basic_shape_grasp_height;
  std::map<std::string, object_grasp_editor::Grasp> addedGraspPoses;
  uint32_t basicShapeType;

  int index_grasps;
  std::string object_name;
  std::string current_grasp_name;

  std::thread thread, thread_basic_tfs;

  //align views tab
  ros::Publisher views_cloud_pub;
  QString views_data_path;
  std::map<int,std::string> map_objects_paths;
  std::vector<ObjectPtr> objects; //contains the objects to be loaded to visualise the alignment of the views
  ObjectPtr current_view_object;
  ros::ServiceClient registration_client, mrsmap_reg_client;
  void publishDebugClouds(ObjectPtr object);
  void publishDebugTf(std::string frame_id,int i);

  //registration tab
  std::string camera_frame = "camera_frame";
  PointCloudTPtr test_data_cloud;
  ARCRegistration arc_registration_lib;




  void loadData();
  void load_main_cloud(std::string path_0);
  void load_relative_cloud(int i);  
  void addShape(uint32_t shape = visualization_msgs::Marker::CYLINDER, geometry_msgs::PoseStamped optionalPose = geometry_msgs::PoseStamped(), double scale_x = 0.2, double scale_y = 0.2, double scale_z = 0.2);
  void addMarker();
  void loadGrasps();
  void saveGrasps();
  void createMarker(geometry_msgs::PoseStamped& pose);
  void addMenu(visualization_msgs::InteractiveMarker& int_marker, std::string type="top");
  void publishBasicShapeTf();
  void addArrow(geometry_msgs::Point& p0, geometry_msgs::Point& pf);
  void tf_publisher();
  void basic_tfs_publisher();
  void simulateDataCloud(std::string& object_name,Eigen::Affine3f& object_pose, PointCloudTPtr& data_cloud);

};
}
#endif // GRASP_EDITOR_H
