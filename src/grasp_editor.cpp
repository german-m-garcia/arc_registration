#include <QFileDialog>
#include <QDirIterator>
#include <tf/transform_listener.h>



#include <pcl/common/common.h>
#include <pcl/common/transforms.h>


#include "arc_registration/Registration.h"
#include "arc_registration/grasp_editor.h"
#include "ui_grasp_editor.h"
#include "arc_registration/object_grasp.h"
#include <tf/transform_broadcaster.h>
#include <random>
#include <pcl/common/centroid.h>
#include <eigen_conversions/eigen_msg.h>

#include <rosmrsmap/ArcRegisterMap.h> //registration of MRSMaps

using namespace boost::filesystem;

grasp_editor::grasp_editor* bad_practise_reference_editor;


/*
 * comparison function to sort filenames in a natural order
 * e.g.:
 *  1.txt
 *  2.txt
 *  10.txt
 *  20.txt
 *
 */
bool compareNatG(const std::string& a, const std::string& b)
{
    if (a.empty())
        return true;
    if (b.empty())
        return false;
    if (std::isdigit(a[0]) && !std::isdigit(b[0]))
        return true;
    if (!std::isdigit(a[0]) && std::isdigit(b[0]))
        return false;
    if (!std::isdigit(a[0]) && !std::isdigit(b[0]))
    {
        if (std::toupper(a[0]) == std::toupper(b[0]))
            return compareNatG(a.substr(1), b.substr(1));
        return (std::toupper(a[0]) < std::toupper(b[0]));
    }

    // Both strings begin with digit --> parse both numbers
    std::istringstream issa(a);
    std::istringstream issb(b);
    int ia, ib;
    issa >> ia;
    issb >> ib;
    if (ia != ib)
        return ia < ib;

    // Numbers are the same --> remove numbers and recurse
    std::string anew, bnew;
    std::getline(issa, anew);
    std::getline(issb, bnew);
    return (compareNatG(anew, bnew));
}

std::string folderNameG (const std::string& str)
{
  size_t found;
  found=str.find_last_of("/\\");
  std::cout << " folder: " << str.substr(0,found) << std::endl;
  std::cout << " file: " << str.substr(found+1) << std::endl;
  return str.substr(found+1);
}

void feedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){

  bad_practise_reference_editor->markerFeedback(feedback); 
}

void shapeFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){

  bad_practise_reference_editor->shapeMarkerFeedback(feedback);
}

namespace grasp_editor {


grasp_editor::grasp_editor(): object_cloud_0(new PointCloudT), server("simple_marker"), addedBasicShape(false), basic_shape_grasp_height(0.f), index_grasps(0),
  test_data_cloud(new PointCloudT), arc_registration_lib("/home/martin/Datasets/ARC/20_7/")
{
  rviz::Display();
  ros::NodeHandle n("");
  bad_practise_reference_editor = this;

}

void grasp_editor::onInitialize(){
  rviz::Display::onInitialize();
  ros::NodeHandle nh("/grasp_editor");
  cloud_pub = nh.advertise<PointCloudT> ("model_cloud", 1);
  test_data_cloud_pub = nh.advertise<PointCloudT> ("test_data_cloud", 1);
  views_cloud_pub = nh.advertise<PointCloudT> ("views_debug_cloud", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("basic_shape", 1);
  grasp_marker_pub = nh.advertise<visualization_msgs::Marker>( "grasp_basic_shape", 0 );
  m_w = new QWidget;
  m_ui.setupUi(m_w);
  setAssociatedWidget(m_w);
  //grasps tab
  connect(m_ui.loadObjectPushButton, SIGNAL(clicked(bool)), SLOT(on_loadObjectPushButton_clicked()));
  connect(m_ui.addGraspPushButton, SIGNAL(clicked(bool)), SLOT(on_addGraspPushButton_clicked()));
  connect(m_ui.saveGraspPushButton, SIGNAL(clicked(bool)), SLOT(on_saveGraspPushButton_clicked()));
  connect(m_ui.addShapePushButton, SIGNAL(clicked(bool)), SLOT(on_addShapePushButton_clicked()));
  connect(m_ui.addShapeGraspPushButton, SIGNAL(clicked(bool)), SLOT(on_addShapeGraspPushButton_clicked()));
  connect(m_ui.loadGraspsPushButton, SIGNAL(clicked(bool)), SLOT(on_loadGraspsPushButton_clicked()));

  connect(m_ui.sampleGraspPushButton, SIGNAL(clicked(bool)), SLOT(on_sampleGraspPushButton_clicked()));
  connect(m_ui.viewMergedPushButton, SIGNAL(clicked(bool)), SLOT(on_viewMergedPushButton_clicked()));


  //align views tab
  connect(m_ui.objectsPathPushButton, SIGNAL(clicked(bool)), SLOT(on_objectsPathPushButton_clicked()));
  connect(m_ui.objectsComboBox, SIGNAL(currentIndexChanged(int)), SLOT(on_objectsComboBox_currentIndexChanged(int index)));
  connect(m_ui.loadViewsPushButton, SIGNAL(clicked(bool)), SLOT(on_loadViewsPushButton_clicked()));
  connect(m_ui.viewPushButton, SIGNAL(clicked(bool)), SLOT(on_viewPushButton_clicked()));
  connect(m_ui.normCheckBox, SIGNAL(clicked(bool)), SLOT(on_normCheckBox_clicked()));
  connect(m_ui.normto0CheckBox, SIGNAL(clicked(bool)), SLOT(on_normto0CheckBox_clicked()));


  //test registration tab
  connect(m_ui.registrationPushButton, SIGNAL(clicked(bool)), SLOT(on_registrationPushButton_clicked()));
  connect(m_ui.loadDataCloudPushButton, SIGNAL(clicked(bool)), SLOT(on_loadDataCloudPushButton_clicked()));
  connect(m_ui.testDataCheckBox, SIGNAL(clicked(bool)), SLOT(on_testDataCheckBox_clicked()));
  connect(m_ui.fullModelcheckBox, SIGNAL(clicked(bool)), SLOT(on_fullModelcheckBox_clicked()));
  connect(m_ui.registerLibPushButton, SIGNAL(clicked(bool)), SLOT(on_registerLibPushButton_clicked()));
  connect(m_ui.mrsMapsPushButton, SIGNAL(clicked(bool)), SLOT(on_mrsMapsPushButton_clicked()));

  m_ui.fullModelcheckBox->setChecked(true);



  m_ui.x->setRange(-10000.0, 10000.0);
  m_ui.y->setRange(-10000.0, 10000.0);
  m_ui.z->setRange(-10000.0, 10000.0);

  m_ui.qx->setRange(-1.0, 1.0);
  m_ui.qy->setRange(-1.0, 1.0);
  m_ui.qz->setRange(-1.0, 1.0);
  m_ui.qw->setRange(-1.0, 1.0);

  m_ui.qx->setSingleStep(0.05);
  m_ui.qy->setSingleStep(0.05);
  m_ui.qz->setSingleStep(0.05);
  m_ui.qw->setSingleStep(0.05);

  registration_client = nh.serviceClient<arc_registration::Registration>("/arcRegistrationNode/registration");
  mrsmap_reg_client = nh.serviceClient<arc_registration::Registration>("/arcRegistrationNode/mrsmap_registration");


  //shape sliders
  connect(m_ui.xSizeHorizontalSlider, SIGNAL(sliderReleased()), SLOT(on_xSizeHorizontalSlider_sliderReleased()));
  connect(m_ui.ySizeHorizontalSlider, SIGNAL(sliderReleased()), SLOT(on_ySizeHorizontalSlider_sliderReleased()));
  connect(m_ui.zSizeHorizontalSlider, SIGNAL(sliderReleased()), SLOT(on_zSizeHorizontalSlider_sliderReleased()));



  boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindCallback =
  bind(&grasp_editor::markerFeedback, this, _1);
  menu_handler.insert( "Delete grasp", &feedback );
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Mode" );
  //add grasp mode options
  {
    interactive_markers::MenuHandler::EntryHandle h_mode_last = menu_handler.insert(sub_menu_handle, "top", &feedback );
    menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::CHECKED );
    h_mode_last = menu_handler.insert( sub_menu_handle, "side", &feedback );
    menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::UNCHECKED );
    h_mode_last = menu_handler.insert( sub_menu_handle, "other", &feedback );
    menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::UNCHECKED );
  }
  m_ui.graspIconPushButton->setEnabled(false);

  std::cout <<"setting up grasp_editor"<<std::endl;
  thread = std::thread(&grasp_editor::tf_publisher, this);
  thread_basic_tfs = std::thread(&grasp_editor::basic_tfs_publisher, this);
}

void grasp_editor::tf_publisher() {

  while(ros::ok()){
    publishBasicShapeTf();
    ros::Duration(0.5).sleep();
  }
}

void grasp_editor::basic_tfs_publisher() {
  static tf::TransformBroadcaster br;
  tf::StampedTransform camera_tf;
  tf::Quaternion q_cam, q_cam2;
  tf::Vector3 translation(0.4,0,0.4);
  q_cam.setRPY(0.,M_PI/2.,0);
  q_cam2.setRPY(-M_PI/2.,0.,0);
  q_cam = q_cam2*q_cam;
  camera_tf.child_frame_id_ = camera_frame;
  camera_tf.frame_id_ =  "world";
  camera_tf.setRotation(q_cam);
  camera_tf.setOrigin(translation);
  while(ros::ok()){
    //publish the camera_frame tf
    camera_tf.stamp_ = ros::Time::now();
    br.sendTransform(camera_tf);
    ros::Duration(0.5).sleep();

  }
}


void grasp_editor::on_normCheckBox_clicked(){
  if(m_ui.normCheckBox->isChecked())
    m_ui.normto0CheckBox->setChecked(false);

  else
    m_ui.normto0CheckBox->setChecked(true);
}

void grasp_editor::on_normto0CheckBox_clicked(){

  if(m_ui.normto0CheckBox->isChecked())
    m_ui.normCheckBox->setChecked(false);

  else
    m_ui.normCheckBox->setChecked(true);

}

void grasp_editor::on_objectsComboBox_currentIndexChanged(int index){
  std::cout <<"selected item path: "<<map_objects_paths[index]<<std::endl;
}

void grasp_editor::on_addShapeGraspPushButton_clicked(){

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = model_cloud_frame_id;
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "grasps_basic_shape_"+object_name;
  int_marker.description = "grasps_basic_shape_"+object_name;
  int_marker.scale = 0.2f;
  //initialise the pose with the given values
  int_marker.pose.orientation.x = currentBasicShapePose.pose.orientation.x;
  int_marker.pose.orientation.y = currentBasicShapePose.pose.orientation.y;
  int_marker.pose.orientation.z = currentBasicShapePose.pose.orientation.z;
  int_marker.pose.orientation.w = currentBasicShapePose.pose.orientation.w;

  int_marker.pose.position.x = currentBasicShapePose.pose.position.x;
  int_marker.pose.position.y = currentBasicShapePose.pose.position.y;
  int_marker.pose.position.z = currentBasicShapePose.pose.position.z;

  // create a grey box marker
  visualization_msgs::Marker main_shape;
  main_shape.type = visualization_msgs::Marker::CYLINDER;
  main_shape.scale.x = basicShapeScaleX;
  main_shape.scale.y = basicShapeScaleY;
  main_shape.scale.z = 0.02;
  main_shape.color.r = 0.5;
  main_shape.color.g = 0.0;
  main_shape.color.b = 0.0;
  main_shape.color.a = 0.7;

  main_shape.pose.position.x = 0;
  main_shape.pose.position.y = 0;
  main_shape.pose.position.z = 0;

  main_shape.pose.orientation.x = 0.;
  main_shape.pose.orientation.y = 0.;
  main_shape.pose.orientation.z = 0.;
  main_shape.pose.orientation.w = 1.;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( main_shape );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  {
    visualization_msgs::InteractiveMarkerControl control2;

    control2.orientation.w = 1;
    control2.orientation.x = 0;
    control2.orientation.y = 1;
    control2.orientation.z = 0;

    control2.name = "move_z";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control2);

  }
  boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
  bind(&grasp_editor::shapeMarkerFeedback, this, _1);
  server.insert(int_marker, bindedLoop);
  server.applyChanges();

}


void  grasp_editor::on_xSizeHorizontalSlider_sliderReleased(){
  basicShapeScaleX = m_ui.xSizeHorizontalSlider->value()/100.f;
  std::cout <<"slider x-size="<< basicShapeScaleX <<std::endl;
  visualization_msgs::InteractiveMarker int_marker;
  bool exists = server.get ("basic_shape_"+object_name, int_marker);
  if(exists){
    int_marker.controls[0].markers[0].scale.x =basicShapeScaleX;
    //replace the marker
    boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
    bind(&grasp_editor::shapeMarkerFeedback, this, _1);
    server.insert(int_marker, bindedLoop);
    server.applyChanges();
  }
  //apply the same modifications to the grasps shape
  visualization_msgs::InteractiveMarker basic_grasps_int_marker;
  bool exists_grasps = server.get ("grasps_basic_shape_"+object_name, basic_grasps_int_marker);
  if(exists_grasps){

    basic_grasps_int_marker.controls[0].markers[0].scale.x =basicShapeScaleX;
    //replace the marker
    boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
    bind(&grasp_editor::shapeMarkerFeedback, this, _1);
    server.insert(basic_grasps_int_marker, bindedLoop);
    server.applyChanges();
  }

}


void grasp_editor::on_ySizeHorizontalSlider_sliderReleased(){
  basicShapeScaleY = m_ui.ySizeHorizontalSlider->value()/100.f;
  std::cout <<"slider y-size="<< basicShapeScaleY <<std::endl;
  visualization_msgs::InteractiveMarker int_marker;
  bool exists = server.get ("basic_shape_"+object_name, int_marker);
  if(exists){
    int_marker.controls[0].markers[0].scale.y =basicShapeScaleY;
    //replace the marker
    boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
    bind(&grasp_editor::shapeMarkerFeedback, this, _1);
    server.insert(int_marker, bindedLoop);
    server.applyChanges();
  }
  //apply the same modifications to the grasps shape
  visualization_msgs::InteractiveMarker basic_grasps_int_marker;
  bool exists_grasps = server.get ("grasps_basic_shape_"+object_name, basic_grasps_int_marker);
  if(exists_grasps){
    basic_grasps_int_marker.controls[0].markers[0].scale.y =basicShapeScaleY;
    //replace the marker
    boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
    bind(&grasp_editor::shapeMarkerFeedback, this, _1);
    server.insert(basic_grasps_int_marker, bindedLoop);
    server.applyChanges();
  }

}

void grasp_editor::on_zSizeHorizontalSlider_sliderReleased(){
  basicShapeScaleZ = m_ui.zSizeHorizontalSlider->value()/100.f;
  std::cout <<"slider z-size="<< basicShapeScaleZ <<std::endl;
  visualization_msgs::InteractiveMarker int_marker;
  bool exists = server.get ("basic_shape_"+object_name, int_marker);
  if(exists){
    int_marker.controls[0].markers[0].scale.z =basicShapeScaleZ;
    //replace the marker
    boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
    bind(&grasp_editor::shapeMarkerFeedback, this, _1);
    server.insert(int_marker, bindedLoop);
    server.applyChanges();
  }
  //apply the same modifications to the grasps shape
  visualization_msgs::InteractiveMarker basic_grasps_int_marker;
  bool exists_grasps = server.get ("grasps_basic_shape_"+object_name, basic_grasps_int_marker);
  if(exists_grasps){
    basic_grasps_int_marker.controls[0].markers[0].scale.z =basicShapeScaleZ;
    //replace the marker
    boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
    bind(&grasp_editor::shapeMarkerFeedback, this, _1);
    server.insert(basic_grasps_int_marker, bindedLoop);
    server.applyChanges();
  }

}

void grasp_editor::on_addShapePushButton_clicked(){
  //display the item selected on the combo box
  std::string selected_shape = m_ui.shapesComboBox->currentText().toStdString();
  std::cout <<"combo box item="<<selected_shape<<std::endl;
  basic_shape_frame_id = "basic_shape_"+object_name;

  if(selected_shape == "Cylinder"){
    basicShapeType = visualization_msgs::Marker::CYLINDER;
    addShape();
  }

  else if(selected_shape == "Cube"){
    basicShapeType = visualization_msgs::Marker::CUBE;
    addShape();
  }

}

void grasp_editor::on_addGraspPushButton_clicked(){
  std::cout <<"adding grasp"<<std::endl;
  current_grasp_name = "grasp_"+object_name+"_"+std::to_string(index_grasps);
  addMarker();
  index_grasps++;

}


void grasp_editor::addMarker(){
  // create an interactive marker server on the topic namespace simple_marker


  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = model_cloud_frame_id;
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = current_grasp_name;
  int_marker.description = "Grasp "+current_grasp_name;
  int_marker.scale = 0.2f;
  int_marker.pose.orientation.x = -0.7;
  int_marker.pose.orientation.y = -0.7;
  int_marker.pose.orientation.z =  0.;
  int_marker.pose.orientation.w =  0.;

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.05;
  box_marker.scale.y = 0.10;
  box_marker.scale.z = 0.02;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  box_marker.pose.position.x = 0;
  box_marker.pose.position.y = 0;
  box_marker.pose.position.z = 0;

  box_marker.pose.orientation.x = 0.;
  box_marker.pose.orientation.y = 0.;
  box_marker.pose.orientation.z = 0.;
  box_marker.pose.orientation.w = 1.;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  {
    visualization_msgs::InteractiveMarkerControl control2;
    control2.orientation.w = 1;
    control2.orientation.x = 1;
    control2.orientation.y = 0;
    control2.orientation.z = 0;
    control2.name = "rotate_x";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control2);
    control2.name = "move_x";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control2);

    control2.orientation.w = 1;
    control2.orientation.x = 0;
    control2.orientation.y = 1;
    control2.orientation.z = 0;
    control2.name = "rotate_z";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control2);
    control2.name = "move_z";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control2);

    control2.orientation.w = 1;
    control2.orientation.x = 0;
    control2.orientation.y = 0;
    control2.orientation.z = 1;
    control2.name = "rotate_y";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control2);
    control2.name = "move_y";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control2);
  }
  addMenu(int_marker);

//  // create a control which will move the box
//  // this control does not contain any markers,
//  // which will cause RViz to insert two arrows
//  visualization_msgs::InteractiveMarkerControl rotate_control;
//  rotate_control.name = "move_x";
//  rotate_control.interaction_mode =
//  visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//  // add the control to the interactive marker
//  int_marker.controls.push_back(rotate_control);

//  rotate_control.name = "move_y";
//  rotate_control.interaction_mode =
//  visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

//  // add the control to the interactive marker
//  int_marker.controls.push_back(rotate_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
  bind(&grasp_editor::markerFeedback, this, _1);
  server.insert(int_marker, bindedLoop);
  menu_handler.apply( server, int_marker.name );
  server.applyChanges();
}


void grasp_editor::addShape(uint32_t shape, geometry_msgs::PoseStamped optionalPose, double scale_x, double scale_y, double scale_z){

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = model_cloud_frame_id;
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = "basic_shape_"+object_name;
    int_marker.description = "basic_shape_"+object_name;
    int_marker.scale = 0.2f;
    //initialise the pose with the given values
    int_marker.pose.orientation.x = optionalPose.pose.orientation.x;
    int_marker.pose.orientation.y = optionalPose.pose.orientation.y;
    int_marker.pose.orientation.z = optionalPose.pose.orientation.z;
    int_marker.pose.orientation.w = optionalPose.pose.orientation.w;

    int_marker.pose.position.x = optionalPose.pose.position.x;
    int_marker.pose.position.y = optionalPose.pose.position.y;
    int_marker.pose.position.z = optionalPose.pose.position.z;


    currentBasicShapePose.pose.position.x = optionalPose.pose.position.x;
    currentBasicShapePose.pose.position.y = optionalPose.pose.position.y;
    currentBasicShapePose.pose.position.z = optionalPose.pose.position.z;

    currentBasicShapePose.pose.orientation.x = optionalPose.pose.orientation.x;
    currentBasicShapePose.pose.orientation.y = optionalPose.pose.orientation.y;
    currentBasicShapePose.pose.orientation.z = optionalPose.pose.orientation.z;
    currentBasicShapePose.pose.orientation.w = optionalPose.pose.orientation.w;

    // create a grey box marker
    visualization_msgs::Marker main_shape;
    main_shape.type = shape;
    main_shape.scale.x = scale_x;
    main_shape.scale.y = scale_y;
    main_shape.scale.z = scale_z;
    main_shape.color.r = 0.0;
    main_shape.color.g = 0.8;
    main_shape.color.b = 0.5;
    main_shape.color.a = 0.6;

    main_shape.pose.position.x = 0;
    main_shape.pose.position.y = 0;
    main_shape.pose.position.z = 0;

    main_shape.pose.orientation.x = 0.;
    main_shape.pose.orientation.y = 0.;
    main_shape.pose.orientation.z = 0.;
    main_shape.pose.orientation.w = 1.;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( main_shape );

    // add the control to the interactive marker
    int_marker.controls.push_back( box_control );

    {
      visualization_msgs::InteractiveMarkerControl control2;
      control2.orientation.w = 1;
      control2.orientation.x = 1;
      control2.orientation.y = 0;
      control2.orientation.z = 0;
      control2.name = "rotate_x";
      control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control2);
      control2.name = "move_x";
      control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control2);

      control2.orientation.w = 1;
      control2.orientation.x = 0;
      control2.orientation.y = 1;
      control2.orientation.z = 0;
      control2.name = "rotate_z";
      control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control2);
      control2.name = "move_z";
      control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control2);

      control2.orientation.w = 1;
      control2.orientation.x = 0;
      control2.orientation.y = 0;
      control2.orientation.z = 1;
      control2.name = "rotate_y";
      control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control2);
      control2.name = "move_y";
      control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control2);
    }
    boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
    bind(&grasp_editor::shapeMarkerFeedback, this, _1);
    server.insert(int_marker, bindedLoop);
    server.applyChanges();
    addedBasicShape = true;

//  }

//  marker_pub.publish(marker);
}

/*
 * callback function of the basic shape marker
 */
void grasp_editor::shapeMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){


//  ROS_INFO_STREAM("grasp_editor::shapeMarkerFeedback "<< feedback->marker_name << " is now at "
//      << feedback->pose.position.x << ", " << feedback->pose.position.y
//      << ", " << feedback->pose.position.z <<" orientation= "
//                   << feedback->pose.orientation.x<<" "
//                   <<feedback->pose.orientation.y<<" "
//                   << feedback->pose.orientation.z<<" "
//                   << feedback->pose.orientation.w<<" "

//                   );

  //check if we are moving the basic shape
  if (feedback->marker_name.find("grasps_basic_shape_") != 0)
  {
    //if the basic shape moved, the associated basic grasp shape should move accordingly
    currentBasicShapePose.pose.position.x = feedback->pose.position.x;
    currentBasicShapePose.pose.position.y = feedback->pose.position.y;
    currentBasicShapePose.pose.position.z = feedback->pose.position.z;

    currentBasicShapePose.pose.orientation.x = feedback->pose.orientation.x;
    currentBasicShapePose.pose.orientation.y = feedback->pose.orientation.y;
    currentBasicShapePose.pose.orientation.z = feedback->pose.orientation.z;
    currentBasicShapePose.pose.orientation.w = feedback->pose.orientation.w;



    visualization_msgs::InteractiveMarker basic_grasps_int_marker;
    bool exists_grasps = server.get ("grasps_basic_shape_"+object_name, basic_grasps_int_marker);
    if(exists_grasps){
      basic_grasps_int_marker.controls[0].markers[0].pose.position.x = feedback->pose.position.x;
      basic_grasps_int_marker.controls[0].markers[0].pose.position.y = feedback->pose.position.y;
      basic_grasps_int_marker.controls[0].markers[0].pose.position.z = feedback->pose.position.z;
      basic_shape_grasp_height = basic_grasps_int_marker.controls[0].markers[0].pose.position.z;

      basic_grasps_int_marker.controls[0].markers[0].pose.orientation.x = feedback->pose.orientation.x;
      basic_grasps_int_marker.controls[0].markers[0].pose.orientation.y = feedback->pose.orientation.y;
      basic_grasps_int_marker.controls[0].markers[0].pose.orientation.z = feedback->pose.orientation.z;
      basic_grasps_int_marker.controls[0].markers[0].pose.orientation.w = feedback->pose.orientation.w;

      //replace the marker
      boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
      bind(&grasp_editor::shapeMarkerFeedback, this, _1);
      server.insert(basic_grasps_int_marker, bindedLoop);
      server.applyChanges();
    }
  }
  else{


    currentBasicGraspShapePose.pose.position.x = feedback->pose.position.x;
    currentBasicGraspShapePose.pose.position.y = feedback->pose.position.y;
    currentBasicGraspShapePose.pose.position.z = feedback->pose.position.z;

    currentBasicGraspShapePose.pose.orientation.x = feedback->pose.orientation.x;
    currentBasicGraspShapePose.pose.orientation.y = feedback->pose.orientation.y;
    currentBasicGraspShapePose.pose.orientation.z = feedback->pose.orientation.z;
    currentBasicGraspShapePose.pose.orientation.w = feedback->pose.orientation.w;

    double x1 = currentBasicShapePose.pose.position.x;
    double y1 = currentBasicShapePose.pose.position.y;
    double z1 = currentBasicShapePose.pose.position.z;

    double x2 = currentBasicGraspShapePose.pose.position.x;
    double y2 = currentBasicGraspShapePose.pose.position.y;
    double z2 = currentBasicGraspShapePose.pose.position.z;

    double x = x2-x1, y = y2-y1, z = z2-z1;

    basic_shape_grasp_height = std::sqrt(x*x + y*y + z*z);
  }
  publishBasicShapeTf();




}

void grasp_editor::markerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){


  switch ( feedback->event_type )
    {

      case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        ROS_INFO_STREAM(" menu item " << feedback->menu_entry_id << " clicked from marker "<<feedback->marker_name );
        //check what is the menu id
        if(feedback->menu_entry_id == 1){
          //have to delete this marker
          server.erase(feedback->marker_name);
          server.applyChanges();
          //remove from the map of poses
          addedGraspPoses.erase (feedback->marker_name);
        }
        //grasp mode is now "top"
        else if(feedback->menu_entry_id == 3){
          //the grasp mode changed
          std::cout <<"the grasp mode changed to top"<<std::endl;
          addedGraspPoses[feedback->marker_name].type = "top";
        }
        //grasp mode is now "side"
        else if(feedback->menu_entry_id == 4){
          //the grasp mode changed
          std::cout <<"the grasp mode changed to side"<<std::endl;
          addedGraspPoses[feedback->marker_name].type = "side";
        }
        else if(feedback->menu_entry_id == 5){
          //the grasp mode changed
          std::cout <<"the grasp mode changed to other"<<std::endl;
          addedGraspPoses[feedback->marker_name].type = "other";
        }


        break;

      case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:



        ROS_INFO_STREAM( feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z <<" orientation= "
                         << feedback->pose.orientation.x<<" "
                         <<feedback->pose.orientation.y<<" "
                         << feedback->pose.orientation.z<<" "
                         << feedback->pose.orientation.w<<" "

                         );
        currentAddedGraspPose.pose = feedback->pose;
        addedGraspPoses[feedback->marker_name].grasp = currentAddedGraspPose;
        break;


  }



}

void grasp_editor::addMenu(visualization_msgs::InteractiveMarker& int_marker, std::string type){
  visualization_msgs::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";
  control.always_visible = true;

  //visualization_msgs::Marker cast_marker = dynamic_cast <visualization_msgs::Marker>(int_marker);
  //control.markers.push_back( cast_marker);
  int_marker.controls.push_back(control);

}

void grasp_editor::createMarker(geometry_msgs::PoseStamped& pose){

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = model_cloud_frame_id;
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = current_grasp_name;
  int_marker.description = "Grasp "+current_grasp_name;
  int_marker.scale = 0.2f;
  //initialise the pose with the given values
  int_marker.pose.orientation.x = pose.pose.orientation.x;
  int_marker.pose.orientation.y = pose.pose.orientation.y;
  int_marker.pose.orientation.z = pose.pose.orientation.z;
  int_marker.pose.orientation.w = pose.pose.orientation.w;

  int_marker.pose.position.x = pose.pose.position.x;
  int_marker.pose.position.y = pose.pose.position.y;
  int_marker.pose.position.z = pose.pose.position.z;

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.05;
  box_marker.scale.y = 0.10;
  box_marker.scale.z = 0.02;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  box_marker.pose.position.x = 0;
  box_marker.pose.position.y = 0;
  box_marker.pose.position.z = 0;

  box_marker.pose.orientation.x = 0.;
  box_marker.pose.orientation.y = 0.;
  box_marker.pose.orientation.z = 0.;
  box_marker.pose.orientation.w = 1.;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  {
    visualization_msgs::InteractiveMarkerControl control2;
    control2.orientation.w = 1;
    control2.orientation.x = 1;
    control2.orientation.y = 0;
    control2.orientation.z = 0;
    control2.name = "rotate_x";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control2);
    control2.name = "move_x";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control2);

    control2.orientation.w = 1;
    control2.orientation.x = 0;
    control2.orientation.y = 1;
    control2.orientation.z = 0;
    control2.name = "rotate_z";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control2);
    control2.name = "move_z";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control2);

    control2.orientation.w = 1;
    control2.orientation.x = 0;
    control2.orientation.y = 0;
    control2.orientation.z = 1;
    control2.name = "rotate_y";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control2);
    control2.name = "move_y";
    control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control2);
  }
  addMenu(int_marker);

//  // create a control which will move the box
//  // this control does not contain any markers,
//  // which will cause RViz to insert two arrows
//  visualization_msgs::InteractiveMarkerControl rotate_control;
//  rotate_control.name = "move_x";
//  rotate_control.interaction_mode =
//  visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//  // add the control to the interactive marker
//  int_marker.controls.push_back(rotate_control);

//  rotate_control.name = "move_y";
//  rotate_control.interaction_mode =
//  visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

//  // add the control to the interactive marker
//  int_marker.controls.push_back(rotate_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  boost::function<void (const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> bindedLoop =
  bind(&grasp_editor::markerFeedback, this, _1);
  server.insert(int_marker, bindedLoop);
  menu_handler.apply( server, int_marker.name );
  server.applyChanges();
}

void grasp_editor::on_sampleGraspPushButton_clicked(){

  //the point will be sampled on the frame of reference of the basic shape
  //check for the shape
  if(basicShapeType == visualization_msgs::Marker::CYLINDER){
    //need to sample a point from an ellipse
    double z = basic_shape_grasp_height;
    double a = basicShapeScaleX/2., b = basicShapeScaleY/2.;
    double a2 = a*a;
    //double b2 = b*b;
    //sample a point on the X axis from the interval [-a,a]
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(-a, +a);
    //Use dis to transform the random unsigned int generated by gen into a double in [1, 2)
    double x = dis(gen);
    //solve for the y and pick one of the two solutions
    double y = b * std::sqrt(1- (x*x)/a2);
    std::cout <<" sampled point = "<<x<<" "<<y<<" "<<z<<std::endl;

    geometry_msgs::Point p0, pf;
    p0.x = x;
    p0.y = y;
    p0.z = z;

    double s = 1.0;

    pf.x = (p0.x - s*a*p0.y/b);
    pf.y = (p0.y + s*b*p0.x/a);


    pf.z = z;


    addArrow(p0,pf);
  }

}

void grasp_editor::addArrow(geometry_msgs::Point& p0, geometry_msgs::Point& pf){
  visualization_msgs::Marker marker;
  marker.header.frame_id = basic_shape_frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.points.push_back(p0);
  marker.points.push_back(pf);

  marker.scale.x = 0.005;
  marker.scale.y = 0.005;
  marker.scale.z = 0.05;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  grasp_marker_pub.publish( marker );

}


void grasp_editor::loadGrasps(){
  addedGraspPoses.clear();
  index_grasps = 0;
  std::string fullpath=data_path.toStdString()+"/grasps.yaml";
  ROS_INFO_STREAM("Loading YAML from " << fullpath);



  std::ifstream in(fullpath);
  object_grasp_editor::ObjectGrasp g = g.loadFromYAML(in);
  basicShapeScaleX = g.basicShapeScaleX;
  basicShapeScaleY = g.basicShapeScaleY;
  basicShapeScaleZ = g.basicShapeScaleZ;
  basicShapeType = g.basicShapeType;
  std::cout <<"read scales= "<<basicShapeScaleX<<" "<<basicShapeScaleY<<" "<<basicShapeScaleZ<<std::endl;

  if(!(basicShapeScaleX == 0 || basicShapeScaleY == 0 || basicShapeScaleZ == 0 ))
    addShape(g.basicShapeType, g.basicShapePose, g.basicShapeScaleX, g.basicShapeScaleY, g.basicShapeScaleZ);
  m_ui.graspsTableWidget->setRowCount(g.graspList.grasps.size());
  m_ui.graspsTableWidget->setColumnCount(2);
  int i = 0;
  for(auto& grasp : g.graspList.grasps){
    std::cout <<"reading grasp pose: "<<grasp.grasp<<std::endl;
    //create interactive markers at those poses
    current_grasp_name = "grasp_"+object_name+"_"+std::to_string(index_grasps);
    geometry_msgs::PoseStamped graspStamped;
    graspStamped = grasp.grasp;
    addedGraspPoses[current_grasp_name].grasp = graspStamped;
    addedGraspPoses[current_grasp_name].type = grasp.type;


    m_ui.graspsTableWidget->setItem(i, 0, new QTableWidgetItem(current_grasp_name.c_str()));
    m_ui.graspsTableWidget->setItem(i, 1, new QTableWidgetItem(grasp.type.c_str()));

    createMarker(grasp.grasp);
    index_grasps++;
    i++;
  }
//  object_grasp_editor::ObjectGrasp::Ptr grasp = boost::make_shared<
//          object_grasp_editor::ObjectGrasp>(g);

//      //m_objectGrasps maps object names to stored grasping poses
//      m_objectGrasps[grasp->name] = grasp;
//    }



}

void grasp_editor::saveGrasps(){
  std::string fullpath=data_path.toStdString()+"/grasps.yaml";
  ROS_INFO_STREAM("Saving YAML to " << fullpath);
  std::ofstream myfile(fullpath.c_str());
  object_grasp_editor::ObjectGrasp grasp;
  grasp.name=current_grasp_name;
  grasp.basicShapeScaleX = basicShapeScaleX;
  grasp.basicShapeScaleY = basicShapeScaleY;
  grasp.basicShapeScaleZ = basicShapeScaleZ;
  grasp.basicShapeType = basicShapeType;

  grasp.basicShapePose = currentBasicShapePose;
  index_grasps++;

  grasp.axis=currentAddedGraspPose;
  /*
   * add empty grasp
   */


  grasp.graspList.header.frame_id=object_name+"_"+model_cloud_frame_id;
  //iterate over the grasps created

  for(auto& grasp_element : addedGraspPoses){
    std::cout <<" grasp map element: "<<grasp_element.first<<" : "<<grasp_element.second.grasp<<std::endl;
    object_grasp_editor::Grasp g;
    g.grasp = grasp_element.second.grasp;
    g.type = grasp_element.second.type;
    grasp.graspList.grasps.push_back(g);
  }


//  geometry_msgs::Pose gpose;
//  gpose.position.x = gpose.position.y = gpose.position.z = 0.;
//  gpose.orientation.w =1.0;
//  object_grasp_editor::Grasp g;
//  g.grasp = gpose;
//  g.type = object_grasp_editor::BASIC_GRASP;
//  grasp.graspList.grasps.push_back(g);


  myfile << grasp.serializeToYAML();
  myfile.close();
}

/*
 * an object has been selected in the views combo box
 * and now the point clouds can be loaded and published
 *
 *
 */
void grasp_editor::on_loadViewsPushButton_clicked(){

  std::vector<std::string> views_paths;
  int index = m_ui.objectsComboBox->currentIndex();
  std::string object_path = map_objects_paths[index];
  std::cout <<"selected item path: "<<object_path<<std::endl;
  //clear the combo box that contains the views of the object
  m_ui.viewsComboBox->clear();
  objects.clear();

  recursive_directory_iterator begin(object_path), end;
  std::vector<directory_entry> v(begin, end);
  std::cout << "There are " << v.size() << " files/directories: \n";
  for(auto& f: v){
      path localPath (f);
      if(is_directory(localPath))
       views_paths.push_back(f.path().string());
  }
  std::sort(views_paths.begin(), views_paths.end(), compareNatG);
  //create a new object
  ObjectPtr object(new Object);
  object->object_name = folderNameG(object_path);
  //load all the available views
  for(auto& f: views_paths){
    std::string cloud_path = f+ "/normalized_cloud.pcd";
    std::string q_path = f+ "/cam_T_view.tf";
    std::string cloud_to_0_path = f+ "/normalized_cloud_to_0.pcd";
    std::cout << "grasp_editor::on_loadViewsPushButton_clicked reading pcd file at " <<cloud_path << '\n';
    //create a Frame
    FramePtr frame(new Frame);

    if (pcl::io::loadPCDFile<PointT> (cloud_path, *(frame->pcloud)) == -1){
      ROS_ERROR_STREAM ("Couldn't read file"<< cloud_path <<"\n");
      continue;
    }

    else{

      //read the camera pose quaternion
      Eigen::Matrix3f cam_matrix;
      Eigen::read_binary(q_path.c_str(), cam_matrix);
      Eigen::Quaternionf q_temp(cam_matrix);
      frame->q = q_temp;

      object->frames_data.push_back(frame);
      QString text = QString::fromStdString(f);
      m_ui.viewsComboBox->addItem(text);

    }
    pcl::io::loadPCDFile<PointT> (cloud_to_0_path, *(frame->pcloud_normalized_to_0));

  }
  current_view_object = object;
  objects.push_back(object);

}

/*
 * show in rviz the selected view of the object
 * publishes a tf and the point cloud of the view
 *
 */
void grasp_editor::on_viewPushButton_clicked(){

  int index = m_ui.viewsComboBox->currentIndex();
  FramePtr frame = current_view_object->frames_data[index];
  std::string frame_id = current_view_object->object_name + "_norm_"+std::to_string(index);
  PointCloudTPtr& cloud = m_ui.normto0CheckBox->isChecked() ? frame->pcloud_normalized_to_0 : frame->pcloud;
  publishDebugTf(frame_id,index);
  cloud->header.stamp = 0;//ros::Time::now().toNSec();
  cloud->header.frame_id = frame_id;
  views_cloud_pub.publish(cloud);
  //update the quaternion
  m_ui.qxCamDoubleSpinBox->setValue(frame->q.x());
  m_ui.qyCamDoubleSpinBox->setValue(frame->q.y());
  m_ui.qzCamDoubleSpinBox->setValue(frame->q.z());
  m_ui.qwCamDoubleSpinBox->setValue(frame->q.w());

  //set the same values for the test registration tab
  m_ui.qx->setValue(frame->q.x());
  m_ui.qy->setValue(frame->q.y());
  m_ui.qz->setValue(frame->q.z());
  m_ui.qw->setValue(frame->q.w());



}

void grasp_editor::publishDebugTf(std::string frame_id,int i){

  static tf::TransformBroadcaster br;
  tf::Transform transform;

  transform.setOrigin( tf::Vector3(0, 0, 0) );
  transform.setRotation(tf::Quaternion(0,0,0,1));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", frame_id));


}

void grasp_editor::publishDebugClouds(ObjectPtr object){

  int i = 0;
  //iterates over the frames
  for(FramePtr frame : object->frames_data){
    std::string frame_id = object->object_name + "_normalized_"+std::to_string(i);
    publishDebugTf(frame_id,i);

    frame->pcloud->header.stamp = 0;//ros::Time::now().toNSec();
    frame->pcloud->header.frame_id = frame_id;
    cloud_pub.publish(frame->pcloud);
    i++;
    ros::Duration(0.5).sleep();
  }



}

/*
 * sets the path where the objects are located
 * to debug the extracted and normalized point clouds
 *
 */
void grasp_editor::on_objectsPathPushButton_clicked(){

  views_data_path = QFileDialog::getExistingDirectory(m_w,tr("Choose a Folder"),QDir::homePath());
  //clear the elements of the objects combo box
  m_ui.objectsComboBox->clear();
  m_ui.registerComboBox->clear();
  m_ui.pathLineEdit->setText(views_data_path);

  path root_path (views_data_path.toStdString());
  if (exists(root_path) && is_directory(root_path))
  {


    //iterate over the directories
    directory_iterator begin(root_path), end;
    std::vector<directory_entry> v(begin, end);
    std::cout << "There are " << v.size() << " files/directories: \n";
    //for each object directory create a new entry in vector objects and load its data
    int index = 0;
    for(auto& f: v){
        path object_path (f);
        if(is_directory(object_path)){

          std::string object_name = folderNameG(f.path().string());
          std::cout <<"------- Loading object_name="<<object_name<<std::endl;
          //add an entry to the combo box
          QString text = QString::fromStdString(object_name);
          m_ui.objectsComboBox->addItem(text);
          m_ui.registerComboBox->addItem(text);
          map_objects_paths[index] = f.path().string();
          index++;
        }

    }
  }


}


void grasp_editor::on_loadObjectPushButton_clicked(){

  data_path = QFileDialog::getExistingDirectory(m_w,tr("Choose a Folder"),QDir::homePath());
  QString separator("/");
  QStringList directories = data_path.split(separator);  
  object_name = directories.at(directories.size()-1).toLocal8Bit().constData();
  std::cout <<" object name=" <<object_name<<std::endl;
  basic_shape_frame_id = "basic_shape_"+object_name;
  //we should clear at this point any previously loaded data
  addedGraspPoses.clear();
  server.clear();
  server.applyChanges();

  loadData();


}

void grasp_editor::on_loadGraspsPushButton_clicked(){
  loadGrasps();
}

void grasp_editor::on_saveGraspPushButton_clicked(){
  saveGrasps();
}

/*
 * the path will be something like ..../tissue_box/0/
 * where the tissue_box_0.pcd has to be found
 */
void grasp_editor::load_main_cloud(std::string path_0){
  std::string path_cloud =  path_0 + "/merged_cloud.pcd";
  std::cout <<"looking for point cloud: "<<std::endl;
  path cloud(path_cloud);
  if (exists(cloud)){   // does p actually exist?

    std::cout <<" openning the merged point cloud file"<<std::endl;
    if (pcl::io::loadPCDFile<PointT> (path_cloud, *object_cloud_0) == -1){
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return;
    }
    //publish the point cloud for rviz
    object_cloud_0->header.stamp = 0;//ros::Time::now().toSec();
    object_cloud_0->header.frame_id = model_cloud_frame_id;
    cloud_pub.publish (object_cloud_0);

  }
  else{
    std::cout <<" the expected file merged_cloud.pcd does not exist"<<std::endl;
  }
}

void grasp_editor::load_relative_cloud(int i){
  std::string path_i(data_path.toStdString()+"/000"+std::to_string(i));
  path dir_path (path_i);
  if(is_directory(dir_path)){
    //load the merged_cloud file and the transformation
    std::string path_cloud =  path_i + "/merged_cloud.pcd";
    path cloud(path_cloud);
    if (exists(cloud)){


      //add an entry to the merged view combo box
      QString item = QString::fromStdString(path_cloud);
      std::cout<<"adding to mergedViewComboBox item="<<path_cloud<<std::endl;
      m_ui.mergedViewComboBox->addItem(item);

      PointCloudTPtr object_cloud(new PointCloudT);
      if (pcl::io::loadPCDFile<PointT> (path_cloud, *object_cloud) == -1){
        ROS_ERROR_STREAM ("Couldn't read the file "<<path_cloud);
        return;
      }
      else
        merged_clouds[path_cloud] = object_cloud;


    }

  }
}

void grasp_editor::simulateDataCloud(std::string& object_name, Eigen::Affine3f& object_pose, PointCloudTPtr& data_cloud){
  //get the merged point cloud of object_name
  PointCloudTPtr tmp_cloud(new PointCloudT);  
  pcl::transformPointCloud (*data_cloud, *tmp_cloud, object_pose );
  //add some noise on the translation component
//  double noise_y = 0.1; // 5 cms
//  for(PointT& p : *tmp_cloud){
//    p.y += noise_y; //these are camera coordinates, so we make the object a bit further away than the prediction
//  }
  data_cloud = tmp_cloud;

}

void grasp_editor::on_testDataCheckBox_clicked(){

  if(m_ui.testDataCheckBox->isChecked())
    m_ui.fullModelcheckBox->setChecked(false);

  else
    m_ui.fullModelcheckBox->setChecked(true);


}

void grasp_editor::on_fullModelcheckBox_clicked(){

  if(m_ui.fullModelcheckBox->isChecked())
    m_ui.testDataCheckBox->setChecked(false);

  else
    m_ui.testDataCheckBox->setChecked(true);

}


/*
 * loads a stored data cloud for testing the registration
 *
 */
void grasp_editor::on_loadDataCloudPushButton_clicked(){
  QString test_data_cloud_path = QFileDialog::getOpenFileName(m_w,tr("Choose a .pcd file"),QDir::homePath());
  std::string test_data_cloud_path_std_string = test_data_cloud_path.toStdString();
  std::cout <<" test_data_cloud_path_std_string=" <<test_data_cloud_path_std_string<<std::endl;
  if (pcl::io::loadPCDFile<PointT> (test_data_cloud_path_std_string, *test_data_cloud) == -1){
    return;
  }

  test_data_cloud->header.stamp = 0;//ros::Time::now().toNSec();
  test_data_cloud->header.frame_id = camera_frame;
  test_data_cloud_pub.publish(test_data_cloud);


}

/*
 * simulates we have a prediction from the network
 * publishes the registered point cloud and the grasps
 * like the on_registrationPushButton_clicked method
 * but uses the library interface
 *
 */
void grasp_editor::on_registerLibPushButton_clicked(){

//  //get the object name
//  std::string object_name = m_ui.registerComboBox->currentText().toStdString();

//  //get the "predicted" position
//  double x = m_ui.x->value();
//  double y = m_ui.y->value();
//  double z = m_ui.z->value();

//  //get the "predicted" orientation
//  double qx = m_ui.qx->value();
//  double qy = m_ui.qy->value();
//  double qz = m_ui.qz->value();
//  double qw = m_ui.qw->value();

//  //normalize the quaternion
//  tf::Quaternion q(qx,qy,qz,qw);
//  q.normalize();
//  m_ui.qx->setValue(q.x());
//  m_ui.qy->setValue(q.y());
//  m_ui.qz->setValue(q.z());
//  m_ui.qw->setValue(q.w());


//  //create pose for the data_cloud
//  PointCloudTPtr data_cloud(new PointCloudT);
//  Eigen::Affine3f data_pose;
//  Eigen::Vector3f t_data(x,y,z);
//  Eigen::Quaternionf q_data(qw,qx,qy,qz);
//  data_pose.matrix().block<3,3>(0,0) = q_data.toRotationMatrix(); //elegant way to assign the rotation component :|
//  data_pose.translation()  = t_data;
//  //path of the point cloud with merged views
//  std::string path_cloud_merged_views = views_data_path.toStdString() +"/"+object_name+"/0000/merged_cloud_to_0.pcd";
//  std::cout <<"reading point cloud file at path="<<path_cloud_merged_views<<std::endl;
//  pcl::io::loadPCDFile<PointT> (path_cloud_merged_views, *data_cloud);


//  //create the point cloud
//  if(m_ui.testDataCheckBox->isChecked()){
//    data_cloud = test_data_cloud;
//    //the predicted position is  now at the centroid of the cloud
//    //compute the projection of the camera average to the plane at the april tag
//    Eigen::Vector4f data_cloud_centroid_4f;

//    pcl::compute3DCentroid(*test_data_cloud, data_cloud_centroid_4f);
//    std::cout <<" test data cloud centroid: "<<data_cloud_centroid_4f[0]<<" "<<data_cloud_centroid_4f[1]<<" "<<data_cloud_centroid_4f[2]<<" "<<std::endl;
//    x = data_cloud_centroid_4f[0];
//    y = data_cloud_centroid_4f[1];
//    z = data_cloud_centroid_4f[2];
//    m_ui.x->setValue(x);
//    m_ui.y->setValue(y);
//    m_ui.z->setValue(z);
//    return;
//  }
//  else{
//    simulateDataCloud(object_name, data_pose, data_cloud);
//  }




//  std::vector<Eigen::Affine3d> grasping_poses;
//  arc_registration_lib.register_model_cloud( object_name, camera_frame,  data_pose, data_cloud, grasping_poses);
//  ROS_INFO_STREAM("arc_registration_lib.register_model_cloud returned this many grasping poses: "<<grasping_poses.size());
//  //call the registration service
//  arc_registration::Registration srv;
//  pcl::toROSMsg(*data_cloud, srv.request.dataCloud);
//  srv.request.predictedPose.position.x = x;
//  srv.request.predictedPose.position.y = y;
//  srv.request.predictedPose.position.z = z;

//  srv.request.predictedPose.orientation.x = qx;
//  srv.request.predictedPose.orientation.y = qy;
//  srv.request.predictedPose.orientation.z = qz;
//  srv.request.predictedPose.orientation.w = qw;

//  srv.request.cameraFrame = camera_frame;
//  srv.request.objectName = object_name;
//  srv.request.onlyPrediction = false;
//  std::cout <<"requesting pose of object:"<<srv.request.objectName<<" at "<< srv.request.predictedPose.position.x<<" "<< srv.request.predictedPose.position.y<<" "<< srv.request.predictedPose.position.z <<std::endl;
//  registration_client.call(srv);

}

/*
 * simulates we have a prediction from the network
 * publishes the registered point cloud and the grasps
 *
 */
void grasp_editor::on_registrationPushButton_clicked(){

  //get the object name
  std::string object_name = m_ui.registerComboBox->currentText().toStdString();

  //get the "predicted" position
  double x = m_ui.x->value();
  double y = m_ui.y->value();
  double z = m_ui.z->value();

  //get the "predicted" orientation
  double qx = m_ui.qx->value();
  double qy = m_ui.qy->value();
  double qz = m_ui.qz->value();
  double qw = m_ui.qw->value();

  //normalize the quaternion
  tf::Quaternion q(qx,qy,qz,qw);
  q.normalize();
  m_ui.qx->setValue(q.x());
  m_ui.qy->setValue(q.y());
  m_ui.qz->setValue(q.z());
  m_ui.qw->setValue(q.w());


  //create pose for the data_cloud
  PointCloudTPtr data_cloud(new PointCloudT);
  Eigen::Affine3f data_pose;
  Eigen::Vector3f t_data(x,y,z);
  Eigen::Quaternionf q_data(qw,qx,qy,qz);
  data_pose.matrix().block<3,3>(0,0) = q_data.toRotationMatrix(); //elegant way to assign the rotation component :|
  data_pose.translation()  = t_data;
  //path of the point cloud with merged views
  std::string path_cloud_merged_views = views_data_path.toStdString() +"/"+object_name+"/0000/merged_cloud_to_0.pcd";
  std::cout <<"reading point cloud file at path="<<path_cloud_merged_views<<std::endl;
  pcl::io::loadPCDFile<PointT> (path_cloud_merged_views, *data_cloud);


  //create the point cloud
  if(m_ui.testDataCheckBox->isChecked()){
    data_cloud = test_data_cloud;
    //the predicted position is  now at the centroid of the cloud    
    Eigen::Vector4f data_cloud_centroid_4f;

    pcl::compute3DCentroid(*test_data_cloud, data_cloud_centroid_4f);
    std::cout <<" test data cloud centroid: "<<data_cloud_centroid_4f[0]<<" "<<data_cloud_centroid_4f[1]<<" "<<data_cloud_centroid_4f[2]<<" "<<std::endl;
    x = data_cloud_centroid_4f[0];
    y = data_cloud_centroid_4f[1];
    z = data_cloud_centroid_4f[2];
    m_ui.x->setValue(x);
    m_ui.y->setValue(y);
    m_ui.z->setValue(z);
  }
  else{

    simulateDataCloud(object_name, data_pose, data_cloud);
    //the predicted position is  now at the centroid of the cloud
    Eigen::Vector4f data_cloud_centroid_4f;

    pcl::compute3DCentroid(*data_cloud, data_cloud_centroid_4f);
    std::cout <<" test data cloud centroid: "<<data_cloud_centroid_4f[0]<<" "<<data_cloud_centroid_4f[1]<<" "<<data_cloud_centroid_4f[2]<<" "<<std::endl;
    x = data_cloud_centroid_4f[0];
    y = data_cloud_centroid_4f[1];
    z = data_cloud_centroid_4f[2];
    m_ui.x->setValue(x);
    m_ui.y->setValue(y);
    m_ui.z->setValue(z);
  }


  //add noise to the data cloud
//  for(PointT& p : data_cloud->points){
//    p.x += 0.1;
//  }



  //call the registration service
  arc_registration::Registration srv;
  pcl::toROSMsg(*data_cloud, srv.request.dataCloud);
  srv.request.predictedPose.position.x = x;
  srv.request.predictedPose.position.y = y;
  srv.request.predictedPose.position.z = z;

  srv.request.predictedPose.orientation.x = qx;
  srv.request.predictedPose.orientation.y = qy;
  srv.request.predictedPose.orientation.z = qz;
  srv.request.predictedPose.orientation.w = qw;

  srv.request.cameraFrame = camera_frame;
  srv.request.objectName = object_name;
  srv.request.onlyPrediction = false;
  std::cout <<"requesting pose of object:"<<srv.request.objectName<<" at "<< srv.request.predictedPose.position.x<<" "<< srv.request.predictedPose.position.y<<" "<< srv.request.predictedPose.position.z <<std::endl;
  registration_client.call(srv);
  //mrsmap_reg_client.call(srv);

}

void grasp_editor::on_mrsMapsPushButton_clicked(){

  //get the object name
  std::string object_name = m_ui.registerComboBox->currentText().toStdString();

  //get the "predicted" position
  double x = m_ui.x->value();
  double y = m_ui.y->value();
  double z = m_ui.z->value();

  //get the "predicted" orientation
  double qx = m_ui.qx->value();
  double qy = m_ui.qy->value();
  double qz = m_ui.qz->value();
  double qw = m_ui.qw->value();

  //normalize the quaternion
  tf::Quaternion q(qx,qy,qz,qw);
  q.normalize();
  m_ui.qx->setValue(q.x());
  m_ui.qy->setValue(q.y());
  m_ui.qz->setValue(q.z());
  m_ui.qw->setValue(q.w());


  //create pose for the data_cloud
  PointCloudTPtr data_cloud(new PointCloudT);
  Eigen::Affine3f data_pose;
  Eigen::Vector3f t_data(x,y,z);
  Eigen::Quaternionf q_data(qw,qx,qy,qz);
  data_pose.matrix().block<3,3>(0,0) = q_data.toRotationMatrix(); //elegant way to assign the rotation component :|
  data_pose.translation()  = t_data;
  //path of the point cloud with merged views
  std::string path_cloud_merged_views = views_data_path.toStdString() +"/"+object_name+"/0000/merged_cloud_to_0.pcd";
  std::cout <<"reading point cloud file at path="<<path_cloud_merged_views<<std::endl;
  pcl::io::loadPCDFile<PointT> (path_cloud_merged_views, *data_cloud);


  //create the point cloud
  if(m_ui.testDataCheckBox->isChecked()){
    data_cloud = test_data_cloud;
    //the predicted position is  now at the centroid of the cloud
    Eigen::Vector4f data_cloud_centroid_4f;

    pcl::compute3DCentroid(*test_data_cloud, data_cloud_centroid_4f);
    std::cout <<" test data cloud centroid: "<<data_cloud_centroid_4f[0]<<" "<<data_cloud_centroid_4f[1]<<" "<<data_cloud_centroid_4f[2]<<" "<<std::endl;
    x = data_cloud_centroid_4f[0];
    y = data_cloud_centroid_4f[1];
    z = data_cloud_centroid_4f[2];
    m_ui.x->setValue(x);
    m_ui.y->setValue(y);
    m_ui.z->setValue(z);
  }
  else{

    simulateDataCloud(object_name, data_pose, data_cloud);
    //the predicted position is  now at the centroid of the cloud
    Eigen::Vector4f data_cloud_centroid_4f;

    pcl::compute3DCentroid(*data_cloud, data_cloud_centroid_4f);
    std::cout <<" test data cloud centroid: "<<data_cloud_centroid_4f[0]<<" "<<data_cloud_centroid_4f[1]<<" "<<data_cloud_centroid_4f[2]<<" "<<std::endl;
    x = data_cloud_centroid_4f[0];
    y = data_cloud_centroid_4f[1];
    z = data_cloud_centroid_4f[2];
    m_ui.x->setValue(x);
    m_ui.y->setValue(y);
    m_ui.z->setValue(z);
  }

  //add noise to the data cloud
//  for(PointT& p : data_cloud->points){
//    p.x += 0.1;
//  }



  //call the registration service
  arc_registration::Registration srv;
  pcl::toROSMsg(*data_cloud, srv.request.dataCloud);
  srv.request.predictedPose.position.x = x;
  srv.request.predictedPose.position.y = y;
  srv.request.predictedPose.position.z = z;

  srv.request.predictedPose.orientation.x = qx;
  srv.request.predictedPose.orientation.y = qy;
  srv.request.predictedPose.orientation.z = qz;
  srv.request.predictedPose.orientation.w = qw;

  srv.request.cameraFrame = camera_frame;
  srv.request.objectName = object_name;
  srv.request.onlyPrediction = false;
  std::cout <<"requesting pose of object:"<<srv.request.objectName<<" at "<< srv.request.predictedPose.position.x<<" "<< srv.request.predictedPose.position.y<<" "<< srv.request.predictedPose.position.z <<std::endl;
  //registration_client.call(srv);
  mrsmap_reg_client.call(srv);
}

void grasp_editor::on_viewMergedPushButton_clicked(){

  std::string path_merged_cloud = m_ui.mergedViewComboBox->currentText().toStdString();
  PointCloudTPtr cloud = merged_clouds[path_merged_cloud];
  //publish the point cloud for rviz
  cloud->header.stamp = ros::Time::now().toSec();
  cloud->header.frame_id = model_cloud_frame_id;
  cloud_pub.publish (cloud);

}


/*
 * the path will be something like ..../tissue_box/
 *
 */
void grasp_editor::loadData(){
    path p (data_path.toStdString());

    std::cout <<"the chosen directory is "<<data_path.toStdString()<<std::endl;

    if (exists(p))    // does p actually exist?
     {
       if (is_regular_file(p))        // is p a regular file?
         std::cout << p << " size is " << file_size(p) << '\n';

       else if (is_directory(p)){      // is p a directory?

           recursive_directory_iterator begin(p), end;
           std::vector<directory_entry> v(begin, end);
           std::cout << "There are " << v.size() << " files/directories: \n";
           for(auto& f: v){
               path localPath (f);
               if(is_directory(localPath))
                image_paths.push_back(f.path().string());
           }
           std::sort(image_paths.begin(), image_paths.end(), compareNatG);
           for(auto& f: image_paths){
               std::cout << f << '\n';
               //loadFrame(f);
           }
       }
       else
         std::cout << p << "exists, but is neither a regular file nor a directory\n";

    load_main_cloud(data_path.toStdString()+"/0000");
    for(int i=0;i<9;i++)
      load_relative_cloud(i);
    }
     else
       std::cout << p << "does not exist\n";

}

void grasp_editor::publishBasicShapeTf(){
  static tf::TransformBroadcaster br;  
  tf::Transform transform;
  if(addedBasicShape){
    transform.setOrigin( tf::Vector3(currentBasicShapePose.pose.position.x, currentBasicShapePose.pose.position.y, currentBasicShapePose.pose.position.z) );
    tf::Quaternion q(currentBasicShapePose.pose.orientation.x, currentBasicShapePose.pose.orientation.y, currentBasicShapePose.pose.orientation.z, currentBasicShapePose.pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), model_cloud_frame_id, basic_shape_frame_id));
  }

}

grasp_editor::~grasp_editor()
{
  thread.join();
}

}
PLUGINLIB_EXPORT_CLASS(grasp_editor::grasp_editor, rviz::Display)
