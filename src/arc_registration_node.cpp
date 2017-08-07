#include <boost/filesystem.hpp>
#include <iostream>
#include <iterator>

#include <tf_conversions/tf_eigen.h>
#include "arc_registration/arc_registration_node.h"
#include <eigen_conversions/eigen_msg.h>
#include <rosmrsmap/ArcRegisterMap.h> //registration of MRSMaps

//#include <boost/serialization/vector.hpp>



using namespace boost::filesystem;


/*
 * comparison function to sort filenames in a natural order
 * e.g.:
 *  1.txt
 *  2.txt
 *  10.txt
 *  20.txt
 *
 */
bool compareNat(const std::string& a, const std::string& b)
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
            return compareNat(a.substr(1), b.substr(1));
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
    return (compareNat(anew, bnew));
}

std::string folderName (const std::string& str)
{
  size_t found;
  found=str.find_last_of("/\\");
  std::cout << " folder: " << str.substr(0,found) << std::endl;
  std::cout << " file: " << str.substr(found+1) << std::endl;
  return str.substr(found+1);
}

ARCRegistration::ARCRegistration(ros::NodeHandle* nodehandle):nh_(*nodehandle), do_publish_tfs(false)
{

    XmlRpc::XmlRpcValue topicList;
    std::vector<std::string> registration_objects_paths;

    ROS_INFO("in class constructor of ExampleRosClass");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeServices();
    std::string home(std::getenv("HOME"));


    //get path parameter
    ROS_INFO_STREAM("nh_.getNamespace()="<<nh_.getNamespace()); 

    if (nh_.getParam("/arcRegistrationNode/registration_objects", topicList))
    {
      std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
      for (i = topicList.begin(); i != topicList.end(); i++)
      {
        std::string topic_name;
        std::string topic_type;

        topic_name = i->first;
        std::cout <<"i->first="<< i->first <<" i->second="<<i->second<<std::endl;
        //topic_type.assign(i->second["topic_type"]);
        std::string local_path = i->second;

        registration_objects_paths.push_back(local_path);
        ROS_INFO_STREAM("parameter from registration_objects: "<<local_path<<" size="<<registration_objects_paths.size());

      }

      loadAllObjects(registration_objects_paths);
    }
    else{
      nh_.param<std::string>(nh_.getNamespace()+"/data_path", data_path, home+"/Datasets/ARC/20_7/");
      ROS_INFO_STREAM("data_path="<<data_path);

      //loads all the objects views and grasp information
      loadAllObjects(data_path);

    }

    //start the publisher
    thread= std::thread(&ARCRegistration::publisher_thread, this);

}

/*
 * Library constructor. Requires the path of the root folder where all the objects are contained
 */
ARCRegistration::ARCRegistration(const std::string& data_path): nh_("arcRegistrationNode"), data_path(data_path), do_publish_tfs(false)
{
  ROS_INFO_STREAM("ARCRegistration::ARCRegistration namespace:"<<nh_.getNamespace());
  initializePublishers();
  //loads all the objects views and grasp information
  loadAllObjects(data_path);
  //start the publisher
  thread= std::thread(&ARCRegistration::publisher_thread, this);

}

ARCRegistration::~ARCRegistration(){
  thread.join();
}


void ARCRegistration::publisher_thread(){

  while(ros::ok()){

    if(do_publish_tfs){
      object_stamped_transform.stamp_ =  ros::Time::now();
      br.sendTransform(object_stamped_transform);
      for(tf::StampedTransform& grasp_tf : grasp_tfs){
        grasp_tf.stamp_ =  ros::Time::now();
        br.sendTransform(grasp_tf);
      }
    }
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }

}


/*
 * The all_objects_path should point to the root where the data
 * has been captured. It will iterate over the objects and load
 * their data
 *
 */
bool ARCRegistration::loadAllObjects(std::vector<std::string>& registration_objects_paths){


    //for each object directory create a new entry in vector objects and load its data
    for(std::string& f: registration_objects_paths){
        path object_path (f);
        if(is_directory(object_path)){

          ObjectPtr object(new Object);
          object->object_name= folderName(f);
          std::cout <<"------- Loading object_name="<<object->object_name<<std::endl;
          object->object_model_path = f;
          loadObjectModelData(object);
          objects.push_back(object);
          objects_map[object->object_name] = object;

          //image_paths.push_back(f.path().string());
        }

    }
    return true;

}

/*
 * The all_objects_path should point to the root where the data
 * has been captured. It will iterate over the objects and load
 * their data
 *
 */
bool ARCRegistration::loadAllObjects(const std::string& all_objects_path){

  path root_path (all_objects_path);
  if (exists(root_path) && is_directory(root_path))
  {


    //iterate over the directories
    directory_iterator begin(root_path), end;
    std::vector<directory_entry> v(begin, end);
    std::cout << "There are " << v.size() << " files/directories: \n";
    //for each object directory create a new entry in vector objects and load its data
    for(auto& f: v){
        path object_path (f);
        if(is_directory(object_path)){

          ObjectPtr object(new Object);
          object->object_name= folderName(f.path().string());
          std::cout <<"------- Loading object_name="<<object->object_name<<std::endl;
          object->object_model_path = f.path().string();
          loadObjectModelData(object);
          objects.push_back(object);
          objects_map[object->object_name] = object;

          //image_paths.push_back(f.path().string());
        }

    }


    return true;
  }
  else
    return false;
}


void ARCRegistration::computeObjectCentroid(ObjectPtr& object){

  Eigen::Vector4f data_cloud_centroid_4f;

  pcl::compute3DCentroid(*(object->merged_cloud), data_cloud_centroid_4f);
  object->offset_centroid_x = data_cloud_centroid_4f[0];
  object->offset_centroid_y = data_cloud_centroid_4f[1];
  object->offset_centroid_z = data_cloud_centroid_4f[2];
}

/*
 * Loads all the views captured for one object model
 *
 */
void ARCRegistration::loadObjectModelData(ObjectPtr object){

  std::vector<std::string> views;
  path p (object->object_model_path);
  //read the merged_cloud file for the first view
  std::string merged_cloud_path = object->object_model_path+ "/0000/merged_cloud.pcd";
  pcl::io::loadPCDFile<PointT> (merged_cloud_path, *(object->merged_cloud));
  computeObjectCentroid(object);
  ROS_INFO_STREAM("read model from "<<merged_cloud_path<<" this many points: "<<object->merged_cloud->points.size());
  ROS_INFO("ARCRegistration::loadObjectModelData calling loadGrasps()");
  loadGrasps(object->object_model_path, object->grasps);
  ROS_INFO_STREAM("loaded grasps: "<<object->grasps.graspList.grasps.size());

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
            views.push_back(f.path().string());
       }
       std::sort(views.begin(), views.end(), compareNat);
       for(auto& view_path: views){
           std::cout << view_path << '\n';
           FramePtr frame(new Frame);
           frame->path = view_path;
           loadFrame(frame);
           object->frames_data.push_back(frame);
       }
   }

   else
     std::cout << p << "exists, but is neither a regular file nor a directory\n";
  }
  else
   std::cout << p << "does not exist\n";
}

/*
 * Loads one captured view of one object. The path specified
 * in directory has to be of the form .../<object_name>/<0,1>/<index>
 *
 */
void ARCRegistration::loadFrame(FramePtr frame){

  std::string path_normalized_cloud_to_0 =  frame->path+"/normalized_cloud_to_0.pcd";
  std::string path_normalized_cloud      =  frame->path+"/normalized_cloud.pcd";  
  std::string path_pose =   frame->path+"/cam_T_view.tf";
  //the files naming should be consistent, e.g.: directory/rgb.png

  frame->rgb = cv::imread(frame->path+"/rgb.png");
  //frame.depth = cv::imread(directory+"/rgb.png");
  frame->mask = cv::imread(frame->path+"/mask.png");


  //read depth frame
  cv::FileStorage fs_read(frame->path+"/depth.xml", cv::FileStorage::READ);
  if (fs_read.isOpened()) {

      fs_read["depth"] >> frame->depth;
      fs_read.release();
  }

  //read .pcd file
  if (pcl::io::loadPCDFile<PointT> (path_normalized_cloud, *(frame->pcloud)) == -1){
    ROS_WARN_STREAM("Couldn't read the .pcd file "<<path_normalized_cloud);
    return;
  }
  if (pcl::io::loadPCDFile<PointT> (path_normalized_cloud_to_0, *(frame->pcloud_normalized_to_0)) == -1){
    frame->pcloud_normalized_to_0 = frame->pcloud;
  }

  //read the transformation
  Eigen::Matrix3f matrix;
  std::cout<<"reading pose from "<<path_pose<<std::endl;
  Eigen::read_binary(path_pose.c_str(), matrix);
  Eigen::Quaternionf q(matrix);
  //add this frame to the map<Quaternion,Frame>
  frame->q = q;


  //frame->frames_data.push_back(frame);
  //depth.convertTo(m_current_depth_float,CV_32FC1);
}


double ARCRegistration::cgicp(
  PointCloudT::Ptr& cloud_1,
  PointCloudT::Ptr& cloud_2,
  PointCloudT::Ptr& out,
  Eigen::Matrix4f& transform)
{

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBA>);
  PointCloudTPtr tmp(new PointCloudT);
  std::vector<int> indices1, indices2;
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
  pcl::removeNaNFromPointCloud(*cloud_1, *tmp, indices1);
  pcl::removeNaNFromPointCloud(*cloud_2, *tmp, indices2);
  pcl::copyPointCloud(*cloud_1,indices1,  *cloud1_rgba);
  pcl::copyPointCloud(*cloud_2,indices2,  *cloud2_rgba);

// 	transform = Eigen::Matrix<float, 4, 4>::Identity();

  std::cout <<"cloud1_rgba points="<<cloud1_rgba->points.size()<<" cloud2_rgba points="<<cloud2_rgba->points.size()<<std::endl;
  pcl::GeneralizedIterativeClosestPoint6D reg; //by default alpha = 0.032
  reg.setInputSource (cloud1_rgba);
  reg.setInputTarget (cloud2_rgba);
  reg.setSearchMethodTarget (tree);
  reg.setMaximumIterations (200);
  reg.setTransformationEpsilon (1e-8);
  reg.setMaxCorrespondenceDistance (0.20);

  // Register
  out = PointCloudTPtr(new PointCloudT);
  reg.align (*output, transform.matrix());

  pcl::copyPointCloud(*output, *out);
  double score = reg.getFitnessScore();
  std::cout << "colour icp has converged:" << reg.hasConverged() << " score: " << score
  << std::endl;
  transform = reg.getFinalTransformation();

  return score;

}


/*
 * Registers cloud_1 into cloud_2 using Generalized ICP
 * and using as initialization the transformation transform
 * The result is output in transform
 */
double ARCRegistration::gicp(PointCloudT::Ptr& cloud_1,
                     PointCloudT::Ptr& cloud_2,
                     PointCloudT::Ptr& out,
                     Eigen::Matrix4f& transform){


    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;


    gicp.setInputSource(cloud_1);
    gicp.setInputTarget(cloud_2);
    gicp.setMaximumIterations (200);
    gicp.setTransformationEpsilon (1e-9);
    gicp.setMaxCorrespondenceDistance (0.25);

    // run registration and get transformation
    out = PointCloudTPtr(new PointCloudT);
    gicp.align(*out);
    double score = gicp.getFitnessScore();
    std::cout <<" gicp score = "<<score<<std::endl;    
    transform = gicp.getFinalTransformation();
    std::cout <<"computed transformation="<<std::endl<<transform<<std::endl;
    return score;
}
/*
 * returns the frame whose orientation is closest to the one given
 *
 */
int ARCRegistration::getClosestFrame(ObjectPtr& object, Eigen::Quaternionf& q_predicted){

  int index = -1, i=0;
  double min_dist = std::numeric_limits<double>::max();
  double x1 = q_predicted.x();
  double y1 = q_predicted.y();
  double z1 = q_predicted.z();
  double w1 = q_predicted.w();
  //iterate over the stored frames
  for(FramePtr& frame : object->frames_data){
    double x2 = frame->q.x();
    double y2 = frame->q.y();
    double z2 = frame->q.z();
    double w2 = frame->q.w();

    double x = (x2-x1);
    double y = (y2-y1);
    double z = (z2-z1);
    double w = (w2-w1);
    double dist = std::sqrt(x*x+y*y+z*z+w*w);
    std::cout <<"q_predicted="<<q_predicted.x()<<" "<<q_predicted.y()<<" "<<q_predicted.z()<<" "<<q_predicted.w()<<" "<<std::endl;
    std::cout <<"q="<<x2<<" "<<y2<<" "<<z2<<" "<<w2<<" "<<std::endl;

    if(dist < min_dist){
      min_dist = dist;
      index = i;
    }
    std::cout <<"dist(q_predicted,q)="<<dist<<" min_dist = "<<min_dist<<std::endl;
    std::cout <<"closes q index="<<index<<std::endl;
    i++;
  }
  std::cout<<"closest quaternion, index "<<index<< " is "<<object->frames_data[index]->q.x()<<" "<<object->frames_data[index]->q.y()<<" "<<object->frames_data[index]->q.z()<<" "<<object->frames_data[index]->q.w()<<std::endl;
  return index;

}
void ARCRegistration::loadGrasps(const std::string& object_grasp_path, object_grasp_editor::ObjectGrasp& g)
{
  std::string fullpath=object_grasp_path+"/grasps.yaml";
  ROS_INFO_STREAM("Loading YAML from " << fullpath);
  std::ifstream in(fullpath);
  g = g.loadFromYAML(in);
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void ARCRegistration::initializeSubscribers()
{
  mrsmap_reg_client = nh_.serviceClient<rosmrsmap::ArcRegisterMap>("/register_map/arc_register");
}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void ARCRegistration::initializeServices()
{
    ROS_INFO("Initializing Services");
    registration_service_ = nh_.advertiseService("registration",
                                                   &ARCRegistration::registrationServiceCallback,
                                                   this);

    mrsregistration_service_ = nh_.advertiseService("mrsmap_registration",
                                                   &ARCRegistration::registrationMRSMapServiceCallback,
                                                   this);
    // add more services here, as needed
}

//member helper function to set up publishers;
void ARCRegistration::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    minimal_publisher_ = nh_.advertise<std_msgs::Float32>("exampleMinimalPubTopic", 1, true);

    //img_pub_ = it_.advertise("rgb", 1);
    //depth_pub_ = it_.advertise("depth", 1);
    registered_cloud_pub_ = nh_.advertise<PointCloudT> ("registered_model_cloud", 1);
    data_cloud_pub_ = nh_.advertise<PointCloudT> ("data_cloud", 1);

    model_cloud_pub_= nh_.advertise<PointCloudT> ("predicted_model_cloud", 1);


}

/*
 *
 * publishes the point cloud of the registered view of the object and the tf
 *
 */
void ARCRegistration::publishRegisteredView(ObjectPtr& object, int closest_view_index, Eigen::Affine3f& transformation, const std::string& camera_frame){
  FramePtr& frame = object->frames_data[closest_view_index];
  std::cout <<"publishing view at path="<<frame->path<<std::endl;
  //transform the view with the translation + rotation of Eigen::Affine3f& transformation

  /*
   *   Eigen::Matrix3d R;
   *   // Find your Rotation Matrix
   *   Eigen::Vector3d T;
   *   // Find your translation Vector
   *   Eigen::Matrix4d Trans; // Your Transformation Matrix
   *   Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
   *   Trans.block<3,3>(0,0) = R;
   *   Trans.rightCols<1>() = T;
   */

  //Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
  //std::cout <<"translation.rightCols<1>().size()="<<translation.block<3,1>(0,3).size()<<" transformation.translation().size()="<<transformation.translation().size()<<std::endl;
  //translation.block<3,1>(0,3) = transformation.translation();
  //std::cout <<"publishing the view at "<<translation<<std::endl;
  PointCloudTPtr transformCloud(new PointCloudT);
  //pcl::transformPointCloud (*(frame->pcloud_normalized_to_0), *transformCloud, translation );
  transformCloud = frame->pcloud_normalized_to_0;

  transformCloud->header.frame_id = object->object_name;
  //Eigen::Affine3f transformationMatrix(translation/*.inverse()*/);
  std::cout <<"publishing the view at "<<transformation.matrix()<<std::endl;
  std::cout <<"object->object_name (frame_id) = "<<object->object_name<<std::endl;
  publishRegisteredObjectTf(transformation, camera_frame, object->object_name);
  ros::spinOnce();
  transformCloud->header.stamp = 0;
  registered_cloud_pub_.publish (transformCloud);
  ros::spinOnce();
}

/*
 *
 * publishes the tf of the object and the grasps
 *
 */
void ARCRegistration::publishRegisteredView(ObjectPtr& object, Eigen::Affine3f& transformation, const  std::string& camera_frame){



  std::cout <<"publishing the view at "<<transformation.matrix()<<std::endl;
  std::cout <<"object->object_name (frame_id) = "<<object->object_name<<std::endl;
  publishRegisteredObjectTf(transformation, camera_frame, object->object_name);
  ros::spinOnce();

}


/*
 * publishes the transformation between the camera_frame and the object frame of reference
 *
 */
void ARCRegistration::publishRegisteredObjectTf(Eigen::Affine3f& pose, const  std::string& camera_frame , const  std::string& object_frame_id){

  tf::Transform object_transform;


  object_transform.setOrigin( tf::Vector3(pose.matrix()(0,3),pose.matrix()(1,3),pose.matrix()(2,3)) );
  Eigen::Quaternionf q(pose.rotation());

  tf::Quaternion qtf(q.x(), q.y(),q.z(),q.w());
  object_transform.setRotation(qtf);
  object_stamped_transform = tf::StampedTransform(object_transform, ros::Time::now(),  camera_frame,  object_frame_id);
  br.sendTransform(object_stamped_transform);
  do_publish_tfs = true;
  std::cout <<"do_publish_tfs = true"<< std::endl;

}

/*
 * iterates over the defined grasps and publishes the in the frame
 * of reference of the registered object
 *
 */
void ARCRegistration::publishGrasps(ObjectPtr& object){

  grasp_tfs.clear(); //clear whatever grasp tfs
  //iterate over the grasps
  int i=0;
  ROS_INFO_STREAM("object->grasps.graspList.grasps.size()= "<<object->grasps.graspList.grasps.size());
  for(object_grasp_editor::Grasp grasp : object->grasps.graspList.grasps){
    std::string grasp_frame_id = "grasp_"+object->object_name+std::to_string(i);
    ROS_INFO_STREAM("publishing grasp "<<grasp_frame_id);
    //grasp.grasp //type geometry_msgs::PoseStamped

    tf::Transform transform;


    transform.setOrigin( tf::Vector3(grasp.grasp.pose.position.x,grasp.grasp.pose.position.y,grasp.grasp.pose.position.z ));

    tf::Quaternion qtf(grasp.grasp.pose.orientation.x,grasp.grasp.pose.orientation.y,grasp.grasp.pose.orientation.z, grasp.grasp.pose.orientation.w);

    transform.setRotation(qtf);
    tf::StampedTransform grasp_tf(transform, ros::Time::now(),  object->object_name,  grasp_frame_id);
    br.sendTransform(grasp_tf);
    grasp_tfs.push_back(grasp_tf);
    i++;

  }
  ros::Duration(0.1).sleep(); //let the broadcaster start publishing the grasp tfs


}

void ARCRegistration ::publishDataCloud(PointCloudTPtr& data_cloud){

  data_cloud->header.stamp = 0;
  data_cloud_pub_.publish (data_cloud);

}


void ARCRegistration ::publishModelCloud(PointCloudTPtr& model_cloud){


  model_cloud->header.stamp = 0;
  model_cloud_pub_.publish (model_cloud);

}

void ARCRegistration ::constructModelCloud(ObjectPtr& object,int closest_view_index ,Eigen::Affine3f& predicted_object_pose, PointCloudTPtr& model_cloud){


    FramePtr& frame = object->frames_data[closest_view_index];
    //subtract the centroid offset first
//    PointCloudT copy_frame_cloud;
//    pcl::copyPointCloud(*(frame->pcloud_normalized_to_0), copy_frame_cloud);
//    for(PointT& p : copy_frame_cloud){
//      p.x -= object->offset_centroid_x;
//      p.y -= object->offset_centroid_y;
//      p.z -= object->offset_centroid_z;
//    }
    pcl::transformPointCloud (*(frame->pcloud_normalized_to_0), *model_cloud, predicted_object_pose );


}


/*
 * This method is the library alternative to the registrationServiceCallback
 * -std::string object_name [IN]
 * -std::string camera_frame [IN]
 * -geometry_msgs/Pose predictedPose [IN]
 * -PointCloudTPtr& data_cloud [IN]
 * -std::vector<Eigen::Affine3f>& grasping_poses [OUT]
 *
 *  returns the score of the GICP method. If the requested object was not found in the database it returns std::numeric_limits<double>::min()
 */

double ARCRegistration::register_model_cloud(const std::string& object_name, const std::string& camera_frame, Eigen::Affine3f& cam_T_pred_pose, PointCloudTPtr& data_cloud, std::vector<Eigen::Affine3d>& grasping_poses){

  if ( objects_map.find(object_name) == objects_map.end() ) {
    // not found
    return std::numeric_limits<double>::min();
  }
  else{
    //construct the point cloud of the model
    ObjectPtr object = objects_map[object_name];
    double score = 0.;
    Eigen::Affine3f cam_T_object;
    Eigen::Quaternionf q_predicted(cam_T_pred_pose.rotation()); //predicted orientation of the object
    int closest_q_index = getClosestFrame(object, q_predicted); //gets the closest view in the database to the predicted orientation
    PointCloudTPtr predicted_model_cloud(new PointCloudT), registered_cloud(new PointCloudT);
    Eigen::Affine3f pred_pose_T_data = Eigen::Affine3f::Identity();

//    //need to subtract the centroid offset to the prediction (in camera coordinates)
//    Eigen::Vector3f subtract_centroid_object_frame(object->offset_centroid_x, object->offset_centroid_y, object->offset_centroid_z);
//    Eigen::Vector3f subtract_centroid_camera_frame = cam_T_pred_pose.rotation() * subtract_centroid_object_frame;
//    cam_T_pred_pose.translation() = cam_T_pred_pose.translation().eval() - subtract_centroid_camera_frame;

    constructModelCloud(object,closest_q_index,cam_T_pred_pose, predicted_model_cloud); //"places" the object model at the predicted pose
    predicted_model_cloud->header.frame_id = camera_frame;
    //need to register the predicted model cloud to the data cloud
    score = gicp(predicted_model_cloud, data_cloud ,registered_cloud,  pred_pose_T_data.matrix()); //and registers it to the data



    //the final transformation is the predicted pose times the final alignment
    cam_T_object = pred_pose_T_data * cam_T_pred_pose;
    publishRegisteredView(object, cam_T_object, camera_frame);
    publishGrasps(object);

    std::cout <<"publishing the registered model"<<std::endl;
    registered_cloud->header.frame_id = camera_frame;
    registered_cloud->header.stamp = 0;
    registered_cloud_pub_.publish (registered_cloud);


    ROS_INFO_STREAM("ARCRegistration::register_model_cloud object->grasps.graspList.grasps.size()="<<object->grasps.graspList.grasps.size());
    //request the grasps pose in the camera frame of reference
    for(unsigned int i=0;i < object->grasps.graspList.grasps.size(); i++){
      std::string grasp_frame_id = "grasp_"+object->object_name+std::to_string(i);
      ROS_INFO_STREAM("requesting grasp "<<grasp_frame_id);
      Eigen::Affine3d graspEigenTransform;
      getGraspingPose(grasp_frame_id, camera_frame, graspEigenTransform);
      grasping_poses.push_back(graspEigenTransform);
    }


    return score;

  }
}

void ARCRegistration ::getGraspingPose(const std::string& grasp_frame_id, const std::string& camera_frame, Eigen::Affine3d& graspEigenTransform){
  try{

    tf::StampedTransform graspTf;
    ros::Time now = ros::Time::now();
    ros::spinOnce();
    listener.waitForTransform(camera_frame,grasp_frame_id,
                                   now, ros::Duration(3.0));

     listener.lookupTransform(camera_frame,grasp_frame_id,
                              ros::Time(0), graspTf);

     tf::transformTFToEigen(graspTf, graspEigenTransform);

   }
   catch (tf::TransformException &ex) {
     ROS_WARN_STREAM("ARCTurntableCalibration::getGraspingPose(): "<<ex.what());
   }
}



void ARCRegistration::subSample(PointCloudTPtr& src_cloud,
    PointCloudTPtr& dst_cloud) {
  PointCloudTPtr tmp_cloud(
      new PointCloudT);
//  pcl::UniformSampling<PointT> uni_sampling;
//  uni_sampling.setInputCloud(src_cloud);
//  //uni_sampling.setRadiusSearch(0.02f);
//  uni_sampling.setRadiusSearch(0.02f);
//  //PCL 1.7
//  //uni_sampling.filter(*tmp_cloud);
//  uni_sampling.detectKeypoints(*tmp_cloud);



  ROS_INFO_STREAM("before sub-sampling: "<<src_cloud->points.size()<< " points");
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (src_cloud);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*tmp_cloud);
  dst_cloud = tmp_cloud;
  ROS_INFO_STREAM("after sub-sampling: "<<dst_cloud->points.size()<< " points");


}
bool ARCRegistration ::registrationMRSMapServiceCallback(arc_registration::RegistrationRequest& request, arc_registration::RegistrationResponse& response) {


    ROS_INFO_STREAM("called registrationMRSMapServiceCallback on object: "<<request.objectName<<" at position "<<request.predictedPose.position.x<<" "<<request.predictedPose.position.y<<" "<<request.predictedPose.position.z);

    if ( objects_map.find(request.objectName) == objects_map.end() ) {
      // not found
      return false;
    } else {
      // found
      std::string camera_frame = request.cameraFrame;
      bool only_prediction = request.onlyPrediction;
      PointCloudTPtr predicted_model_cloud(new PointCloudT);
      PointCloudTPtr data_cloud(new PointCloudT);
      PointCloudTPtr registered_cloud(new PointCloudT);
      pcl::fromROSMsg(request.dataCloud, *data_cloud);
      data_cloud->header.frame_id = camera_frame;
      //publish the data cloud
      publishDataCloud(data_cloud);



      //get the predicted orientation and look for the closest one
      Eigen::Quaternionf q_predicted(request.predictedPose.orientation.w,request.predictedPose.orientation.x, request.predictedPose.orientation.y, request.predictedPose.orientation.z);

      int closest_q_index = getClosestFrame(objects_map[request.objectName], q_predicted);
      Eigen::Affine3f cam_T_object, cam_T_pred_pose;
      //set the predicted offset
      cam_T_pred_pose.matrix()(0,3) = request.predictedPose.position.x;
      cam_T_pred_pose.matrix()(1,3) = request.predictedPose.position.y;
      cam_T_pred_pose.matrix()(2,3) = request.predictedPose.position.z;

      //set the orientation
      Eigen::Matrix3f R(q_predicted);
      cam_T_pred_pose.matrix().block<3,3>(0,0) = R;



      //need to subtract the centroid offset to the prediction (in camera coordinates)
      ObjectPtr object = objects_map[request.objectName];
      Eigen::Vector3f subtract_centroid_object_frame(object->offset_centroid_x, object->offset_centroid_y, object->offset_centroid_z);
      Eigen::Vector3f subtract_centroid_camera_frame = cam_T_pred_pose.rotation() * subtract_centroid_object_frame;
      cam_T_pred_pose.translation() = cam_T_pred_pose.translation().eval() - subtract_centroid_camera_frame;

      //build the model cloud
      std::cout <<"constructing model cloud"<<std::endl;
      constructModelCloud(objects_map[request.objectName],closest_q_index,cam_T_pred_pose, predicted_model_cloud);
      predicted_model_cloud->header.frame_id = camera_frame;
      publishModelCloud(predicted_model_cloud);

      //need to register the predicted model cloud to the data cloud
      Eigen::Affine3f pred_pose_T_data = Eigen::Affine3f::Identity();
      if(!only_prediction){
        std::cout <<"calling gicp..."<<std::endl;
        //subsample the data cloud
        PointCloudTPtr subs_data_cloud(new PointCloudT),subs_pred_model_cloud(new PointCloudT);;
        subSample(data_cloud,subs_data_cloud);
        subSample(predicted_model_cloud,subs_pred_model_cloud);


        //gicp(predicted_model_cloud, subs_data_cloud ,registered_cloud,  pred_pose_T_data.matrix());
        //instead of gicp we construct a call for the magical mrsmaps
        rosmrsmap::ArcRegisterMap srv;
        pcl::toROSMsg(*predicted_model_cloud, srv.request.modelCloud);
        pcl::toROSMsg(*subs_data_cloud, srv.request.dataCloud);

        mrsmap_reg_client.call(srv);
        //get the result back from the response
        Eigen::Affine3d pred_pose_T_data_d;
        tf::transformMsgToEigen (srv.response.transform,pred_pose_T_data_d);
        pred_pose_T_data = pred_pose_T_data_d.cast<float>();

        std::cout <<"ARCRegistration ::registrationMRSMapServiceCallback pred_pose_T_data="<<pred_pose_T_data.matrix()<<std::endl;

      }

      pcl::transformPointCloud (*predicted_model_cloud, *registered_cloud, pred_pose_T_data );
      //pred_pose_T_data.matrix() = pred_pose_T_data.matrix().inverse().eval();
      std::cout <<"publishing the registered model"<<std::endl;
      registered_cloud->header.frame_id = camera_frame;
      registered_cloud->header.stamp = 0;
      registered_cloud_pub_.publish (registered_cloud);

      //the final transformation is the predicted pose times the final alignment
      cam_T_object = pred_pose_T_data * cam_T_pred_pose;
      publishRegisteredView(objects_map[request.objectName], cam_T_object, camera_frame);
      publishGrasps(objects_map[request.objectName]);
      //request the grasps pose in the camera frame of reference
      for(unsigned int i=0;i < object->grasps.graspList.grasps.size(); i++){
        std::string grasp_frame_id = "grasp_"+object->object_name+std::to_string(i);
        ROS_INFO_STREAM("requesting grasp "<<grasp_frame_id);
        Eigen::Affine3d graspEigenTransform = Eigen::Affine3d::Identity();
        getGraspingPose(grasp_frame_id, camera_frame, graspEigenTransform);
        Eigen::Quaterniond q(graspEigenTransform.rotation());
        ROS_INFO_STREAM("requested grasping pose quaternion: "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w());
        i++;
      }



      //publishRegisteredView(objects_map[request.objectName],closest_q_index, predicted_object_pose, camera_frame);
      return true;

    }
}


/*
 * the request for registration comes with
 *  -string objectName: the name of the object
 *  -string cameraFrame: the frame of reference of the camera
 *  -geometry_msgs/Pose predictedPose: the pose of the object predicted by the network
 *  -sensor_msgs/PointCloud2 dataCloud: the point cloud of the object to which we will register the model, initialised at the predicted pose
 *
 *
 */
bool ARCRegistration ::registrationServiceCallback(arc_registration::RegistrationRequest& request, arc_registration::RegistrationResponse& response) {
    ROS_INFO_STREAM("called registrationServiceCallback on object: "<<request.objectName<<" at position "<<request.predictedPose.position.x<<" "<<request.predictedPose.position.y<<" "<<request.predictedPose.position.z);

    if ( objects_map.find(request.objectName) == objects_map.end() ) {
      // not found
      return false;
    } else {
      // found
      std::string camera_frame = request.cameraFrame;
      bool only_prediction = request.onlyPrediction;
      PointCloudTPtr predicted_model_cloud(new PointCloudT);
      PointCloudTPtr data_cloud(new PointCloudT);
      PointCloudTPtr registered_cloud(new PointCloudT);
      pcl::fromROSMsg(request.dataCloud, *data_cloud);
      data_cloud->header.frame_id = camera_frame;
      //publish the data cloud
      publishDataCloud(data_cloud);



      //get the predicted orientation and look for the closest one
      Eigen::Quaternionf q_predicted(request.predictedPose.orientation.w,request.predictedPose.orientation.x, request.predictedPose.orientation.y, request.predictedPose.orientation.z);

      int closest_q_index = getClosestFrame(objects_map[request.objectName], q_predicted);
      Eigen::Affine3f cam_T_object, cam_T_pred_pose;
      //set the predicted offset
      cam_T_pred_pose.matrix()(0,3) = request.predictedPose.position.x;
      cam_T_pred_pose.matrix()(1,3) = request.predictedPose.position.y;
      cam_T_pred_pose.matrix()(2,3) = request.predictedPose.position.z;

      //set the orientation
      Eigen::Matrix3f R(q_predicted);
      cam_T_pred_pose.matrix().block<3,3>(0,0) = R;



      //need to subtract the centroid offset to the prediction (in camera coordinates)
      ObjectPtr object = objects_map[request.objectName];
      Eigen::Vector3f subtract_centroid_object_frame(object->offset_centroid_x, object->offset_centroid_y, object->offset_centroid_z);
      Eigen::Vector3f subtract_centroid_camera_frame = cam_T_pred_pose.rotation() * subtract_centroid_object_frame;
      cam_T_pred_pose.translation() = cam_T_pred_pose.translation().eval() - subtract_centroid_camera_frame;

      std::cout <<"constructing model cloud"<<std::endl;
      constructModelCloud(objects_map[request.objectName],closest_q_index,cam_T_pred_pose, predicted_model_cloud);
      predicted_model_cloud->header.frame_id = camera_frame;
      publishModelCloud(predicted_model_cloud);

      //need to register the predicted model cloud to the data cloud
      Eigen::Affine3f pred_pose_T_data = Eigen::Affine3f::Identity();
      if(!only_prediction){
        std::cout <<"calling gicp..."<<std::endl;
        //subsample the data cloud
        PointCloudTPtr subs_data_cloud(new PointCloudT),subs_pred_model_cloud(new PointCloudT);;
        subSample(data_cloud,subs_data_cloud);
        subSample(predicted_model_cloud,subs_pred_model_cloud);


        gicp(predicted_model_cloud, subs_data_cloud ,registered_cloud,  pred_pose_T_data.matrix());
      }

      pcl::transformPointCloud (*predicted_model_cloud, *registered_cloud, pred_pose_T_data );
      //pred_pose_T_data.matrix() = pred_pose_T_data.matrix().inverse().eval();
      std::cout <<"publishing the registered model"<<std::endl;
      registered_cloud->header.frame_id = camera_frame;
      registered_cloud->header.stamp = 0;
      registered_cloud_pub_.publish (registered_cloud);

      //the final transformation is the predicted pose times the final alignment
      cam_T_object = pred_pose_T_data * cam_T_pred_pose;
      publishRegisteredView(objects_map[request.objectName], cam_T_object, camera_frame);
      publishGrasps(objects_map[request.objectName]);
      //request the grasps pose in the camera frame of reference
      for(unsigned int i=0;i < object->grasps.graspList.grasps.size(); i++){
        std::string grasp_frame_id = "grasp_"+object->object_name+std::to_string(i);
        ROS_INFO_STREAM("requesting grasp "<<grasp_frame_id);
        Eigen::Affine3d graspEigenTransform = Eigen::Affine3d::Identity();
        getGraspingPose(grasp_frame_id, camera_frame, graspEigenTransform);
        Eigen::Quaterniond q(graspEigenTransform.rotation());
        ROS_INFO_STREAM("requested grasping pose quaternion: "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w());
        i++;
      }



      //publishRegisteredView(objects_map[request.objectName],closest_q_index, predicted_object_pose, camera_frame);
      return true;
    }



}



int main(int argc, char** argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "arcRegistrationNode"); //node name

    ros::NodeHandle nh("~"); // create a node handle; need to pass this to the class constructor
    ROS_INFO_STREAM("namespace of node running on main:"<<nh.getNamespace());

    ARCRegistration registration_node(&nh);
    //ARCRegistration registration_lib("/home/martin/Datasets/ARC/20_7/"); //to ru

    ros::spin();
    return 0;
}

