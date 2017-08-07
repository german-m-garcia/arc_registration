#include <ros/console.h>
#include <ros/package.h>
#include <boost/filesystem/convenience.hpp>

#include <vector>

#include <boost/filesystem.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/gicp6d.h>
//#include <pcl/point_types_conversion.h>


#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>



#include "arc_registration/utils.hpp"

using namespace boost::filesystem;

constexpr int DISCRETIZATION = 12;


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;



double gicp(PointCloudT::Ptr& cloud_1,
                     PointCloudT::Ptr& cloud_2,
                    Eigen::Matrix4f& transform){


    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;


    gicp.setInputSource(cloud_1);
    gicp.setInputTarget(cloud_2);
    gicp.setMaximumIterations (100);
    gicp.setTransformationEpsilon (1e-8);
    gicp.setMaxCorrespondenceDistance (0.01);

    // run registration and get transformation
    PointCloudT::Ptr output(new PointCloudT);
    gicp.align(*output, transform);
    double score = gicp.getFitnessScore();
    cout <<" gicp score = "<<score<<endl;

    transform = gicp.getFinalTransformation();
    return score;
}

double cgicp(
	PointCloudT::Ptr& cloud_1,
	PointCloudT::Ptr& cloud_2,
	Eigen::Matrix4f& transform)
{
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBA>);
	PointCloudT::Ptr tmp(new PointCloudT);
	std::vector<int> indices1, indices2;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	pcl::removeNaNFromPointCloud(*cloud_1, *tmp, indices1);
	pcl::removeNaNFromPointCloud(*cloud_2, *tmp, indices2);
	copyPointCloud(*cloud_1,indices1,  *cloud1_rgba);
	copyPointCloud(*cloud_2,indices2,  *cloud2_rgba);
	
// 	transform = Eigen::Matrix<float, 4, 4>::Identity();
	
	std::cout <<"cloud1_rgba points="<<cloud1_rgba->points.size()<<" cloud2_rgba points="<<cloud2_rgba->points.size()<<std::endl;
  pcl::GeneralizedIterativeClosestPoint6D reg(0.01); //by default alpha = 0.032
	reg.setInputSource (cloud1_rgba);
	reg.setInputTarget (cloud2_rgba);
	reg.setSearchMethodTarget (tree);
  reg.setMaximumIterations (100);
	reg.setTransformationEpsilon (1e-8);
  reg.setMaxCorrespondenceDistance (0.01);
	
	// Register
	reg.align (*output, transform.matrix());
	double score = reg.getFitnessScore();
	std::cout << "colour icp has converged:" << reg.hasConverged() << " score: " << score
	<< std::endl;
	transform = reg.getFinalTransformation();
	
	return score;
	
}

void normalizeCloud(PointCloudTPtr srcCloud, PointCloudTPtr dstCloud, Eigen::Affine3f& transformation){
  //Eigen::Affine3f translation(Eigen::Affine3f::Identity());
  //translation.translation() = transformation.translation();
  //transform the point cloud
  pcl::transformPointCloud (*srcCloud, *dstCloud, transformation.matrix() );
}


void alignViews(std::string& m_viewDir)
{

  Eigen::Affine3f align_views_to_0_affine;
  Eigen::Affine3d cam_T_object_d;
  Eigen::Affine3f cam_T_object;
  //need to read the file 1/align_views.tf which contains the transformation from 1/ to 0/
  std::string align_file(m_viewDir+"/transformation.tf"); //file obtained from Arul's alignment tool, contains the pose of view 1/ wrt to view 0/
  std::string camera_pose_path(m_viewDir+"/cam_T_obj.tf"); //the pose of the object in the camera frame of reference
  std::string merged_cloud_path(m_viewDir + "/merged_cloud.pcd");
  std::string merged_cloud_to_0_path(m_viewDir + "/merged_cloud_to_0.pcd");

  std::cout <<"opening "<<align_file<<std::endl;
  //check whether the file exists
  path root_path (align_file);
  if (!exists(root_path)){
    ROS_ERROR_STREAM("The file "<<align_file <<" does not exist");
    return;
  }


  Eigen::read_binary(camera_pose_path.c_str(), cam_T_object_d.matrix());
  cam_T_object = cam_T_object_d.cast<float>();
  Eigen::Quaternionf tmp_q(cam_T_object.rotation());
  std::cout <<"cam_T_object= "<<tmp_q.vec()<<" " <<tmp_q.w()<<std::endl;
  Eigen::read_binary(align_file.c_str(), align_views_to_0_affine.matrix());  
  std::cout <<" read alignment tf matrix="<<std::endl<< align_views_to_0_affine.matrix() << std::endl;
  //we need to transform points from view 1 to view 0 (i.e. need the pose of view 0 in view 1 frame of reference)
  align_views_to_0_affine.matrix() = align_views_to_0_affine.matrix().inverse().eval();



  Eigen::Quaternionf runj_T_run0_q( align_views_to_0_affine.rotation());
  //std::cout <<"align_views_affine ="<<align_views_to_0_affine.matrix()<<std::endl;
  Eigen::Vector3f rpy = runj_T_run0_q.toRotationMatrix().eulerAngles(0, 1, 2); //roll, pitch, yaw
  std::cout <<"rpy ="<<rpy<<std::endl;

  //transform the merged_cloud file as well
  PointCloudTPtr merged_cloud (new PointCloudT);
  PointCloudTPtr merged_cloud_normalized (new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (merged_cloud_path, *merged_cloud) == -1)
  {
    ROS_ERROR_STREAM ("Couldn't read "<<merged_cloud_path);
  }
  //rotate the merged cloud (generated by the register_views node)
  normalizeCloud(merged_cloud, merged_cloud_normalized, align_views_to_0_affine);
  //save it
  std::cout <<"saving the normalized cloud to "<<merged_cloud_to_0_path<<std::endl;
  pcl::io::savePCDFile( merged_cloud_to_0_path, *merged_cloud_normalized,true);


	
  int start = 0;//inp == 0 ? 0 : inp -2 ;
  for(int i = start; i < 20; ++i)
	{
		
    Eigen::Affine3f cam_T_view; // cam_T_view =  cam_T_object object_T_view
    std::cout<<"i "<<i<<std::endl;
    PointCloudTPtr inCloud (new PointCloudT);
    PointCloudTPtr filteredCloud (new PointCloudT);
    PointCloudTPtr currentCloud (new PointCloudT);
    PointCloudTPtr normalizedCloud (new PointCloudT);
		std::stringstream pcdName;
		pcdName << std::setfill('0') << std::setw(4) << i;
		std::string path1(pcdName.str());
		
    std::string path(                   m_viewDir + "/"+ path1+ "/extracted.pcd");
    std::string rotation_path(          m_viewDir + "/"+ path1+ "/rotation.tf"); //pose of frame 0000 in the 000i frame of reference
    std::string rotation_to_0_path(     m_viewDir + "/"+ path1+ "/rotation_to_0.tf");
    std::string camera_pose_file(       m_viewDir + "/"+ path1+ "/cam_T_view.tf"); //pose of this view in the camera frame of reference


    std::string normalizedCloudPath(    m_viewDir + "/"+ path1+ "/normalized_cloud.pcd");
    std::string normalizedCloudTo0Path( m_viewDir + "/"+ path1+ "/normalized_cloud_to_0.pcd");

		
    std::cout << "path  ="<<normalizedCloudPath<<std::endl;
    if (pcl::io::loadPCDFile<PointT> (normalizedCloudPath, *inCloud) == -1)
		{
			PCL_ERROR ("Couldn't read pcd file\n");
		}
		
    pcl::PassThrough<PointT> pass;
		pass.setInputCloud (inCloud);
    pass.filter (*currentCloud);

    //currentCloud is the normalized.pcd cloud, which is already normalized to the 0000 frame of the current view
    //just need to transform it to the 0/ view
    normalizeCloud(currentCloud, normalizedCloud, align_views_to_0_affine);



    Eigen::Matrix3f read_relative_rot_matrix_to_0000;
    Eigen::read_binary(rotation_path.c_str(), read_relative_rot_matrix_to_0000);
    Eigen::Quaternionf object_T_view_q, viewj_T_view0_q(read_relative_rot_matrix_to_0000);

    boost::filesystem::path p (rotation_path);
    if (exists(p)){

      std::cout <<" read quaternion ="<<viewj_T_view0_q.x()<<" "<< viewj_T_view0_q.y()<<" "<< viewj_T_view0_q.z()<<" "<< viewj_T_view0_q.w()<<std::endl;
      //need to transform first to the current view 0000 frame, then to the original view 0/0000
      object_T_view_q = viewj_T_view0_q * runj_T_run0_q.inverse();
      std::cout <<"updated quaternion to rotate to frame 0/ is "<<object_T_view_q.x()<<" "<<object_T_view_q.y()<<" "<<object_T_view_q.z()<<" "<<object_T_view_q.w()<<std::endl;
      //now save it
      Eigen::write_binary(rotation_to_0_path.c_str(), object_T_view_q.toRotationMatrix());

      std::cout <<"saving the normalized cloud to "<<normalizedCloudTo0Path<<std::endl;
      pcl::io::savePCDFile( normalizedCloudTo0Path, *normalizedCloud,true);

      //now create the camera pose file
      //Eigen::Affine3f cam_T_view; // cam_T_view =  cam_T_object object_T_view
      Eigen::Quaternionf cam_T_object_q, cam_T_turntable_q(cam_T_object.rotation());
      cam_T_object_q = cam_T_turntable_q * object_T_view_q;

      std::cout <<"cam_T_view_q = cam_T_object_q * object_T_view_q :"<<cam_T_object_q.vec()<<" " <<cam_T_object_q.w()<<" = "<<cam_T_turntable_q.vec()<<" " <<cam_T_turntable_q.w()
               <<" * "<<object_T_view_q.vec()<<" " <<object_T_view_q.w() <<std::endl;
      Eigen::write_binary(camera_pose_file.c_str(), cam_T_object_q.toRotationMatrix());


    }
    else{
      std::cout <<" no rotation matrix file available "<<std::endl;
    }


//    pcl::visualization::PCLVisualizer viewer ("TF");
//    pcl::visualization::PointCloudColorHandlerRGBField<pointRGB> rgb_cch (normalizedCloud);
//    viewer.addPointCloud (normalizedCloud, rgb_cch, "TF");
//    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "TF");

//    viewer.addCoordinateSystem(0.2);

//    while (!viewer.wasStopped ()) {
//      viewer.spinOnce ();
//    }
//    viewer.close();

		

		
  }
	


}
int main(int argc, char ** argv)
{

  if(argc ==2){
    //std::cout<<"usage: $>./align_tfs_views <path to object view #1, e.g. .../epsom_salts/0 > <path to object view #2, e.g. .../epsom_salts/1 >"<<std::endl;
    std::string path(argv[1]);
    alignViews(path);
    return 0;
  }

  ros::init(argc, argv, "align_tfs_views");
  ros::NodeHandle nh("~");
  ros::spin();
	
	return 0;
}
