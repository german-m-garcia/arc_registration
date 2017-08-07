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




int readFromFile (std::string path, std::vector <std::string> & mv) // muss vector vorher resized werden? wenn ja lese zeilenanzahl
{
    fstream file;
    std::string line;
    file.open(path);

    while (getline(file,line)) // lese zeile für zeile
    {
        mv.push_back(line); //fülle vector von hinten last in first
    }
    file.close();
    return 0;
}

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

/*
 * transforms the point cloud of view 000i/ to the frame of reference of view 0000/
 *
 */
void normalizeCloud(PointCloudTPtr srcCloud, PointCloudTPtr dstCloud, Eigen::Affine3f& transformation){
  //Eigen::Affine3f translation(Eigen::Affine3f::Identity());
  //translation.translation() = transformation.translation();
  //transform the point cloud
  pcl::transformPointCloud (*srcCloud, *dstCloud, transformation.matrix());
}


void mergeClouds(std::string& view_dir_path, std::string& output_path)
{

  PointCloudTPtr merged_cloud (new PointCloudT);
	
	std::cout <<"# of iterations?"<<std::endl;
	int inp = 2,use_gicp = 0, use_cgicp = 0;
  std::cin >>inp;
	std::cout <<"shall we use GICP? [y=1/n=0]"<<std::endl;
	std::cin >>use_gicp;
	bool use_gicp_b = use_gicp == 1 ? true : false;
	if(!use_gicp_b){
		std::cout <<"shall we use CGICP? [y=1/n=0]"<<std::endl;
		std::cin >>use_cgicp;
  }

  bool use_cgicp_b = use_cgicp == 1 ? true : false;
	
  PointCloudTPtr previous_cloud(new PointCloudT);
  Eigen::Affine3f view0_T_viewj;
	
  int start = 0;
  double angle;
  for(int i = start; i < inp; ++i)
	{
		
		std::cout<<"i "<<i<<std::endl;
    PointCloudTPtr input_cloud (new PointCloudT);
    PointCloudTPtr current_cloud (new PointCloudT);
    std::stringstream folderName;
    folderName << std::setfill('0') << std::setw(4) << i;
    std::string path1(folderName.str());
		
    std::string path_extracted_cloud(view_dir_path + "/"+ path1+ "/extracted.pcd");  //should contain the point cloud of the original frame, with the background masked out
    std::string normalized_cloud_path(view_dir_path + "/"+ path1+ "/normalized_cloud.pcd"); //the point cloud of the object, normalized to the frame of reference of this view: i/0000
    std::string normalized_cloud_to0_path(view_dir_path + "/"+ path1+ "/normalized_cloud_to_0.pcd"); //the point cloud of the object, normalized to the frame of reference of view 0/0000
		
    std::cout << "path  ="<<path_extracted_cloud<<std::endl;
    if (pcl::io::loadPCDFile<PointT> (path_extracted_cloud, *input_cloud) == -1)
		{
			PCL_ERROR ("Couldn't read pcd file\n");
		}
		
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (input_cloud);
    pass.filter (*current_cloud);



    //if we are looking at the initial view : 0/0000 or 1/0000
    if (i == start)
		{
      PointT min, max;
      pcl::getMinMax3D(*current_cloud, min, max);
			ROS_INFO_STREAM(" currentCloud "<< min << max);
      *merged_cloud += *current_cloud;
      //makes the points of the first view point cloud a bit reddish
      for(PointT& p : merged_cloud->points){
        p.r = 255;
      }
			
      view0_T_viewj = Eigen::Affine3f::Identity();

      pcl::io::savePCDFile( normalized_cloud_path, *current_cloud,true);
      pcl::io::savePCDFile( normalized_cloud_to0_path, *current_cloud,true);

      std::string rotation_file(view_dir_path+"/" + path1 + "/rotation.tf");
      std::string angle_file(view_dir_path+"/" + path1 + "/turntable_angle.txt");
      //read the yaw angle at the first frame of the view
      std::ifstream angle_stream(angle_file);

      angle_stream >> angle;
      std::cout <<"read angle="<<angle<<std::endl;


      std::cout <<"saving the rotation component "<<view0_T_viewj.rotation()<<" at "<<rotation_file<<std::endl;
      Eigen::Affine3f current_rotation = view0_T_viewj.inverse();
      Eigen::write_binary(rotation_file.c_str(), current_rotation.rotation()/*tfPrevious2init.rotation()*/);
      Eigen::Quaternionf q( view0_T_viewj.rotation());
      std::cout <<"equivalent quaternion ="<<q.x()<<" "<< q.y()<<" "<< q.z()<<" "<< q.w()<<std::endl;


//      pcl::visualization::PCLVisualizer viewer1 ("view");
//      pcl::visualization::PointCloudColorHandlerRGBField<pointRGB> rgb_cch (currentCloud);
//      viewer1.addPointCloud (currentCloud, rgb_cch, "view");
//      viewer1.setBackgroundColor(0.05, 0.05, 0.05, 0);
//      viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "view");

//      viewer1.addCoordinateSystem(0.2);

//      while (!viewer1.wasStopped ()) {
//        viewer1.spinOnce ();
//      }
//      viewer1.close();

		}
		else
		{
      PointCloudTPtr transformCloud (new PointCloudT);
      PointCloudTPtr normalizedCloud (new PointCloudT);
      Eigen::Affine3f yawTf;      

      std::string angle_file(view_dir_path+"/" + path1 + "/turntable_angle.txt");
      std::string rotation_file(view_dir_path+"/" + path1 + "/rotation.tf");

      double current_angle, angle_interval;
      std::ifstream angle_stream(angle_file);
      angle_stream >> current_angle;
      std::cout <<"--------------------"<<std::endl;
      std::cout <<"read angle="<<current_angle<<std::endl;
      if(current_angle < 0.)
        current_angle += (2*M_PI);
      angle_interval = current_angle - angle;
      std::cout <<"fixed angle="<<current_angle<<std::endl;
      std::cout <<"angle interval="<<angle_interval<<std::endl;
      std::cout <<"--------------------"<<std::endl;
      angle = current_angle;



      boost::filesystem::path p (angle_file);
      if (exists(p)){
        //std::cout <<" read rotation matrix ="<<std::endl<< readRotMatrix.matrix()<<std::endl;
        //std::cout <<" read quaternion ="<<readQuat.x()<<" "<< readQuat.y()<<" "<< readQuat.z()<<" "<< readQuat.w()<<std::endl;
      }
      else{
        std::cout <<" no turntable angle file available "<<std::endl;
      }


      yawTf.translation() =  Eigen::Vector3f(0, 0, 0);			
      //case we have hardcoded 30 degrees
      //yawTf.linear() =  Eigen::AngleAxisf( (-1 * 30 * M_PI / 180) , Eigen::Vector3f::UnitZ()).matrix();

      //case we use the angle interval read from the txt file
      yawTf.linear() =  Eigen::AngleAxisf( ( -angle_interval) , Eigen::Vector3f::UnitZ()).matrix();
			

      Eigen::Affine3f previous_view_T_current_view = yawTf;
      std::cout <<"currentCloud size="<<current_cloud->points.size()<<"  previousCloud size="<<previous_cloud->points.size()<<std::endl;
      if(use_gicp_b){
        double res = gicp(current_cloud, previous_cloud, previous_view_T_current_view.matrix());
        std::cout <<"used gicp res " <<res<<std::endl;
      }
      else if(use_cgicp_b){
        double res = cgicp(current_cloud, previous_cloud, previous_view_T_current_view.matrix());
        std::cout <<"used cgicp res " <<res<<std::endl;
      }
      else{
        std::cout <<"using fk"<<std::endl;
      }

      std::cout<<previous_view_T_current_view.matrix()<<std::endl;


      view0_T_viewj.matrix() =  view0_T_viewj.matrix() * previous_view_T_current_view.matrix() ;
			
      //normalize the current point cloud to the frame of reference of this view 0000
      normalizeCloud(current_cloud, normalizedCloud, view0_T_viewj);
      std::cout <<"saving the normalized cloud to "<<normalized_cloud_path<<std::endl;
      //save the normalized_cloud.pcd file (the current point cloud in the view0 frame of reference)
      pcl::io::savePCDFile( normalized_cloud_path, *normalizedCloud,true);

      std::cout <<"saving the rotation component "<<view0_T_viewj.rotation()<<" at "<<rotation_file<<std::endl;
      //now we save the pose of the object frame (view0) in the current viewj
      Eigen::Affine3f viewj_T_view0 = view0_T_viewj.inverse();
      Eigen::write_binary(rotation_file.c_str(), viewj_T_view0.rotation());
      Eigen::Quaternionf q( view0_T_viewj.rotation());
      std::cout <<"equivalent quaternion ="<<q.x()<<" "<< q.y()<<" "<< q.z()<<" "<< q.w()<<std::endl;

      pcl::transformPointCloud (*current_cloud, *transformCloud, view0_T_viewj.matrix() );


			
			
      *merged_cloud += *transformCloud;

//      pcl::visualization::PCLVisualizer viewer1 ("view");
//      pcl::visualization::PointCloudColorHandlerRGBField<pointRGB> rgb_cch (normalizedCloud);
//      viewer1.addPointCloud (normalizedCloud, rgb_cch, "view");
//      viewer1.setBackgroundColor(0.05, 0.05, 0.05, 0);
//      viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "view");

//      viewer1.addCoordinateSystem(0.2);

//      while (!viewer1.wasStopped ()) {
//        viewer1.spinOnce ();
//      }
//      viewer1.close();
		}
		
		
    *previous_cloud = *current_cloud;
		
	}
  pcl::io::savePCDFile( output_path, *merged_cloud,true);
  std::cout<<" PCD file saved at "<<output_path<<std::endl;
	
	
	pcl::visualization::PCLVisualizer viewer ("TF");
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_cch (merged_cloud);
  viewer.addPointCloud (merged_cloud, rgb_cch, "TF");
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "TF");

	viewer.addCoordinateSystem(0.2);
	
	while (!viewer.wasStopped ()) {
		viewer.spinOnce ();
	}
	viewer.close();

}
int main(int argc, char ** argv)
{

  if(argc !=2){
    std::cout<<"usage: $>./register_views <path to object>"<<std::endl;
    return 0;
  }
  std::string path(argv[1]);
  std::string output = path + "/merged_cloud.pcd";
  mergeClouds(path, output);

	
	return 0;
}
