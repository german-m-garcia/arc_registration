
#ifndef OBJECT_GRASP_H_
#define OBJECT_GRASP_H_

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <yaml-cpp/emitter.h>
#include <yaml-cpp/parser.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <ros/console.h>
#include <roscpp/GetLoggers.h>


namespace object_grasp_editor{

enum GraspType{
  TOP_GRASP,
  SIDE_GRASP,
  OTHER_GRASP
};
struct Grasp{
  geometry_msgs::PoseStamped grasp;
  std::string type;

};
struct GraspList{
	std_msgs::Header header;
	std::vector<Grasp> grasps;
};
class ObjectGrasp{

public:
	typedef boost::shared_ptr<ObjectGrasp> Ptr;
	
  ObjectGrasp();
	~ObjectGrasp();

	std::string name;
  geometry_msgs::PoseStamped basicShapePose;
  float basicShapeScaleX, basicShapeScaleY, basicShapeScaleZ;
  uint32_t basicShapeType;

	GraspList graspList;
	geometry_msgs::PoseStamped axis;
	float weight;
  const geometry_msgs::PoseStamped getPose(int index) const;
  const std::string getGraspType(int index) const;
	const double getRoll( int index ) const;
	const double getYaw( int index ) const;
	const double getPitch( int index ) const;
	static ObjectGrasp loadFromYAML(std::istream& file);
	std::string serializeToYAML();
  void setPose(int index, geometry_msgs::PoseStamped pose);
	void setPosX( int index, double x );
	void setPosY( int index, double y );
	void setPosZ( int index, double z );
	void setRoll( int index, double roll );
	void setYaw( int index, double yaw );
	void setPitch( int index, double pitch );
	void setGraspType(int index, GraspType type );
	inline int getNumOfGrasps(){return graspList.grasps.size();}
};
}

YAML::Emitter& operator<<(YAML::Emitter& out, const object_grasp_editor::GraspList& p);

void operator>>(const YAML::Node& in, object_grasp_editor::GraspList& p);
YAML::Emitter& operator<<(YAML::Emitter& out, const geometry_msgs::Pose& p);
void operator>>(const YAML::Node& in, geometry_msgs::Pose& p);
YAML::Emitter& operator<<(YAML::Emitter& out, const geometry_msgs::PoseStamped& p);
void operator>>(const YAML::Node& in, geometry_msgs::PoseStamped& p);


#endif /* OBJECT_GRASP_H_ */
