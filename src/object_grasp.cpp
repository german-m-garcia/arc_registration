

#include <arc_registration/object_grasp.h>
#include <yaml-cpp/yaml.h>

namespace object_grasp_editor{


  ObjectGrasp::ObjectGrasp(): basicShapeScaleX(0.), basicShapeScaleY(0.), basicShapeScaleZ(0.)
	{
		graspList.header.frame_id = "";
		graspList.grasps.clear();
		axis.pose.position.x =axis.pose.position.y = axis.pose.position.z = 0;
		axis.header.frame_id ="";    
	}
	ObjectGrasp::~ObjectGrasp(){ }

	const double ObjectGrasp::getRoll(int index) const
	{
    tf::Quaternion qt = tf::Quaternion( graspList.grasps[index].grasp.pose.orientation.x, graspList.grasps[index].grasp.pose.orientation.y,
      graspList.grasps[index].grasp.pose.orientation.z, graspList.grasps[index].grasp.pose.orientation.w);
		tf::Matrix3x3 mt;
		mt.setRotation(qt);
		double roll, yaw, pitch;
		mt.getRPY(roll, yaw, pitch);
		return roll;
	}
	const double ObjectGrasp::getYaw(int index) const
	{
    tf::Quaternion qt = tf::Quaternion( graspList.grasps[index].grasp.pose.orientation.x, graspList.grasps[index].grasp.pose.orientation.y,
      graspList.grasps[index].grasp.pose.orientation.z, graspList.grasps[index].grasp.pose.orientation.w);
		tf::Matrix3x3 mt;
		mt.setRotation(qt);
		double roll, yaw, pitch;
		mt.getRPY(roll, yaw, pitch);
		return yaw;
	}
	const double  ObjectGrasp::getPitch(int index) const
	{
    tf::Quaternion qt = tf::Quaternion( graspList.grasps[index].grasp.pose.orientation.x, graspList.grasps[index].grasp.pose.orientation.y,
      graspList.grasps[index].grasp.pose.orientation.z, graspList.grasps[index].grasp.pose.orientation.w);
		tf::Matrix3x3 mt;
		mt.setRotation(qt);
		double roll, yaw, pitch;
		mt.getRPY(roll, yaw, pitch);
		return pitch;
	}

  const geometry_msgs::PoseStamped ObjectGrasp::getPose(int index) const
	{
    return graspList.grasps[index].grasp;
	}

  const std::string ObjectGrasp::getGraspType(int index) const
  {
          return graspList.grasps[index].type;
  }

	ObjectGrasp ObjectGrasp::loadFromYAML(std::istream& file)
	{
		YAML::Node doc;
		try
		{ 
      doc = YAML::Load(file);
			ObjectGrasp grasp = ObjectGrasp();
      grasp.name = doc["name"].as<std::string>();
      grasp.basicShapeScaleX = doc["basicShapeScaleX"].as<float>();
      grasp.basicShapeScaleY = doc["basicShapeScaleY"].as<float>();
      grasp.basicShapeScaleZ = doc["basicShapeScaleZ"].as<float>();
      grasp.basicShapeType = doc["basicShapeType"].as<uint32_t>();
      doc["basicShapePose"] >> grasp.basicShapePose;
      doc["graspList"] >> grasp.graspList;            
			return grasp;
		}
		catch(YAML::Exception& e)
		{
			ROS_WARN("Could not parse YAML: %s", e.what());
			return ObjectGrasp();
		}
		return ObjectGrasp();
	}

	std::string  ObjectGrasp::serializeToYAML()
	{
		YAML::Emitter out;

		out << YAML::BeginMap;

		out << YAML::Key << "name";
		out << YAML::Value << name;

    out << YAML::Key << "basicShapePose";
    out << YAML::Value << basicShapePose;

    out << YAML::Key << "basicShapeScaleX";
    out << YAML::Value << basicShapeScaleX;


    out << YAML::Key << "basicShapeScaleY";
    out << YAML::Value << basicShapeScaleY;


    out << YAML::Key << "basicShapeScaleZ";
    out << YAML::Value << basicShapeScaleZ;

    out << YAML::Key << "basicShapeType";
    out << YAML::Value << basicShapeType;


    out << YAML::Key << "graspList";
    out << YAML::Value << graspList;


		
		out << YAML::EndMap;

		return out.c_str();
	}

  void  ObjectGrasp::setPose(int index, geometry_msgs::PoseStamped pose)
	{
		graspList.grasps[index].grasp = pose;
	}

	void  ObjectGrasp::setPosX( int index, double x )
	{
    graspList.grasps[index].grasp.pose.position.x = x;
	}

	void  ObjectGrasp::setPosY( int index, double y )
	{
    graspList.grasps[index].grasp.pose.position.y = y;
	}

	void  ObjectGrasp::setPosZ( int index, double z )
	{
    graspList.grasps[index].grasp.pose.position.z = z;
	}

	void  ObjectGrasp::setRoll( int index, double roll )
	{
    tf::Quaternion qt = tf::Quaternion( graspList.grasps[index].grasp.pose.orientation.x, graspList.grasps[index].grasp.pose.orientation.y,
        graspList.grasps[index].grasp.pose.orientation.z, graspList.grasps[index].grasp.pose.orientation.w);
		tf::Matrix3x3 mt;
		mt.setRotation(qt);
		double r, yaw, pitch;
		mt.getRPY(r, yaw, pitch);
		qt.setRPY(roll, yaw, pitch);
    graspList.grasps[index].grasp.pose.orientation.x = qt.getX();
    graspList.grasps[index].grasp.pose.orientation.y = qt.getY();
    graspList.grasps[index].grasp.pose.orientation.z = qt.getZ();
    graspList.grasps[index].grasp.pose.orientation.w = qt.getW();
	}

	void  ObjectGrasp::setYaw( int index, double yaw )
	{
    tf::Quaternion qt = tf::Quaternion( graspList.grasps[index].grasp.pose.orientation.x, graspList.grasps[index].grasp.pose.orientation.y,
            graspList.grasps[index].grasp.pose.orientation.z, graspList.grasps[index].grasp.pose.orientation.w);
		tf::Matrix3x3 mt;
		mt.setRotation(qt);
		double roll, y, pitch;
		mt.getRPY(roll, y, pitch);
		qt.setRPY(roll, yaw, pitch);
    graspList.grasps[index].grasp.pose.orientation.x = qt.getX();
    graspList.grasps[index].grasp.pose.orientation.y = qt.getY();
    graspList.grasps[index].grasp.pose.orientation.z = qt.getZ();
    graspList.grasps[index].grasp.pose.orientation.w = qt.getW();
	}
	void  ObjectGrasp::setPitch( int index, double pitch )
	{
    tf::Quaternion qt = tf::Quaternion( graspList.grasps[index].grasp.pose.orientation.x, graspList.grasps[index].grasp.pose.orientation.y,
            graspList.grasps[index].grasp.pose.orientation.z, graspList.grasps[index].grasp.pose.orientation.w);
		tf::Matrix3x3 mt;
		mt.setRotation(qt);
		double roll, yaw, p;
		mt.getRPY(roll, yaw, p);
		qt.setRPY(roll, yaw, pitch);
    graspList.grasps[index].grasp.pose.orientation.x = qt.getX();
    graspList.grasps[index].grasp.pose.orientation.y = qt.getY();
    graspList.grasps[index].grasp.pose.orientation.z = qt.getZ();
    graspList.grasps[index].grasp.pose.orientation.w = qt.getW();
	}
	void ObjectGrasp::setGraspType(int index, GraspType type )
	{
		graspList.grasps[index].type = type;
	}
}


YAML::Emitter& operator<<(YAML::Emitter& out, const object_grasp_editor::GraspList& p)
{
	out << YAML::BeginMap;
	out << YAML::Key << "frame_id";
	
	out << YAML::Value << p.header.frame_id;
	out << YAML::Key << "grasps";
	out << YAML::Value;
	out << YAML::BeginSeq;
	for(size_t i=0; i<p.grasps.size(); i++)
	{
		out << YAML::BeginMap;
    out << YAML::Key << "grasp";
    out << YAML::Value << p.grasps[i].grasp;
		out << YAML::Key << "type";
    out << YAML::Value<< p.grasps[i].type;
		out << YAML::EndMap;
	}
	out << YAML::EndSeq;
	out << YAML::EndMap;
	return out;
}

void operator>>(const YAML::Node& in, object_grasp_editor::GraspList& p)
{
  p.header.frame_id = in["frame_id"].as<std::string>() ;
	const YAML::Node& grasps =in["grasps"];
	for(size_t i = 0; i < grasps.size(); ++i)
	{
		const YAML::Node& g =grasps[i];
		object_grasp_editor::Grasp grasp;
    std::string t;
    g["grasp"] >> grasp.grasp;
    grasp.type = g["type"].as<std::string>() ;
		p.grasps.push_back(grasp);
	}
}


YAML::Emitter& operator<<(YAML::Emitter& out, const geometry_msgs::Pose& p)
{	       
	out << YAML::BeginMap;
	out << YAML::Key << "position";
	out << YAML::Value<< YAML::Flow;
	out << YAML::BeginMap;
	out << YAML::Key << "x";
	out << YAML::Value << p.position.x;
	out << YAML::Key << "y";
	out << YAML::Value << p.position.y;
	out << YAML::Key << "z";
	out << YAML::Value << p.position.z;
	out << YAML::EndMap;
	
	out << YAML::Key << "orientation";
	out << YAML::Value << YAML::Flow;
	out << YAML::BeginMap;
	out << YAML::Key << "x";
	out << YAML::Value << p.orientation.x;
	out << YAML::Key << "y";
	out << YAML::Value << p.orientation.y;
	out << YAML::Key << "z";
	out << YAML::Value << p.orientation.z;
	out << YAML::Key << "w";
	out << YAML::Value << p.orientation.w;
	
	out << YAML::EndMap;
	
	out << YAML::EndMap;
	return out;
}

void operator>>(const YAML::Node& in, geometry_msgs::Pose& p)
{
	double x, y, z, w;
	const YAML::Node& pos = in["position"];
  x = pos["x"].as<float>();
  y = pos["y"].as<float>();
  z = pos["z"].as<float>();
	p.position.x = x;
	p.position.y = y;
	p.position.z = z;
	const YAML::Node& orien = in["orientation"];	
  x = orien["x"].as<float>();
  y = orien["y"].as<float>();
  z = orien["z"].as<float>();
  w = orien["w"].as<float>();
	p.orientation.x = x;
	p.orientation.y = y;
	p.orientation.z = z;
	p.orientation.w = w;

}
		
YAML::Emitter& operator<<(YAML::Emitter& out, const geometry_msgs::PoseStamped& p)
{
		out << YAML::BeginMap;
		out << YAML::Key << "frame_id";
		out << YAML::Value << p.header.frame_id;
		out << YAML::Key << "pose";
		out << YAML::Value << p.pose;
		out << YAML::EndMap;

		return out;
}
			
void operator>>(const YAML::Node& in, geometry_msgs::PoseStamped& p)
{
    p.header.frame_id = in["frame_id"].as<std::string>();
    in["pose"] >> p.pose;
}
			
