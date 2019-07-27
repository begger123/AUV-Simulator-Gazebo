#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
class ModelImport : public WorldPlugin
{
	public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*sdf*/)
	{
		_parent->InsertModelFile("model:://auv");
	}
};
	GZ_REGISTER_WORLD_PLUGIN(ModelImport)
}