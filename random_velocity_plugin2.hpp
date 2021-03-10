/*
  My random_velocity_plugin based on gazebo's RandomVelocityPlugin.
  Added features: XYZ position bounding box
*/

#ifndef GAZEBO_PLUGINS_RANDOMVELOCITYPLUGIN2_HH_
#define GAZEBO_PLUGINS_RANDOMVELOCITYPLUGIN2_HH_

//#include <memory>

#include <sdf/sdf.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  // Forward declare private data class.
  class RandomVelocityPlugin2Private;

  class GAZEBO_VISIBLE RandomVelocityPlugin2 : public ModelPlugin
  {
    public: RandomVelocityPlugin2();
    public: ~RandomVelocityPlugin2();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Reset();
    private: void Update(const common::UpdateInfo &_info);

    //Private data pointer
    private: std::unique_ptr<RandomVelocityPlugin2Private> dataPtr;
  };
}
#endif
