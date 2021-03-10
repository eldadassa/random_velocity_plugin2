#include <ignition/math/Rand.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include "random_velocity_plugin2_private.hpp"
#include "random_velocity_plugin2.hpp"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(RandomVelocityPlugin2)

/////////////////////////////////////////////////
RandomVelocityPlugin2::RandomVelocityPlugin2()
  : dataPtr(new RandomVelocityPlugin2Private)
{
}

/////////////////////////////////////////////////
RandomVelocityPlugin2::~RandomVelocityPlugin2()
{
  //delete this->dataPtr;
}

/////////////////////////////////////////////////
void RandomVelocityPlugin2::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");

  // Make sure the link has been specified
  if (!_sdf->HasElement("link"))
  {
    gzerr << "<link> element missing from RandomVelocity2 plugin. "
      << "The plugin will not function.\n";
    return;
  }

  // Get the link;
  this->dataPtr->link = _model->GetLink(_sdf->Get<std::string>("link"));
  if (!this->dataPtr->link)
  {
    gzerr << "Unable to find link[" << _sdf->Get<std::string>("link") << "] "
      << "in model[" << _model->GetName() << "]. The RandomVelocity2 plugin "
      << "will not function.\n";
    return;
  }

  // Get x clamping values
  if (_sdf->HasElement("min_x"))
    this->dataPtr->xRange.X(_sdf->Get<double>("min_x"));
  if (_sdf->HasElement("max_x"))
    this->dataPtr->xRange.Y(_sdf->Get<double>("max_x"));

  // Make sure min <= max
  ignition::math::Vector2d tmp = this->dataPtr->xRange;
  this->dataPtr->xRange.X(std::min(tmp.X(), tmp.Y()));
  this->dataPtr->xRange.Y(std::max(tmp.X(), tmp.Y()));

  // Get y clamping values
  if (_sdf->HasElement("min_y"))
    this->dataPtr->yRange.X(_sdf->Get<double>("min_y"));
  if (_sdf->HasElement("max_y"))
    this->dataPtr->yRange.Y(_sdf->Get<double>("max_y"));

  // Make sure min <= max
  tmp = this->dataPtr->yRange;
  this->dataPtr->yRange.X(std::min(tmp.X(), tmp.Y()));
  this->dataPtr->yRange.Y(std::max(tmp.X(), tmp.Y()));

  // Get z clamping values
  if (_sdf->HasElement("min_z"))
    this->dataPtr->zRange.X(_sdf->Get<double>("min_z"));
  if (_sdf->HasElement("max_z"))
    this->dataPtr->zRange.Y(_sdf->Get<double>("max_z"));

  // Make sure min <= max
  tmp = this->dataPtr->zRange;
  this->dataPtr->zRange.X(std::min(tmp.X(), tmp.Y()));
  this->dataPtr->zRange.Y(std::max(tmp.X(), tmp.Y()));

  // Get x position clamping values
  if (_sdf->HasElement("min_x_pos"))
    this->dataPtr->xRangePos.X(_sdf->Get<double>("min_x_pos"));
  if (_sdf->HasElement("max_x_pos"))
    this->dataPtr->xRangePos.Y(_sdf->Get<double>("max_x_pos"));

  // Make sure min <= max
  tmp = this->dataPtr->xRangePos;
  this->dataPtr->xRangePos.X(std::min(tmp.X(), tmp.Y()));
  this->dataPtr->xRangePos.Y(std::max(tmp.X(), tmp.Y()));

  // Get y position clamping values
  if (_sdf->HasElement("min_y_pos"))
    this->dataPtr->yRangePos.X(_sdf->Get<double>("min_y_pos"));
  if (_sdf->HasElement("max_y_pos"))
    this->dataPtr->yRangePos.Y(_sdf->Get<double>("max_y_pos"));

  // Make sure min <= max
  tmp = this->dataPtr->yRangePos;
  this->dataPtr->yRangePos.X(std::min(tmp.X(), tmp.Y()));
  this->dataPtr->yRangePos.Y(std::max(tmp.X(), tmp.Y()));

  // Get z position clamping values
  if (_sdf->HasElement("min_z_pos"))
    this->dataPtr->zRangePos.X(_sdf->Get<double>("min_z_pos"));
  if (_sdf->HasElement("max_z_pos"))
    this->dataPtr->zRangePos.Y(_sdf->Get<double>("max_z_pos"));

  // Make sure min <= max
  tmp = this->dataPtr->zRangePos;
  this->dataPtr->zRangePos.X(std::min(tmp.X(), tmp.Y()));
  this->dataPtr->zRangePos.Y(std::max(tmp.X(), tmp.Y()));

  // Set the initial velocity, if present
  if (_sdf->HasElement("initial_velocity"))
  {
    this->dataPtr->velocity =
      _sdf->Get<ignition::math::Vector3d>("initial_velocity");
  }

  // Set the velocity factor
  if (_sdf->HasElement("velocity_factor"))
    this->dataPtr->velocityFactor = _sdf->Get<double>("velocity_factor");

  // Set the update period
  if (_sdf->HasElement("update_period"))
    this->dataPtr->updatePeriod = _sdf->Get<double>("update_period");

  // Connect to the world update signal
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&RandomVelocityPlugin2::Update, this, std::placeholders::_1));
}

/////////////////////////////////////////////////
void RandomVelocityPlugin2::Reset()
{
  this->dataPtr->prevUpdate.Set(0, 0);
}

/////////////////////////////////////////////////
void RandomVelocityPlugin2::Update(const common::UpdateInfo &_info)
{
  GZ_ASSERT(this->dataPtr->link, "<link> in RandomVelocity2 plugin is null");

  // Short-circuit in case the link is invalid.
  if (!this->dataPtr->link)
    return;

  // Change direction when enough time has elapsed
  if (_info.simTime - this->dataPtr->prevUpdate > this->dataPtr->updatePeriod)
  {
    // Get a random velocity value.
    this->dataPtr->velocity.Set(
        ignition::math::Rand::DblUniform(-1, 1),
        ignition::math::Rand::DblUniform(-1, 1),
        ignition::math::Rand::DblUniform(-1, 1));

    // Apply scaling factor
    this->dataPtr->velocity.Normalize();
    this->dataPtr->velocity *= this->dataPtr->velocityFactor;

    // Clamp X value
    this->dataPtr->velocity.X(ignition::math::clamp(this->dataPtr->velocity.X(),
        this->dataPtr->xRange.X(), this->dataPtr->xRange.Y()));

    // Clamp Y value
    this->dataPtr->velocity.Y(ignition::math::clamp(this->dataPtr->velocity.Y(),
        this->dataPtr->yRange.X(), this->dataPtr->yRange.Y()));

    // Clamp Z value
    this->dataPtr->velocity.Z(ignition::math::clamp(this->dataPtr->velocity.Z(),
        this->dataPtr->zRange.X(), this->dataPtr->zRange.Y()));

    this->dataPtr->prevUpdate = _info.simTime;

  }

  //gzerr<<this->dataPtr->xRangePos.X()<<"\n";

  // Apply position bounding box
  ignition::math::Pose3d pose = this->dataPtr->link->WorldCoGPose();
  ignition::math::Vector3<double> position = pose.Pos();

  if ((position.X() <= this->dataPtr->xRangePos.X() && this->dataPtr->velocity.X() <= 0) ||
      (position.X() >= this->dataPtr->xRangePos.Y() && this->dataPtr->velocity.X() >= 0) )
     this->dataPtr->velocity.X(0);

  if (position.Y() <= this->dataPtr->yRangePos.X() && this->dataPtr->velocity.Y() <= 0 ||
      position.Y() >= this->dataPtr->yRangePos.Y() && this->dataPtr->velocity.Y() >= 0)
     this->dataPtr->velocity.Y(0);

  if (position.Z() <= this->dataPtr->zRangePos.X() && this->dataPtr->velocity.Z() <= 0 ||
      position.Z() >= this->dataPtr->zRangePos.Y() && this->dataPtr->velocity.Z() >= 0)
     this->dataPtr->velocity.Z(0);
 
  // Apply velocity
  this->dataPtr->link->SetLinearVel(this->dataPtr->velocity);
}