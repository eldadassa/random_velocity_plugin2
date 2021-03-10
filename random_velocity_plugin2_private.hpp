#ifndef GAZEBO_PLUGINS_RANDOMVELOCITYPLUGIN2PRIVATE_HH_
#define GAZEBO_PLUGINS_RANDOMVELOCITYPLUGIN2PRIVATE_HH_

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>

#include <gazebo/physics/Link.hh>
#include <gazebo/common/Time.hh>

namespace gazebo
{
  /// \internal
  /// \brief Private data for the RandomVelocityPlugin.
  class RandomVelocityPlugin2Private
  {
    public: RandomVelocityPlugin2Private()
            : velocityFactor(1.0),
              updatePeriod(10, 0),
              xRange(-ignition::math::MAX_D, ignition::math::MAX_D),
              yRange(-ignition::math::MAX_D, ignition::math::MAX_D),
              zRange(-ignition::math::MAX_D, ignition::math::MAX_D),
              xRangePos(-ignition::math::MAX_D, ignition::math::MAX_D),
              yRangePos(-ignition::math::MAX_D, ignition::math::MAX_D),
              zRangePos(-ignition::math::MAX_D, ignition::math::MAX_D)
            {
            }

    /// \brief Velocity scaling factor.
    public: double velocityFactor;

    /// \brief Time between recomputing a new velocity vector
    public: common::Time updatePeriod;

    /// \brief Time the of the last update.
    public: common::Time prevUpdate;

    /// \brief Velocity to apply.
    public: ignition::math::Vector3d velocity;

    /// \brief Connects to world update event.
    public: event::ConnectionPtr updateConnection;

    /// \brief X velocity clamping values
    public: ignition::math::Vector2d xRange;

    /// \brief Y velocity clamping values
    public: ignition::math::Vector2d yRange;

    /// \brief Z velocity clamping values
    public: ignition::math::Vector2d zRange;

    /// \brief X position clamping values
    public: ignition::math::Vector2d xRangePos;

    /// \brief Y position clamping values
    public: ignition::math::Vector2d yRangePos;

    /// \brief Z position clamping values
    public: ignition::math::Vector2d zRangePos;

    /// \brief Pointer to the link that will receive the velocity.
    public: physics::LinkPtr link;
  };
}
#endif
