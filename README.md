# random_velocity_plugin2
modified random velocity plugin (libRandomVelocityPlugin.so) for the Gazebo robot simulator.

Added feature: Define a bounding box within the object is allowed to move (clip object 3D position)

To use this plugin clone the repository and build the plugin:
Create 'build' directory under the cloned directory (mkdir build)
Build the plugin library (cmake .. && make)
Add the path of the built so file to $GAZEBO_PLUGIN_PATH

Look at the sdf file for an example of how to integrate the plugin within a Gazebo world file.
