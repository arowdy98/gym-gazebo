Compiling the plugin:

<Gazebo 7 required>
1. mkdir build
2. cd build
3. cmake ..
4. make



Add your library path to the GAZEBO_PLUGIN_PATH:

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:..husky_plugin/build



To give this velocity to a model:

Add the following lines:

<plugin name="random" filename="libvelset.so">

<!-- Name of the link in this model that receives the velocity -->
<link>link</link>

<!-- Initial velocity that is applied to the link -->
<initial_velocity>0 0.5 0</initial_velocity>

<!-- Scaling factor that is used to compute a new velocity -->
<velocity_factor>0.5</velocity_factor>

<!-- Time, in seconds, between new velocities -->
<update_period>5</update_period>

<!-- Clamp the Z velocity value to zero. You can also clamp x and
     y values -->
<min_x>-5</min_x>
<max_x>5</max_x>
<min_y>-5</min_y>
<max_y>5</max_y>

<min_z>0</min_z>
<max_z>0</max_z>
</plugin>

