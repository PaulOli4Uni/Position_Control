
## Building Project

Make project in 'Gz_Plugins' folder. (.cc file, .hh file and a CMakeLists file)

~~~
cd Gz_Plugins/<Project Folder>
mkdir build
cd build
cmake ..
make
~~~

This will generate the 'ProjectName' library under `build`.

## Run

The plugin must be attached to an entity to be loaded. Therefore the plugin needs to be added to a entity of a .SDF file

Before starting Gazebo, we must make sure it can find the plugin by doing:

~~~
cd Gz_Plugins/<Project Folder>
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then load the example world:

    gz sim -v 3 hello_world_plugin.sdf

You should see green messages on the terminal like:

```
[Msg] Hello, world! Simulation is paused.
```

Toggle the play / pause buttons to see the message change.


## SDF File Plugin options
# Needed
<timestep_precision> Integer (ranges from 1, infinty) that sets the precision of the system. 
Smaller number will be more precise, but less smooth


# Optional
<pose_topic> Changes topic name to which Pose information should be sent to. If not present the topic will pulblish to "/model/<model_name>/pos_contr"
Format > Text < 

Sending message to subscriber topic from terminal
gz topic -t /<topic_name> -m gz.msgs.StringMsg -p 'data:"Text"'

<file_topic> Changes topic name to which File_with_Pose.txt information should be sent to. If not present the topic will pulblish to "/model/<model_name>/file_pos_contr"
Format > Text < 



-Use the following offsets to place the centre of the model
<xyz offset>  XYZ offset of pose. (Meters)
    Format > 0.0 0.0 0.0 < 
<rpy offset> Roll, Pitch, Yaw offset. (Radians)
    Format > 0.0 0.0 0.0 < 


## Terminal message that might be needed
For both messages 'box' can be substituted to a different model name

-Send a Pose message
gz topic -t /box -m gz.msgs.StringMsg -p 'data:"-10,0,0,0,0,0,1"'

-Get Pose of the model 
gz model -m "box" -p
