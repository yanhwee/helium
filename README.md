# Helium
A drone synthetic environment development project based on Ardupilot, Gazebo & ROS.

## Learning the Basics
https://github.com/yanhwee/ardupilot-gazebo-ros-guide

## Installation

### Installation Script
From the terminal:  
`wget -O - https://raw.githubusercontent.com/yanhwee/helium/master/docs/install.bash | bash`

This installation script is modified based on the installation from https://github.com/yanhwee/ardupilot-gazebo-ros-guide. Refer to it for more details. It installs the following prerequisites and setups this repository. Note that this repository is a catkin package.

### Prerequisites
#### Main SITL Stack
- ArduPilot (Modified)
- ArduPilot Gazebo Plugin (Modified)
- Gazebo 9
- ROS Melodic
- MAVROS
#### Supporting Stack
- Catkin Workspace & Helium Package
- QGroundControl (QGC) Daily Builds
- Python Libraries
    - pymavlink

## Quick Test (SITL Stack)
1. Open four terminals (Tip: Ctrl-Shift-T)
2. In each:
    1. roslaunch Gazebo world  
    `roslaunch helium hills_lidar.world`
    2. Start ArduPilot SITL  
    `sim_vehicle.py -v ArduCopter -f gazebo-iris`
    3. Start QGroundControl (wait a few seconds before doing this step)  
    `./QGroundControl.AppImage`
    4. Start MAVROS (wait a few seconds before doing this step as well)  
    `roslaunch helium apm.launch`
3. Note (for first time):
    1. `sim_vehicle.py` will need time to compile so wait till its done before continuing

## Features Implemented for ArduCopter SITL
For ArduCopter Version: 4.0.4-dev

Forked repositories:

- ArduPilot  
https://github.com/yanhwee/ardupilot

- ArduPilot Gazebo Plugin (khancyr)  
https://github.com/yanhwee/ardupilot_gazebo

### 1. Terrain Following
1. References
    1. https://discuss.ardupilot.org/t/gazebo-with-lidar/24717/6

2. Psuedo-Diagram  
    ![ArduPilot Gazebo Diagram](docs/gazebo-to-ardupilot.png)

3. Walk-through  
    1. SIM_Gazebo
        1. Modify the FDM Packet to receive the rangefinder value. 
        2. Then, moidfy the function that updates the range value (in SIM_Gazebo) using the received rangefinder value.
    
    2. ArduPilotPlugin.hh/.cc
        1. Modify the FDM Packet to take in the rangefinder value. 
        2. Use Gazebo transport node to read laser topic messages published by laser ray model.

    3. Gazebo  
        1. Create a model file for Lidar.
        2. Use ray sensors with one laser. 
        3. Add this lidar model to iris_with_ardupilot, using links and joints.
            - Laser topic messages is automatically published.

4. ArduPilot Parameters
    1. References
        1. https://ardupilot.org/copter/docs/terrain-following.html
    2. By QGroundControl or MAVProxy
        1. TERRAIN_ENABLE = 1 (Default)

5. Test
    1. QGroundControl
        1. Select MAVLink Inspector
        2. Find Distance Sensor
        3. Plot, 60 secs (optional)

    2. MAVProxy (Alternatively, less preferred way)
        1. https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#adding-a-virtual-rangefinder
        2. Don't `param set` anything
        3. Just load the graph

### 2. Object Avoidance
1. References
    1. https://github.com/ArduPilot/ardupilot/issues/5608
    2. https://answers.gazebosim.org//question/18881/publish-a-gazebo-message-into-a-ros-topic/

2. Walk-through
    1. Gazebo
        1. Create model file for 360 lidar.
            1. Ensure the lasers are setup in a "circular" manner
                - Actually clockwise or not it doesn't matter
                    - Rather, ArduPilot parameter, `PRX_ORIENT`, controls the orientation of the proximity sensors
                - Just ensure that `min_angle` starts from 0
                - Don't overlap the first and last laser
                ```
                <samples>n</samples>
                <min_angle>0</min_angle>
                <max_angle>-x<max_angle>
                <!-- where x = 2pi / n * (n - 1) -->
                ```
        2. But this time, publish laser messages via MAVROS.
            1. Use a Gazebo Plugin, "libgazebo_ros_laser.so".
            2. Set publish topic target directly to "/mavros/obstacle/send".
                - It is a mavros rostopic that takes in Gazebo laser messages.
        3. Put lidar model on top of the iris_with_ardupilot using links and joints.

    2. Catkin Workspace
        1. Use roslaunch to run Gazebo. Create a launch file for that.
            - This is for "libgazebo_ros_laser.so", to be able to publish laser messages to rostopic.

3. ArduPilot Parameters
    1. References
        1. https://ardupilot.org/copter/docs/common-simple-object-avoidance.html
        2. https://ardupilot.org/copter/docs/common-oa-bendyruler.html
    2. By QGroundControl or MAVProxy
        1. AVOID_ENABLE = 7 (or any values that includes "Proximity Sensors")
        2. PRX_TYPE = 2 (MAVLink)
        3. PRX_ORIENT = 1 (Upside Down)
        4. OA_TYPE = 1 (BendyRuler)

4. Test
    1. Via Terminal
        1. `rostopic echo /mavros/obstacle/send`
    2. QGroundControl
        1. Ensure ArduPilot Parameter, `OA_DB_OUTPUT` >= 2 (Send HIGH and NORMAL importance item)
        2. Obstacle icons should show up on main GUI

### 3. Terrain Following with Object Avoidance
The current version of ArduCopter does not allow terrain following if object avoidance is enabled.

1. Psuedo-Diagram
![ArduPilot Object Avoidance Diagram](docs/ardupilot-oa.png)

2. Walk-through
    1. Regardless whether OA is enabled, `AC_WPNav_OA` will always be used rather than `AC_WPNav`. `AC_WPNav_OA::update_wpnav()` will always call `AC_WPNav::update_wpnav()` at the end. The main problem lies within `AC_WPNav_OA`.
    2. `AP_OAPathPlanner` uses threading to process and adjust the waypoints. Sometimes, it isn't ready and will return `AP_OAPathPlanner::OA_Processing`.
    3. A switch case in `AC_WPNav_OA` handles the results. However, if the `OAPathPlanner` is still processing, a false boolean will disable terrain following (for that waypoint). (This might be a bug?)
    4. Since the check occurs so frequently, terrain following will be disabled for all waypoints throughout the mission.
    5. The fix is to replace the false boolean to `_terrain_alt`, a boolean that stores the state of terrain following, true if enabled and false otherwise.