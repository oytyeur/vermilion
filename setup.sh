#!/bin/bash
echo "Setup unitree ros2 environment"
# source /opt/ros/foxy/setup.bash
# source cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
#                             <NetworkInterface name="enp2s0" priority="default" multicast="default" />
#                         </Interfaces></General></Domain></CycloneDDS>'
export CYCLONEDDS_URI=/home/oytyeur/workspace/MGTS/Unitree/Go2Edu/vermilion/cyclonedds_ws/src/cyclonedds.xml                       
