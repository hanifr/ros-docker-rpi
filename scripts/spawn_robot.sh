#!/bin/bash
docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    export GAZEBO_MASTER_URI=http://gazebo:11345 && \
    roslaunch my_robot_gazebo spawn_robot.launch"