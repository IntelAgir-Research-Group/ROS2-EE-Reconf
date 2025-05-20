#!/bin/bash

ros_load_param_file="ros2 param load /local_costmap/local_costmap"

files=`ls configs/`

function bringup_containers {
    cd ../../../../
    docker-compose -f docker/compose/compose-ros-humble-navigation-allinone.yml up gazebo 2>&1 >./$file-gazebo.log &
    sleep 15
    docker-compose -f docker/compose/compose-ros-humble-navigation-allinone.yml up nav2 2>&1 >./$file-nav2.log &
    sleep 30
    docker-compose -f docker/compose/compose-ros-humble-navigation-allinone.yml up rviz 2>&1 >./$file-rviz.log &
    sleep 15
    cd -
}

function bringdown_containers {
    docker rm -f $(docker ps -a -q)
    docker volume rm compose_nav2_profiler
    docker volume rm compose_nav2_data
    docker volume rm compose_exp_orchestration
}

function clean_profiling_data {
    rm ../../../data/*resource*
    cat /dev/null > ../../../data/rapl.csv
}

function dump_profiling_data {
    # Dump profiling data with a sugestive name (from docker/data to data)
    echo "Dump profiling data TBD"
}

function setting_configuration {
    echo "Experimenting with configuration "$file
    sleep 15
    docker exec nav2_container bash -c "source /opt/ros/humble/setup.bash && $ros_load_param_file /orchestrator/configs/$file"
    sleep 15
}

function robot_states {
    echo "robot states TBD"
    # simple: in a third log file, drop the timestamp we enter a state (it can be a single preset sequence of states)
    # complex: get a list of states, drive the robot and drop the timestamp we enter the state
    # we start with the simple as a MVP
}

for file in $files; do
    bringdown_containers
    clean_profiling_data
    bringup_containers
    setting_configuration
    robot_states
    dump_profiling_data
    break
done