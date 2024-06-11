#!/bin/bash

export ROS_REMOTE_PC=$(ifconfig enp70s0 | awk '/inet / {print $2}') # for joesbox
export ROS_PORT=11311

xhost +local:docker
docker run -it --rm --privileged --net=host --env=NVIDIA_VISIBLE_DEVICES=all \
    --env=NVIDIA_DRIVER_CAPABILITIES=all --env=DISPLAY --env=QT_X11_NO_MITSHM=1 \
    --env "ROS_MASTER_URI=http://$ROS_REMOTE_PC:$ROS_PORT" \
    --env "ROS_HOSTNAME=$ROS_REMOTE_PC" \
    -v /tmp/.X11-unix:/tmp/.X11-unix --gpus 2 rviz_test /bin/bash