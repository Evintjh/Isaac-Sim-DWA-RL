#!/bin/bash


XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
if [ ! -z "$xauth_list" ]
then
echo $xauth_list | xauth -f $XAUTH nmerge -
else
touch $XAUTH
fi
chmod a+r $XAUTH
fi

export ROS_REMOTE_PC=$(ifconfig enp70s0 | awk '/inet / {print $2}') # for joesbox
export ROS_PORT=11011

docker run --name isaac-sim-ros-2 --entrypoint bash -it --gpus device=1 -e "ACCEPT_EULA=Y" --rm --network=host \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache/Kit:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    -v ~/Reinforcement-Learning-on-Corriere-LM:/isaac-sim/src:rw \
    -v ~/OmniIsaacGymEnvs:/isaac-sim/OmniIsaacGymEnvs:rw\
    --env "ROS_MASTER_URI=http://$ROS_REMOTE_PC:$ROS_PORT" \
    --env "ROS_HOSTNAME=$ROS_REMOTE_PC" \
    isaac_sim_with_ros:justin

docker exec isaac-sim-ros-2 bash ./python.sh -m pip install rospkg 
docker exec isaac-sim-ros-2 bash ./python.sh -m pip install omegaconf
docker exec isaac-sim-ros-2 cp /isaac-sim/src/etc/differential_controller.py /isaac-sim/exts/omni.isaac.wheeled_robots/omni/isaac/wheeled_robots/controllers/differential_controller.py