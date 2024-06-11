
# MODIFY BELOW (NOTE(jwd) - you may need to change the network id `wlp3s0` below)
#export ROS_REMOTE_PC=$(ifconfig wlp3s0 | awk '/inet / {print $2}')
export ROS_REMOTE_PC=$(ifconfig wlp6s0| awk '/inet / {print $2}')  # dreamcore enp4s0 corriere enp5s0 droc1 wlp6s0
export ROS_PORT=11311 #4 3
# END MODIFY

docker run --name justin-rl-ros --entrypoint bash -it --rm \
    --env DISPLAY=$DISPLAY \
    --env "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env "XAUTHORITY=$XAUTH" \
    --env "CUDA_VISIBLE_DEVICES=1" \
    -v "$XAUTH:$XAUTH" \
    -v /etc/localtime:/etc/localtime:ro \
    -v /home/droc1/Justin/src:/home/justin/src \
    --privileged \
    --network=host \
    --env "ROS_MASTER_URI=http://$ROS_REMOTE_PC:$ROS_PORT" \
    --env "ROS_HOSTNAME=$ROS_REMOTE_PC" \
    drl-noetic-deploy:latest


    