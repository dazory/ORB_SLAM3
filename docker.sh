#!/bin/bash
if [ "${1:-run}" == "run" ]; 
then
    # UI permisions
    XSOCK=/tmp/.X11-unix
    XAUTH=/tmp/.docker.xauth
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

    xhost +local:docker
    xhost +local:root

    # Remove existing container
    docker rm -f orbslam3 &>/dev/null

    # Create a new container
    docker run -td --privileged --net=host --ipc=host \
        --name="orbslam3" \
        -e "DISPLAY=$DISPLAY" \
        -e "QT_X11_NO_MITSHM=1" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -e "XAUTHORITY=$XAUTH" \
        -e ROS_IP=127.0.0.1 \
        --cap-add=SYS_PTRACE \
        -v `pwd`/Datasets:/Datasets \
        -v /etc/group:/etc/group:ro \
        -v `pwd`/ORB_SLAM3:/ORB_SLAM3 \
        jahaniam/orbslam3:ubuntu18_melodic_cpu bash
fi

if [ "${1:-exec}" == "exec" ]; 
then
    docker exec -it orbslam3 bash
fi
