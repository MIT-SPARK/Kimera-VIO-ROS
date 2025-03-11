#!/bin/bash

# Base script taken from https://github.com/MIT-SPARK/Kimera-VIO/blob/master/scripts/docker/kimera_vio_docker.bash
DOCKER_CONTAINER="kimera_vio_ros"

if [ "$#" -eq 1 ]; then
    DOCKER_CONTAINER=$1
    echo "container name: $DOCKER_CONTAINER"
fi

# Allow X server connection
xhost +local:root
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    $DOCKER_CONTAINER
# Disallow X server connection
xhost -local:root
