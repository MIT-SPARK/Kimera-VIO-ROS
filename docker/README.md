# Using the Kimera-VIO ROS wrapper with Docker

## Building and running your container
To build the container, in this directory, run the following:
```
docker build -t kimera_vio_ros:latest .
```
The Dockerfile also provide build arguments which allow the user to specify:
1. URL (default: https://github.com/MIT-SPARK/Kimera-VIO-ROS.git)- The URL for the repository that you want to clone. This is useful when developing using a fork.
2. BRANCH (default: master) - The branch on the _remote_ repository that should be checked out to. Also useful when doing development.
3. THREADS (default: 4)- The number of threads to use when building with catkin.

Example use with these flags:
```
docker build -t kimera_vio_ros:latest \
    --build-arg URL=https://github.com/MIT-SPARK/Kimera-VIO-ROS.git \
    --build-arg BRANCH=master \
    --build-arg THREADS=8 .
```

To run the container, run the following command. This script allows X11 forwarding, which allows Rviz to be used with the container.
```
./docker_test.bash [optional image name; default is kimera_vio_ros]
```

## Running the EuRoC demonstration with Rviz
For running the EuRoC demonstration with Rviz, follow the instructions [here](https://github.com/MIT-SPARK/Kimera-VIO-ROS?tab=readme-ov-file#2-usage).

These instructions mention running various commands in multiple terminals. This can be done with Docker in the following ways:

1. Using `docker exec` [commands](https://docs.docker.com/reference/cli/docker/container/exec/).
2. If you are using VS Code, you can use the Dev Containers [plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) to connect to your container and use terminals from within VS Code.
3. Using [tmux](https://www.redhat.com/en/blog/introduction-tmux-linux) for terminal multiplexing.

## Possible errors:
* If your trajectory tracking looks incorrect immediately on running, update the `autoInitialize` field from 0 to 1 in `/catkin_ws/src/Kimera-VIO/params/Euroc/BackendParams.yaml`.
  * In some configurations rviz seg faults if this value is 0. The exact cause is unknown, but will be investigated and fixed if possible.
* An X server should be installed on your local machine to allow Rviz to be used with the container.
* Currently, the default number of jobs for building is set to 4. With a 13th Gen Intel i7-1360P Processor, this takes up to ~8GB of RAM with no other applications running. You may want to reduce the number of jobs if your machine does not have enough memory.
