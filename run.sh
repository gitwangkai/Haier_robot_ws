#!/bin/bash
xhost +local:docker
touch /tmp/.docker.xauth
xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -

# 运行容器
docker run -it \
  --net host \
  --name haier_robot_ws \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
  -v /usr/share/glvnd/egl_vendor.d:/usr/share/glvnd/egl_vendor.d \
  -v ./src:/root/ros2_ws/src \
  -v ./install:/root/ros2_ws/install \
  -v ./build:/root/ros2_ws/build \
  -v ./log:/root/ros2_ws/log \
  -v ./config:/root/ros2_ws/config \
  --device /dev/dri:/dev/dri \
  --device /dev/nvidia0:/dev/nvidia0 \
  --device /dev/nvidiactl:/dev/nvidiactl \
  --device /dev/nvidia-uvm:/dev/nvidia-uvm \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  --security-opt seccomp=unconfined \
  --privileged \
    aidlux/ros_amd64:haier_robot_base \
    /bin/bash