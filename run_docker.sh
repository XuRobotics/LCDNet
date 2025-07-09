#!/bin/bash
xhost +local:root

docker run -it \
    --name="lcdnet" \
    --gpus all \
    --rm \
    -v /home/sam/bags:/bags \
    -v /home/sam/LCDNet:/LCDNet-XuRobotics \
    --shm-size=12gb \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --network host \
    --privileged \
    lcdnet \
    bash
