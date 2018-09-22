#!/bin/bash

# might need to `xhost +` first
sudo docker run -p 9090:9090 -p 8080:8080 -it --rm \
 -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
 devine bash
