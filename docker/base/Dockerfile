FROM guesswhat

WORKDIR /usr/src

# add here large frameworks/technologies/languages required for DEVINE
RUN apt-get install apt-transport-https \
    && sh -c 'echo "deb https://ftp.osuosl.org/pub/ros/packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
    && sh -c 'echo "deb https://debian.snips.ai/jessie stable main" > /etc/apt/sources.list.d/snips.list' \
    && apt-key adv --keyserver hkp://pgp.mit.edu --recv-key F727C778CCB0A455 \
    && apt-get update \
    && apt-get install -y python3-tk \
    && apt-get install -y ros-kinetic-desktop-full ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control \
    && apt-get install -y ros-kinetic-openni-launch ros-kinetic-openni-camera ros-kinetic-openni-description ros-kinetic-compressed-image-transport \
    && apt-get install -y ros-kinetic-rosbridge-server \
    && apt-get install -y snips-platform-voice \
    && apt-get install -y python-pip \
    && pip2 install --upgrade pip setuptools wheel \
    && pip3 install --upgrade pip setuptools wheel \
    && pip2 install opencv-contrib-python \
    && pip3 install opencv-contrib-python \
    && mkdir /data \
    && wget https://github.com/matterport/Mask_RCNN/releases/download/v2.0/mask_rcnn_coco.h5 -P /data \
    && wget https://storage.googleapis.com/download.tensorflow.org/models/vgg_16_2016_08_28.tar.gz -P /data \
    && wget https://github.com/projetdevine/static/releases/download/v0.0.1/weights.zip -P /data \
# work around to get opencv working with py2 and py3, requires removing the one from ros packages
    && rm -f /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so \
    && apt-get install -y git \
    && curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh | bash \
    && bash -ci 'nvm install --lts' \
    && git clone https://github.com/introlab/IRL-1.git
