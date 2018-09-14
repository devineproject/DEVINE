FROM devineproject/devine-base

COPY . /usr/src/DEVINE

WORKDIR /usr/src/DEVINE

RUN \
# dashboard
    cd dashboard \
    && pip3 install -r requirements.txt \
    && bash -ci 'npm install && npm run build' \
# guesswhat
    && cd ../guesswhat \
    && unzip /data/weights.zip -d devine_guesswhat/data \
    && rm -f /data/weights.zip \
# image_processing
    && cd ../image_processing \
    && pip3 install Cython \
    && pip3 install scikit-image bson pymongo pycocotools keras==2.1.6 catkin_pkg rospkg \
    && ln -s /data/mask_rcnn_coco.h5 mask_rcnn_coco.h5 \
    && tar xzf /data/vgg_16_2016_08_28.tar.gz \
    && rm -f /data/vgg_16_2016_08_28.tar.gz \
    && git clone https://github.com/ildoonet/tf-pose-estimation.git \
    && cd tf-pose-estimation \
# hack remove this once body tracking is updated to a python3 node
    && sed -i 's/matplotlib >= 2.2.2/matplotlib == 2.2.2/' setup.py \
    && python2 setup.py install \
    && pip2 install tensorflow --ignore-installed enum34 \
    && cd .. \
    && rm -rf tf-pose-estimation \
    && ln -s $(find /usr/local/lib/python2.7/dist-packages/ -name mobilenet_thin)/graph_opt.pb mobilenet_thin.pb \
# game_system
    && cd ../game_system \
    && pip2 install transitions \
# snips
    && pip2 install paho-mqtt \
# robot sim
    && cd ../robot_control \
    && mkdir /root/.rviz \
    && cp irl_control/irl_point.rviz /root/.rviz/default.rviz 

# catkin
RUN cd /usr \
    && echo ". /opt/ros/kinetic/setup.sh" >> ~/.bashrc \
    && echo "export ROS_PACKAGE_PATH=/usr/src:$ROS_PACKAGE_PATH" >> ~/.bashrc \
    && rosdep init && rosdep update \
    && bash -ci catkin_make
