#!/bin/bash

install() {
  local catkinsrc=$1
  local devineroot=$2
  confirm "This script was made for a fresh 16.04 Ubuntu image and may harm your system, continue" || exit

  install_base $catkinsrc
  install_devine $catkinsrc $devineroot

  echo reload bash for $(whoami) to finish installation
}

install_devine() {
  local catkinsrc=$1
  local devineroot=$2

  if [ ! -d "$catkinsrc/DEVINE" ]
  then
    ln -s "$catkinsrc/DEVINE" "$devineroot"
  fi

  pushd "$catkinsrc/DEVINE"

  if python3 -c "import guesswhat" 2>&1 | grep '^'
  then
    pip3 install nltk tqdm image
    git clone --recursive https://github.com/devineproject/guesswhat.git
    ensure_line "export \"$(pwd)/guesswhat/src:\$PYTHONPATH\"" ~/.bashrc
  fi

  # TODO move pip installs to respective setup.py
  cd dashboard
  pip3 install -r requirements.txt
  bash -ci 'npm install && npm run build'
  cd ../guesswhat
  unzip /data/weights.zip -d devine_guesswhat/data
  rm -f /data/weights.zip
  cd ../image_processing
  pip3 install Cython
  pip3 install scikit-image bson pymongo pycocotools keras==2.1.6 catkin_pkg rospkg
  ln -s /data/mask_rcnn_coco.h5 mask_rcnn_coco.h5
  tar xzf /data/vgg_16_2016_08_28.tar.gz
  rm -f /data/vgg_16_2016_08_28.tar.gz
  git clone https://github.com/ildoonet/tf-pose-estimation.git
  cd tf-pose-estimation
  # hack remove this once body tracking is updated to a python3 node
  sed -i 's/matplotlib >= 2.2.2/matplotlib == 2.2.2/' setup.py
  python2 setup.py install
  pip2 install tensorflow --ignore-installed enum34
  cd ..
  rm -rf tf-pose-estimation
  ln -s "$(find /usr/local/lib/python2.7/dist-packages/ -name mobilenet_thin)/graph_opt.pb" mobilenet_thin.pb
  cd ../game_system
  pip2 install transitions
  pip2 install paho-mqtt
  cd ../robot_control
  mkdir ~/.rviz
  cp irl_control/irl_point.rviz ~/.rviz/default.rviz

  cd ../../..
  bash -ci catkin_make

  popd
}

install_base() {
  local catkinsrc=$1
  pushd "$catkinsrc"

  as_su apt-get update
  as_su apt-get install -y apt-transport-https git
  as_su sh -c "'echo \"deb https://ftp.osuosl.org/pub/ros/packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list'"
  as_su apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
  as_su sh -c "'echo \"deb https://debian.snips.ai/jessie stable main\" > /etc/apt/sources.list.d/snips.list'"
  as_su apt-key adv --keyserver hkp://pgp.mit.edu --recv-key F727C778CCB0A455
  as_su apt-get update
  as_su apt-get install -y python3 python3-tk python3-pip
  as_su apt-get install -y ros-kinetic-desktop-full ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control
  as_su apt-get install -y ros-kinetic-openni-launch ros-kinetic-openni-camera ros-kinetic-openni-description ros-kinetic-compressed-image-transport
  as_su apt-get install -y ros-kinetic-rosbridge-server
  as_su apt-get install -y snips-platform-voice
  as_su apt-get install -y python-pip
  pip2 install --upgrade pip setuptools wheel pyopenssl cryptography
  pip3 install --upgrade pip setuptools wheel pyopenssl cryptography
  pip3 install tensorflow
  pip2 install opencv-contrib-python
  pip3 install opencv-contrib-python
  as_su mkdir -p /data
  local user=$(whoami)
  as_su chown $user:$user /data
  ensure_data https://github.com/matterport/Mask_RCNN/releases/download/v2.0/mask_rcnn_coco.h5
  ensure_data https://storage.googleapis.com/download.tensorflow.org/models/vgg_16_2016_08_28.tar.gz
  ensure_data https://github.com/projetdevine/static/releases/download/v0.0.1/weights.zip
  as_su rm -f /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so
  curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh | bash
  bash -ci 'nvm install --lts'

  if [ ! -f IRL-1 ]
  then
    git clone --branch ctrl_import https://github.com/introlab/IRL-1.git
  fi
  ensure_line ". /opt/ros/kinetic/setup.sh" ~/.bashrc
  ensure_line "export \"ROS_PACKAGE_PATH=$(pwd):\$ROS_PACKAGE_PATH\"" ~/.bashrc
  cd ..
  bash -ci "rosdep init && rosdep update && catkin_make"

  popd
}

as_su() {
  if [ "$EUID" -ne 0 ]
  then
    sudo $@
  else
    eval $@
  fi
}

ensure_data() {
  local url=$1
  local file=${url##*/}

  if [ ! -f /data/$file ]
  then
    wget $url -P /data
  fi
}

ensure_line() {
  local line=$1
  local file=$2

  if ! grep -q -F "$line" "$file"
  then
    echo "$line" >> "$file"
  fi
}

get_keypress() {
  local REPLY IFS=
  >/dev/tty printf '%s' "$*"
  [[ $ZSH_VERSION ]] && read -rk1
  [[ $BASH_VERSION ]] && </dev/tty read -rn1
  printf '%s' "$REPLY"
}

confirm() {
  local prompt="${*:-Are you sure} [y/N]? "
  local enter_return=1
  local REPLY
  while REPLY=$(get_keypress "$prompt"); do
    [[ $REPLY ]] && printf '\n'
    case "$REPLY" in
      Y|y)  return 0;;
      N|n)  return 1;;
      '')   [[ $enter_return ]] && return "$enter_return"
    esac
  done
}
