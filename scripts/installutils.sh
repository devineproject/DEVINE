#!/bin/bash

datapath=~/.devine/data

install() {
  local catkinsrc=$1
  local devineroot=$2
  local tensorflow_package=$3

  confirm "This script was made for a fresh \033[0;31m16.04 Ubuntu\033[0m desktop image and may \033[0;31mharm\033[0m your system. Installation will be effective for user \033[0;31m$(whoami)\033[0m, continue" || exit 1

  install_base "$catkinsrc" "$tensorflow_package"
  install_devine "$catkinsrc" "$devineroot"

  echo -e "\033[0;31mReload bash for $(whoami) to finish installation\033[0m"
}

install_devine() {
  local catkinsrc=$1
  local devineroot=$2

  ln -sf "$(readlink -f $devineroot)" "$(readlink -f $catkinsrc)"
  
  pushd "$catkinsrc"

  cd DEVINE/src/guesswhat
  python3 -m pip install --user -r requirements.txt
  unzip -o "$datapath/weights.zip" -d data
  cd ../image_processing
  python3 -m pip install --user -r requirements.txt
  ln -sf "$datapath/mask_rcnn_coco.h5" mask_rcnn_coco.h5
  tar --overwrite -xzf "$datapath/vgg_16_2016_08_28.tar.gz"
  ln -sf "$(find "$(dirname "$(python3 -c 'import tf_pose;print(tf_pose.__file__)')")"/.. -name mobilenet_thin)/graph_opt.pb" mobilenet_thin.pb
  cd ../dialog
  python2 -m pip install --user -r requirements.txt
  cd ../robot_control
  mkdir -p ~/.rviz
  cp -f launch/irl_point.rviz ~/.rviz/default.rviz
  cd ../head_coordinator
  cp -f apriltags2_config/* ../../../apriltags2_ros/apriltags2_ros/config
  cd ../common
  python2 -m pip install --user -r requirements.txt

  cd ../../../..
  catkin_make

  cd src/DEVINE/src/dashboard
  python3 -m pip install --user -r requirements.txt
  npm install && npm run build

  popd
}

install_base() {
  local catkinsrc=$1
  local tensorflow_package=$2

  pushd "$catkinsrc"
  mkdir -p "$datapath"
  as_su apt-get update
  as_su apt-get install -y apt-transport-https git libffi-dev
  as_su add-apt-repository -y ppa:ubuntu-toolchain-r/test
  as_su sh -c 'echo "deb https://ftp.osuosl.org/pub/ros/packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  as_su apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 || exit 1
  as_su sh -c 'echo "deb https://debian.snips.ai/stretch stable main" > /etc/apt/sources.list.d/snips.list'
  as_su apt-key adv --keyserver hkp://pgp.mit.edu --recv-key F727C778CCB0A455 || \
  curl -L https://github.com/devineproject/static/releases/download/v0.0.1/snips.key | as_su apt-key add -
  as_su apt-get update
  as_su apt-get upgrade -y libstdc++6
  as_su apt-get install -y python3 python3-tk python3-pip python python-pip cython3
  as_su apt-get install -y ros-kinetic-desktop-full ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control
  as_su apt-get install -y ros-kinetic-openni-launch ros-kinetic-openni-camera ros-kinetic-openni-description ros-kinetic-compressed-image-transport
  as_su apt-get install -y ros-kinetic-rosbridge-server
  as_su apt-get install -y ros-kinetic-joy
  as_su apt-get install -y snips-platform-voice=0.58.3
  ensure_data https://github.com/projetdevine/static/releases/download/v0.0.1/assistant.zip
  as_su unzip -o "$datapath/assistant.zip" -d /usr/share/snips
  as_su python2 -m pip install --upgrade pip setuptools wheel pyopenssl cryptography
  as_su python3 -m pip install --upgrade pip setuptools wheel pyopenssl cryptography
  python3 -m pip install --user $tensorflow_package
  python2 -m pip install --user opencv-contrib-python
  python3 -m pip install --user opencv-contrib-python
  ensure_data https://github.com/matterport/Mask_RCNN/releases/download/v2.0/mask_rcnn_coco.h5
  ensure_data https://storage.googleapis.com/download.tensorflow.org/models/vgg_16_2016_08_28.tar.gz
  ensure_data https://github.com/projetdevine/static/releases/download/v0.0.1/weights.zip
  as_su rm -f /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so
  python3 -m pip install --user 'git+https://github.com/ildoonet/tf-pose-estimation.git@b119759e8a41828c633bd39b5c883bf5a56a214f#egg=tf_pose'
  IFS= x=$(curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh | bash | tail -n 3)
  eval $x
  nvm install --lts

  if python3 -c "import guesswhat" 2>&1 | grep '^' > /dev/null
  then
    python3 -m pip install --user nltk tqdm image
    git clone --recursive https://github.com/devineproject/guesswhat.git || exit 1
    ensure_line "export \"PYTHONPATH=$(pwd)/guesswhat/src:\$PYTHONPATH\"" ~/.bashrc
  fi

  if [ ! -f IRL-1 ]
  then
    git clone https://github.com/introlab/IRL-1.git
  fi

  if [ ! -f apriltags2_ros ]
  then
    git clone https://github.com/dmalyuta/apriltags2_ros.git
  fi

  ensure_line ". /opt/ros/kinetic/setup.sh" ~/.bashrc
  ensure_line "export \"ROS_PACKAGE_PATH=$(pwd):\$ROS_PACKAGE_PATH\"" ~/.bashrc
  cd ..
  ensure_line ". $(pwd)/devel/setup.bash" ~/.bashrc
  if [ ! -d /etc/ros/rosdep ]
  then
    as_su rosdep -q init
  fi
  rosdep -q update
  catkin_make

  popd
}

as_su() {
  if [ "$EUID" -ne 0 ]
  then
    sudo "$@"
  else
    "$@"
  fi
}

ensure_data() {
  local url=$1
  local file=${url##*/}

  if [ ! -f "$datapath/$file" ]
  then
    wget "$url" -P "$datapath" -q
  fi
}

ensure_line() {
  local line=$1
  local file=$2

  eval $line
  if ! grep -q -F "$line" "$file"
  then
    echo "$line" >> "$file"
  fi
}

get_keypress() {
  local REPLY IFS=
  >/dev/tty printf "$*"
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
