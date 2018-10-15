#!/bin/bash

if [ ! "$#" -eq 2 ]
then
  echo "Bad arguments. Expected $0 catkin_src_dir devine_dir"
  exit 1
fi

ROOT=$(dirname "$(readlink -f "$0")")/..

. $ROOT/scripts/installutils.sh


dir_exists() {
  if [ ! -d "$1" ]
  then
    echo "$1 is not a directory"
    exit 1
  fi
}

dir_exists "$1"
dir_exists "$2"

dir="${1%/}"
if [ "${dir##*/}" != "src" ]
then
    echo "catkin_src_dir must be named src. Is named ${dir##*/}"
    exit 1
fi

install "$1" "$2" "tensorflow"
