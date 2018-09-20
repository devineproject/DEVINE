#!/usr/bin/env bash

BASEDIR="$( cd "$(dirname "$0")"; pwd -P )"

sudo docker run --rm -it -v $BASEDIR/docs:/documents suttang/sphinx-rtd-theme make html

xdg-open $BASEDIR/docs/build/html/index.html
