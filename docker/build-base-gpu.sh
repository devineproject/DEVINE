#!/bin/bash

ROOT=$(dirname "$(readlink -f "$0")")/..

sudo docker build -f $ROOT/docker/base/Dockerfile -t devine-base-gpu --build-arg guesswhat_docker=latest-gpu --build-arg tensorflow_package=tensorflow-gpu $ROOT
