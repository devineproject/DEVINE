#!/bin/bash

ROOT=$(dirname "$(readlink -f "$0")")/..

sudo docker build -f $ROOT/docker/base/Dockerfile -t devine-base $ROOT
