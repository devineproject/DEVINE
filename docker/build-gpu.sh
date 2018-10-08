#!/bin/bash

ROOT=$(dirname "$(readlink -f "$0")")/..
cd $ROOT

sudo docker build -f docker/Dockerfile -t devine --build-arg base_docker=latest-gpu .
