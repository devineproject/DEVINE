#!/bin/bash
ROOT=$(dirname "$(readlink -f "$0")")/..

build_gpu_version='false'

while test $# -gt 0; do
    case "$1" in
        -h|--help)
            echo "Build the DEVINE Docker image"
            echo " "
            echo "Note: might need to 'xhost +' first"
            echo " "
            echo "options:"
            echo "-h, --help                show brief help"
            echo "-g, --gpu                 build image for GPU use"
            exit 0;;
        -g|--gpu)
            build_gpu_version='true'
            shift;;
    esac
done


if $build_gpu_version
then
cmd_args='-t devine-gpu --build-arg base_docker=latest-gpu'
else
cmd_args='-t devine'
fi

CMD="sudo docker build -f $ROOT/docker/Dockerfile $cmd_args $ROOT"

echo $CMD
eval $CMD
