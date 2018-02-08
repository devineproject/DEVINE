ARG tensorflow_docker=latest-py3
ARG tensorflow_package=tensorflow
FROM tensorflow/tensorflow:${tensorflow_docker}

WORKDIR /usr/src

COPY . .

RUN apt-get update && apt-get install -y wget && ./install_pretrained.sh ${tensorflow_package}
