FROM tensorflow/tensorflow:latest-gpu

WORKDIR /usr/src

COPY . .

RUN apt-get update && apt-get install -y wget && ./install_pretrained.sh
