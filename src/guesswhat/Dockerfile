ARG tensorflow_docker=1.4.1-py3
ARG tensorflow_package=tensorflow
FROM tensorflow/tensorflow:${tensorflow_docker}

WORKDIR /usr/src

COPY . .

ENV PYTHONPATH="/usr/src/guesswhat/src:$PYTHONPATH"
RUN apt-get update && apt-get install -y wget && ./install_pretrained.sh ${tensorflow_package}
