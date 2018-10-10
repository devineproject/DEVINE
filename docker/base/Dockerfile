ARG guesswhat_docker=latest

FROM devineproject/guesswhat:$guesswhat_docker

ARG tensorflow_package=tensorflow

ENTRYPOINT []

ENV PATH=/root/.local/bin:$PATH

WORKDIR /usr/src

COPY scripts/installutils.sh .

RUN bash -ci ". installutils.sh && install_base $(pwd) $tensorflow_package"
