ARG guesswhat_docker=latest
ARG tensorflow_package=tensorflow

FROM devineproject/guesswhat:$guesswhat_docker

ENTRYPOINT []

ENV PATH=/root/.local/bin:$PATH

WORKDIR /usr/src

COPY scripts/installutils.sh .

RUN bash -ci ". installutils.sh && install_base $(pwd) $tensorflow_package"
