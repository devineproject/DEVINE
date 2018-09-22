FROM devineproject/devine-base

COPY . /usr/src/DEVINE

WORKDIR /usr/src/DEVINE

RUN bash -ci ". scripts/installutils.sh && install_devine $(pwd)/.. $(pwd)"
