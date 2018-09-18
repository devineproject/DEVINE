FROM devineproject/guesswhat

ENV LIBGL_ALWAYS_SOFTWARE=1

WORKDIR /usr/src

COPY scripts/installutils.sh .

RUN bash -ci ". installutils.sh && install_base $(pwd)"
