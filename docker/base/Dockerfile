FROM devineproject/guesswhat

ENTRYPOINT []

ENV LIBGL_ALWAYS_SOFTWARE=1 PATH=/root/.local/bin:$PATH

WORKDIR /usr/src

COPY scripts/installutils.sh .

RUN bash -ci ". installutils.sh && install_base $(pwd)"
