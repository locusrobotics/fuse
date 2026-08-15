FROM ros:rolling

RUN apt-get update && apt-get install -y --no-install-recommends \
    ccache \
    gdb \
    git \
    libbenchmark-dev \
    python3-vcstool \
  && rm -rf /var/lib/apt/lists/*
