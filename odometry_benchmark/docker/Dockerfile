# Linux runtime for the two odometry methods that ship no Windows wheels:
#   GenZ-ICP 0.3.2 (Lee et al., RA-L 2025)
#   MAD-ICP 0.0.10 (Ferrari et al., RA-L 2024)
# Build from the benchmark root so the vendored sources are in the context:
#   docker build -t odom-bench:1.0 -f docker/Dockerfile .
FROM python@sha256:0a310eeecf4e1f5a0743f9a6520c90c88d089c903ca5fd283f501e3a805f5f89
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git libgomp1 libeigen3-dev libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*
# Every Python dependency pinned to the versions the published results used.
COPY env/docker_requirements.lock.txt /tmp/lock.txt
RUN grep -vE "^(genz-icp|mad-icp)==" /tmp/lock.txt > /tmp/deps.txt \
    && pip install --no-cache-dir -r /tmp/deps.txt \
    && pip install --no-cache-dir "scikit-build-core==0.9.10" "pybind11==3.1.0"
# Methods built from the vendored, hash-checked PyPI source archives (not the network).
COPY third_party/sdist /tmp/sdist
RUN cd /tmp/sdist \
    && echo "ed4463f5d7c1c951c481ae554323f4d83f98954fdf1c401e92386ad297a528ff  genz_icp-0.3.2.tar.gz" | sha256sum -c - \
    && echo "e1e3b7d56100e3928dce0d0c920348fb9dec6fa824512e0b601ef59ff7a11500  mad_icp-0.0.10.tar.gz" | sha256sum -c - \
    && pip install --no-cache-dir --no-deps genz_icp-0.3.2.tar.gz \
    # mad-icp's pyproject uses cmake.verbose, removed in scikit-build-core 0.10
    && pip install --no-cache-dir --no-deps --no-build-isolation mad_icp-0.0.10.tar.gz
WORKDIR /work
