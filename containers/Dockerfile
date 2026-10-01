FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt/uav_lab
COPY dependencies/build-requirements.txt dependencies/
COPY scripts/install_host.sh scripts/
RUN bash scripts/install_host.sh --system-only
COPY dependencies/lock.json dependencies/
COPY scripts/bootstrap.py scripts/build.sh scripts/
RUN /usr/bin/python3 scripts/bootstrap.py --jobs 2 && ./scripts/build.sh
COPY . .
RUN ./scripts/build.sh && /usr/bin/python3 scripts/doctor.py --runtime
ARG LAB_REVISION=source-archive
ENV LAB_REVISION=${LAB_REVISION}
CMD ["./scripts/start_lab.sh", "--headless"]
