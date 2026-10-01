FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt/uav_lab
COPY . .
RUN bash scripts/install_host.sh
CMD ["./scripts/start_lab.sh", "--headless"]
