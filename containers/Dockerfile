FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt/uav_lab
COPY dependencies/build-requirements.txt dependencies/observation-requirements.txt dependencies/
COPY scripts/install_host.sh scripts/
RUN bash scripts/install_host.sh --system-only
COPY dependencies/lock.json dependencies/
COPY scripts/bootstrap.py scripts/build.sh scripts/
RUN /usr/bin/python3 scripts/bootstrap.py --jobs 2 && ./scripts/build.sh
COPY scripts/bootstrap_slam.py scripts/slam_config.py scripts/env.sh scripts/
COPY ros2_ws/src/uav_lab_localization/uav_lab_localization/__init__.py ros2_ws/src/uav_lab_localization/uav_lab_localization/configuration.py ros2_ws/src/uav_lab_localization/uav_lab_localization/
COPY configs/sensors.json configs/sensors.json
COPY configs/slam/ configs/slam/
COPY localization/native/ localization/native/
RUN ./scripts/env.sh /usr/bin/python3 scripts/bootstrap_slam.py --jobs 2
COPY . .
RUN ./scripts/build.sh && /usr/bin/python3 scripts/doctor.py --runtime
ARG LAB_REVISION=source-archive
ENV LAB_REVISION=${LAB_REVISION}
CMD ["./scripts/start_lab.sh", "--headless"]
