FROM ghcr.io/sloretz/ros:humble-desktop
SHELL ["/bin/bash", "-c"] 

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    socat \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

COPY src/wiringpi /tmp/wiringpi

RUN cd /tmp/wiringpi/wiringPi \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/wiringpi

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "if [ -f /app/ros2_ws/install/setup.bash ]; then source /app/ros2_ws/install/setup.bash; fi" >> ~/.bashrc

WORKDIR /app/ros2_ws
