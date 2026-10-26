FROM docker.io/osrf/ros:jazzy-desktop-full

# Install Zivid SDK 2.17.2 for Ubuntu 24.04 (Noble)
RUN apt-get update && apt-get install -y --no-install-recommends wget && \
    mkdir -p /tmp/zivid && cd /tmp/zivid && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u24/amd64/zivid_2.17.2+440b2367-1_amd64.deb && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u24/amd64/zivid-studio_2.17.2+440b2367-1_amd64.deb && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u24/amd64/zivid-tools_2.17.2+440b2367-1_amd64.deb && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u24/amd64/zivid-genicam_2.17.2+440b2367-1_amd64.deb && \
    apt-get install -y --no-install-recommends /tmp/zivid/*.deb && \
    rm -rf /tmp/zivid /var/lib/apt/lists/*

WORKDIR /root/ws/erobs

COPY src/ src/
COPY utils/ utils/
# Import driver repos and install ROS dependencies.
RUN chmod +x utils/setup.sh utils/start_mcp_robot_stack.sh && \
    bash utils/setup.sh && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=0

# Source workspace in all shells (podman exec, interactive, etc.)
RUN echo "source /root/ws/erobs/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/root/ws/erobs/utils/start_mcp_robot_stack.sh"]
