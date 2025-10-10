FROM ros:humble

# Install ament linting tools
RUN apt-get update && apt-get install -y \
    ros-humble-ament-lint \
    ros-humble-ament-cppcheck \
    ros-humble-ament-uncrustify \
    ros-humble-ament-lint-cmake \
    ros-humble-ament-cpplint \
    ros-humble-ament-xmllint \
    && rm -rf /var/lib/apt/lists/*

COPY run.sh /run.sh
RUN chmod +x /run.sh

ENTRYPOINT ["/run.sh"]