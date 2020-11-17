FROM ros:kinetic-perception

ENV CATKIN_WS=/root/catkin_ws RVIO_ROOT=/root/catkin_ws/src/r-vio

COPY ./ $RVIO_ROOT

# Build r-vio
WORKDIR $CATKIN_WS
COPY ./scripts/ $CATKIN_WS
RUN ["/bin/bash", "-c", "chmod +x build.sh && ./build.sh && chmod +x modify_entrypoint.sh && ./modify_entrypoint.sh"]
