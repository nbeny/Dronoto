# Nœuds embarqués Dronoto.
#
# Construction multi-étapes : compilation dans une étape lourde, exécution
# dans une étape minimale. C'est cette image qui ira sur le Jetson, donc sa
# taille compte réellement (cible < 2,5 Go, docs/architecture/13-docker-infra.md).

# ---------------------------------------------------------------- compilation
FROM dronoto-base:latest AS builder

ARG PX4_MSGS_BRANCH=release/1.16

RUN mkdir -p /ws/src && \
    git clone --depth 1 -b "${PX4_MSGS_BRANCH}" \
      https://github.com/PX4/px4_msgs.git /ws/src/px4_msgs

COPY interfaces/ros_msgs/dronoto_msgs /ws/src/dronoto_msgs
COPY drone/ros2/dronoto_core        /ws/src/dronoto_core
COPY drone/ros2/dronoto_interface   /ws/src/dronoto_interface
COPY drone/ros2/dronoto_safety      /ws/src/dronoto_safety
COPY drone/ros2/dronoto_navigation  /ws/src/dronoto_navigation
COPY drone/ros2/dronoto_mission     /ws/src/dronoto_mission
COPY drone/ros2/dronoto_bringup     /ws/src/dronoto_bringup

RUN source /opt/ros/jazzy/setup.bash && \
    cd /ws && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# ----------------------------------------------------------------- exécution
FROM ros:jazzy-ros-core AS runtime

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-tf2-ros \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
      libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ws/install /opt/dronoto

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /opt/dronoto/setup.bash'  >> /root/.bashrc

ENV DRONOTO_INSTALL=/opt/dronoto

# autostart:=false : démarrer un conteneur ne doit jamais faire décoller
# un aéronef.
CMD ["bash", "-lc", \
     "ros2 launch dronoto_bringup bringup_drone.launch.py autostart:=false"]
