FROM osrf/ros:melodic-desktop-full

RUN apt-get update && apt-get install -y qt4-default ros-melodic-image-common ros-melodic-cv-bridge
RUN mkdir -p ~/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws; catkin_make'
RUN /bin/bash -c '. ~/catkin_ws/devel/setup.bash'