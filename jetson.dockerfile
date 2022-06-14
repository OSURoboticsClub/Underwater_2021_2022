FROM osrf/ros:melodic-desktop-full

ADD / /workspaces/Underwater_2021_2022
EXPOSE 17002
EXPOSE 17001
RUN apt-get update && apt-get install -y qt4-default ros-melodic-image-common ros-melodic-cv-bridge xauth
RUN mkdir -p ~/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws; catkin_make'
RUN /bin/bash -c '. ~/catkin_ws/devel/setup.bash'