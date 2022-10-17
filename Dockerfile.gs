FROM osrf/ros:melodic-desktop-full

ADD . /root/Dev/Underwater_2021_2022
#GS receives over 17002
EXPOSE 17002
#GS sends over 17001
EXPOSE 17001
RUN apt-get update && apt-get install -y qt4-default ros-melodic-image-common ros-melodic-cv-bridge xauth
RUN mkdir -p ~/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws; catkin_make'
RUN /bin/bash -c '. ~/catkin_ws/devel/setup.bash'
#CMD [ "/bin/bash" ]