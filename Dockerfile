FROM ros:noetic
ENTRYPOINT [ ]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update \
 && apt install -y --no-install-recommends ros-noetic-tf2-geometry-msgs ros-noetic-vision-msgs \
 ros-noetic-ros-numpy ros-noetic-tf2-sensor-msgs ros-noetic-vision-opencv

RUN apt install -y python3-catkin-tools git

RUN apt install -y python3-catkin-tools git python3-pip
RUN pip install numpy==1.23 transforms3d open3d open3d_ros_helper

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source /opt/ros/noetic/setup.bash

RUN mkdir -p /root/catkin_ws/src/table_plane_extractor

COPY . /root/catkin_ws/src/table_plane_extractor

# clone and build message and service definitions
RUN /bin/bash -c 'cd /root/catkin_ws/src; \
                  git clone https://github.com/ichores-research/table_plane_extractor_msgs.git; \
                  git clone https://github.com/v4r-tuwien/v4r_util.git; \
                  git clone https://github.com/v4r-tuwien/grasping_pipeline_msgs.git; \
                  git clone https://gitlab.informatik.uni-bremen.de/robokudo/robokudo_msgs.git'
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin build && source devel/setup.bash'
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN source /root/catkin_ws/devel/setup.bash

WORKDIR /root
CMD [ "bash", "-c", "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && roslaunch table_plane_extractor table_plane_extractor.launch" ]