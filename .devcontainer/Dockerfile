# Server RMF with simulation

# add AS builder to have a build enviroment see internet about
ARG rosimage=osrf/ros:jazzy-desktop-full

FROM $rosimage

#install develepment tool
RUN apt update && apt install -y ros-dev-tools vim tmux git \
    && rosdep update
    # && rosdep init  
    # && colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    # && colcon mixin update default

#install Core rmf source
# RUN mkdir -p ~/openRMF_ws/src \
#     && apt update \
#     && apt upgrade -y  && apt install -y wget python3-pip python3-vcstool ros-jazzy-rmw-cyclonedds-cpp \
#     && pip3 install nudged eclipse-zenoh==1.5.0 pycdr2 rosbags --break-system-packages \
#     && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nsource /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
#     && /bin/bash -c "source /opt/ros/jazzy/setup.bash" \
#     && wget https://raw.githubusercontent.com/open-rmf/rmf/jazzy/rmf.repos \
#     && vcs import ~/openRMF_ws/src < rmf.repos \
#     && cd ~/openRMF_ws \
#     && rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr \
#     && /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release" \
#     && rm -rf /var/lib/apt/lists/*

#install Core rmf binary
RUN apt update \
    && apt upgrade -y  && apt install -y wget python3-pip python3-vcstool ros-jazzy-rmw-cyclonedds-cpp \
    && pip3 install nudged eclipse-zenoh==1.5.0 pycdr2 rosbags --break-system-packages \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nsource /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && apt install -y ros-jazzy-rmf-dev \
    && rm -rf /var/lib/apt/lists/*


# # #install simulation code
RUN apt update && apt install -y ros-jazzy-nav2-bringup git \
    && git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations ~/turtlebot3_simulations \
    && echo "export TURTLEBOT3_MODEL=waffle\nexport GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_simulations/turtlebot3_gazebo/models" >> ~/.bashrc \
    && rm -rf /var/lib/apt/lists/*

# # #install zenoh
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null \
    && apt update \
    && apt install -y systemctl zenoh \
    && rm -rf /var/lib/apt/lists/*

#install free fleet and personal code

RUN mkdir -p ~/openRMF_ws/src

RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

RUN --mount=type=ssh git clone -b master --recurse-submodules git@github.com:michbelle/tesi_mag.git ~/openRMF_ws/src

RUN apt update \
    && cd ~/openRMF_ws \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr \
    && /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release" \
    && echo "source ~/openRMF_ws/install/setup.bash" >> ~/.bashrc

#install D-start-trajectory-planner
# RUN cd ~/openRMF_ws/src \
#     && git clone -b jazzy https://github.com/ElettraSciComp/DStar-Trajectory-Planner.git \
#     && apt update \
#     && cd ~/openRMF_ws \
#     && rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr \
#     && /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select dstar_global_planner --cmake-args -DCMAKE_BUILD_TYPE=Release" 

# #install tesi code
# RUN cd ~/openRMF_ws/src \
#     && git clone -b master https://github.com/michbelle/myTesiCode.git \
#     && apt update \
#     && cd ~/openRMF_ws \
#     && rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr \
#     && echo "export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/openRMF_ws/src/myTesiCode/rmf_code/ign_world/elettra/modelOUT/" >> ~/.bashrc \
#     && /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select myCode" \
#     && echo "source ~/openRMF_ws/install/setup.bash" >> ~/.bashrc

# #set simulation robot for nav2
# ENV TURTLEBOT3_MODEL=burger