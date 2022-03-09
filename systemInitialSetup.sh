#!/bin/bash 

machineIs=$(whiptail --checklist --separate-output --title "Device Type" "Choose the main purpose of the device" 10 78 5 \
            "1" "On-board computer of the robot." off \
            "2" "Base station for task dispatching and management." off \
            "3" "Workstation for simulating robot tasks." off 3>&1 1>&2 2>&3)

# update repos
sudo apt update

# setup ssh
sudo apt install -y ssh
sudo systemctl enable ssh

if [[ "${machineIs}" == *"1"* ]]; then
    # setup canbus
    sudo apt install -y can-utils
    sudo cp ./src/omniveyor_hardware/pcv_base/resources/setup/can.conf /etc/modules-load.d/
    sudo chmod +x /etc/modules-load.d/can.conf
    sudo cp ./src/omniveyor_hardware/pcv_base/resources/setup/canbus.service /etc/systemd/system/
    sudo chmod +x /etc/systemd/system/canbus.service
    sudo systemctl enable canbus

    # setup x11vnc
    sudo apt install -y x11vnc
    sudo cp ./src/omniveyor_hardware/pcv_base/resources/setup/x11vnc.service /etc/systemd/system/
    PASSWORD=$(whiptail --passwordbox "please enter your VNC login password" 8 78 --title "VNC Password" 3>&1 1>&2 2>&3)
    sudo sed -i "s/-password.*-shared/-password $PASSWORD -shared/g" \
        /etc/systemd/system/x11vnc.service
    sudo chmod +x /etc/systemd/system/x11vnc.service
    sudo systemctl enable x11vnc

    # setup Robot ID
    nodeNumber=$(hostname | tr -dc '0-9' | sed 's/^0*//')
    echo "export NODE_NO=$nodeNumber" | sudo tee /etc/profile.d/robot_name.sh

    # setup limits
    echo "# increase message queue size
    fs.mqueue.msg_max = 100" | sudo tee -a /etc/sysctl.conf
    USERNAME=$(whoami)
    sudo sed "$i$USERNAME         hard    memlock         524288" /etc/security/limits.conf
    sudo sed "$i$USERNAME         soft    memlock         524288" /etc/security/limits.conf
    sudo sed "$i$USERNAME         hard    priority        85" /etc/security/limits.conf
    sudo sed "$i$USERNAME         hard    rtprio          85" /etc/security/limits.conf
    sudo sed "$i$USERNAME         soft    rtprio          85" /etc/security/limits.conf

    # setup network boradcasting for multi-master ROS1 system
    #echo "# enable ipv4 broadcast response for ROS1 multimaster
    #net.ipv4.icmp_echo_ignore_broadcasts=0" | sudo tee -a /etc/sysctl.conf

    # restart sysctl to take effect
    sudo sysctl -p
fi

# install dependent system packages
# system utilities
sudo apt install -y net-tools curl
# system build & run dependencies
sudo apt install -y build-essential libopenvdb-dev python3-pip python-is-python3
sudo -H pip3 install numpy pymysql opencv-contrib-python pyrealsense2
if [[ "${machineIs}" == *"1"* ]]; then
    # hardware drivers: Realsense cameras
    # refer to: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
    key=$(curl -s https://raw.githubusercontent.com/IntelRealSense/librealsense/master/doc/distribution_linux.md |\
        grep "keyserver.ubuntu.com" |\
        sed 's/.*--recv-key \([^ ]*\).`/\1/')
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key $key ||\
        sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key $key
    sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
    sudo apt install -y librealsense2-dkms
    sudo apt install -y librealsense2-utils
    sudo apt install -y librealsense2-dev
fi

# install ROS
# refer to: http://wiki.ros.org/noetic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "# setup ROS environment
export ROS_HOSTNAME=$(hostname).local
source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-rosdep
sudo rosdep init
rosdep update

# install dependent ROS packages
sudo apt install -y ros-noetic-amcl ros-noetic-move-base ros-noetic-gmapping \
            ros-noetic-teb-local-planner ros-noetic-urg-node ros-noetic-map-server \
            ros-noetic-global-planner ros-noetic-rtabmap ros-noetic-realsense2-camera\
            ros-noetic-cv-bridge ros-noetic-geographic-msgs ros-noetic-ros-numpy \
            ros-noetic-rosserial-python ros-noetic-imu-filter-madgwick ros-noetic-smacha \
            ros-noetic-joy

# search and apply remaining upgrades
sudo apt update
sudo apt upgrade

if [[ "${machineIs}" == *"1"* ]]; then
    # optional: setup Arduino IO
    mkdir ./Arduino
    VER=$(curl -L -s https://api.github.com/repos/arduino/Arduino/releases/latest | \
        grep --regexp='Arduino/releases/tag/[0-9][0-9]*\.[0-9][0-9]*\.[0-9][0-9]*.*' | tail -1 | \
        cut -d '/' -f 8 | cut -d '"' -f 1)
    wget -c "https://downloads.arduino.cc/arduino-${VER}-linux64.tar.xz" -O - | tar -xJvf -C ./Arduino --strip-components=1
    cd ./Arduino
    chmod +x ./install.sh
    sudo ./install.sh
    ./arduino-linux-setup.sh `whoami`
    cd ..
fi

# populate repo
wstool init src https://raw.githubusercontent.com/HaoguangYang/omniveyor/master/omniveyor.rosinstall
# dirty-fix: nimbro_network cmake file formating
sed -i 's/cmake_minimum_required(VERSION 3.10)/cmake_minimum_required(VERSION 3.10.0)/g' \
    ./src/omniveyor/nimbro_network/nimbro_network/CMakeLists.txt

if [[ "${machineIs}" == *"1"* ]] || [[ "${machineIs}" == *"3"* ]]; then
    wstool merge -t src https://raw.githubusercontent.com/HaoguangYang/omniveyor_hardware/master/omniveyor_hardware.rosinstall
fi

if [[ "${machineIs}" == *"3"* ]]; then
    wstool merge -t src https://raw.githubusercontent.com/HaoguangYang/omniveyor_gazebo_world/master/omniveyor_gazebo_world.rosinstall
fi

# now it's safe to build.
catkin_make -DCMAKE_BUILD_TYPE=Release

if [[ "${machineIs}" == *"1"* ]] || [[ "${machineIs}" == *"3"* ]]; then
    # arduino-based payload interface
    if compgen -G "/dev/ttyACM*" > /dev/null; then
        PORT=`ls /dev/ttyACM* | head -1`
        arduino --upload --board arduino:avr:uno --port /dev/ttyACM ${PORT} \
            ./src/omniveyor_hardware/pcv_base/resources/payload-Generic/payload-Generic.ino
    fi
fi

# optional: setup autolaunch

