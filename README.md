# OmniVeyors
Root Repository for the OmniVeyor Robots.

## Robot configuration

### Hardware Setup

- Remove top panel and rear panel.
- Remove UPS and short the two pairs of wires with two connection blocks (+ with +, - with -)
- Install SSD and PCI-e to USB 3.0 card inside the robot computer. Remove the dedicated graphics card to make room for the USB 3.0 card.
- Install Hokuyo UST-10LX LiDAR, Power the LiDAR from terminals (+12V and GND).
- Install Arduino and custom function button. 
  Button connects to Pin D8 and GND. 
  LED on the button (5V) connects to D11 and GND.
- Install wireless charger. 
  The receiver side of the charger has two power lines and three signal lines. Connect Power Red to Battery + (red), Power Black to Battery - (black). 
  Connect Signal Brown (Control) to Arduino D12, Signal Yellow (Fault) to Arduino D13, and Signal Blue (GND) to Arduino GND.
- Check and set servo motor IDs.
  Front Right Steer - 1, Front Right Roll - 2, Front Left Steer - 3, Front Left Roll - 4, ..., Rear Right Roll - 8.
- Install RealSense Cameras. 
  Front Left: D435i. Front Right: T265. Rear Right: D435i (optional). 
  Connect the USB cables to the USB 3.0 card. From left to right the order is: T265, No Connection, Rear D435i, Front D435i.

### Install Linux System

Recommended: `Lubuntu 20.04` or `Kubuntu 20.04`. 

LXQt / KDE + SDDM uses significantly less RAM and CPU resources than the default Gnome desktop in Ubuntu 20.04.

*Specific to the ADLink MXC6300 machine on the robot*
- Pre-installation: Hold `Del` key to enter BIOS. Find Advanced > Advanced Power Management > Restore AC Power Loss. 
Set to `Power On`.

- Find Chipset > Primary IGFX Boot Display. Set to `DP1`. 
Then find Active LFP and set to `Int-LVDS`. 
Set the LCD Panel Type to the desired resolution resolution (`1920x1080 LVDS` for example). 
This option creates a virtual screen for graphical display when the robot is running headless.

- Insert installation media. Press `F4` key to Save and Reset.

- Installation: Hold `Del` to enter BIOS. In the Save and Exit tab, find the installation media and override boot order.

- Enter live session and install system. 
In case the virtual screen is recognized as the main screen and you see a blank desktop, use the following key order to invoke monitor setting:
`Windows`, `m`, `o`, `n`, `up`, `Enter`
In the opened window select `Unified View`.

#### Setup Linux System

- This section assumes you have already installed Lubuntu 20.04, or equivalent flavors with SDDM desktop manager.
- Clone this repository to `~/Dev`, for example:
  ```sh
  mkdir -p ~/Dev/ros_ws/src
  cd ~/Dev/ros_ws/src
  git clone https://github.com/HaoguangYang/omniveyor.git
  ```
- **In the `ros_ws` folder**, run the all-in-one [setup script](https://github.com/HaoguangYang/omniveyor/blob/master/systemInitialSetup.sh):
  ```sh
  cd ..; ./src/omniveyor/systemInitialSetup.sh
  ```
- Sit back and relax!

#### File Structure
```
ros_ws/src/
      |---omniveyor <This repository>
      |   |---omniveyor_mobility (High-level package for mapping, localization, and navigation)
      |   |---spatio_temporal_voxel_layer (MoveBase plugin for spatio voxel cloud generation and obstacle avoidance)
      |   ----nimbro_network (Multi ROS master communication infrastructure)
      |---omniveyor_common (Common definitions, message types, and definitions across hardware and simulator)
      |---omniveyor_hardware
      |   |---pcv_base (Robot base driver)
      |   |---<Placeholder for future hardware>
      |   ----robot_localization (Odometer filtering tools -- EKF, UKF, etc.)
      ----omniveyor_simulator
          |---omniveyor_gazebo_world (Robot simulator main)
          ----realsense_ros_gazebo (Plugin for Realsense simulation)
```

#### Debugging and Troubleshooting
- To debug CAN bus, use candump:
  ```sh
  candump -ax can0
  ```
  The documentation for the iPOS motor controller CAN protocol can be found [HERE](http://www.technosoft.ro/KB/index.php?/getAttach/46/AA-15445/P091.063.CANopen.iPOS.UM.pdf).

- If canbus keeps dropping frames, may need to manually address QoS strategy in `/etc/systemd/system/canbus.service`:
  ```sh
  # change congestion control method for CAN0 to ensure data packets are not stale under congestion.
  ExecStartPre=... && tc qdisc add dev can0 root handle 1: pfifo_head_drop limit 9
  ``` 

- If you are using Gnome desktop manager, comment out the `x11vnc` section of the `./systemInitialSetup.sh` script, and use the inbuilt Vino server instead. The VNC server is started through Settings > Sharing > Desktop Sharing.
  *Optional* enable access of VNC through Windows machines by turning off encryption (**NO sudo**):
  ```sh
  gsettings set org.gnome.Vino require-encryption false
  ```

- If WiFi dongle works below expectation, first make sure it is plugged into a USB2 port instead of a USB3 port. USB3 is known for its [excessive interference within the 2.4GHz frequency range](https://www.usb.org/sites/default/files/327216.pdf). If the port switching still does not solve the issue, you may try to use third-party driver, e.g. [Aircrack-ng's RTL 8188EUS driver](https://github.com/aircrack-ng/rtl8188eus) for better WiFi connection.

- `qterminal` may be swapped by `xterm` in some occasions during the installation of `ros-noetic-fkie-multimaster`. If you feel uncomfortable, do:
  ```sh
  sudo mv /usr/bin/xterm /usr/bin/xterm.old && sudo ln -s /usr/bin/qterminal /usr/bin/xterm
  ```

- **Deprecated, Use with Caution** This repository includes a bootup script, such that the SSH and VNC ports of the cart are mapped to a Virtual Private Server with static IP. To enable the automatic bootup sequence, add the following line in `crontab -e`:
  ```sh
  @reboot sleep 30; cd /home/cartman/Dev/Mobile_Manipulation_Dev/; sh ./onBoot.sh
  ```
  **WARNING:** The program will run 30s after the machine is booted up. Plugging in a joystick will mobilize the robot. Refer to `autorun.py` (under `src/pcv_base/scripts`) for series of commands it executes.

### Compile and run the code: 
```sh
catkin_make -DCMAKE_BUILD_TYPE=Release
```

Then run the launch files in `pcv_base` and `omniveyor_mobility` packages.

**Deprecated** Automatic run procedure is defined in the autorun.py, which structures a move of the robot as a "task", as defined in the `pcv_base/scripts` folder:
```
autorun.py --(imports)--> task --(imports)--> payload
                            |----(calls)--> launch
                            |----(uses)--> resources
```

## Base Station configuration
A base station is the computer that wirelessly connects to all active robots, issuing commands to the robots and monitoring their status. Two approaches of interconnecting multiple robots are provided -- through native ROS setup, and through the `nimbro_network` package.
### Connect to the robot with Remote ROS Master
```sh
export ROS_MASTER_URI=http://$ROBOT_IP:11311
export ROS_IP=$(ip route get 1 | awk '{print $(NF-2);exit}')
```
If `rostopic list` works but no message comes through, check:
- Router is resolving local hostnames (try `ping $ROBOT_HOSTNAME.local` to verify `$ROBOT_HOSTNAME.local` is reachable).
- `ROS_IP` or `ROS_HOSTNAME` setting on the robot side. Refer to [ROS Network Setup](http://wiki.ros.org/ROS/NetworkSetup).

### Connect to the robot with `nimbro_network`
On the base station side:
```sh
roslaunch omniveyor_mobility multi_robot_p2p_onHost.launch
```
On the robot side:
```sh
roslaunch pcv_base multi_robot_p2p.launch
```

### Multi-Robot Working Example:
> #### Network setup
> Host, Robot 1, Robot 2, and Robot 3 are all connected to the same WiFi Access Point. Assume the IP addresses of the robots and hosts are under `192.168.254.96/26`, with the robot addresses start from `192.168.254.101`.
> #### Operation Procedures
> N.B. Replace `192.168.254.10X` with actual node IP
> 1. Power up robots
> #### For new setup:
> 2. ssh into one of the robot: `ssh cartman@192.168.254.10X`
> 3. on **robot**: `cd Dev/omniveyor_newsetup && source devel/setup.bash`
> 4. on **robot**: `roslaunch omniveyor_mobility build_map.launch map_save_period:=30`
> 5. on **host**: Connect the joystick and then `cd omniveyor_ws && source devel/setup.bash`
> 6. on **host**: `roslaunch omniveyor_mobility remote_teleop_on_host.launch node_id:=X`
> 7. on host (optional for visualization): `source devel/setup.bash && export ROS_MASTER_URI=http://192.168.254.10X:11311 && export ROS_IP=$(ip route get 1 | awk '{print $(NF-2);exit}') && roslaunch omniveyor_mobility visualization.launch`
> 8. on **robot**: to enable motors `rostopic pub -1 /control_mode std_msgs/Byte "data: 1"`
>
>    on **host**: teleop the robot with joystick until test area is covered. Exit launch file started in step 4;
>
>    on **robot**: to disable motors `rostopic pub -1 /control_mode std_msgs/Byte "data: 0"`
> 9. copy folder on robot `omniveyor_newsetup/src/omniveyor/omniveyor_mobility/resources/maps/` to peer robots. Use `sftp cartman@192.168.254.10X` and then `put -R` / `get -R` the `maps` folder
> #### For each robot, place robot at mapping origin:
> 10. ssh into robots: `ssh cartman@192.168.254.10X`
> 11. on **robot**: `cd Dev/omniveyor_newsetup && source devel/setup.bash`
> 12. on **robot**: `roslaunch omniveyor_mobility robot_remote_teleop_pos_feedback.launch`
> 13. on **host**: Connect the joystick and then `cd omniveyor_ws && source devel/setup.bash`
> 14. on **host**: `roslaunch omniveyor_mobility multi_robot_remote_teleop_on_host.launch`
>
>     Then, follow instructions on the terminal to enable robots and drive individual robots (Modes 0...2) until positioning covariance converges.
> 15. on host (optional for visualization): `source devel/setup.bash && export ROS_MASTER_URI=http://192.168.254.10X:11311 && export ROS_IP=$(ip route get 1 | awk '{print $(NF-2);exit}') && roslaunch omniveyor_mobility visualization.launch`
> 16. Enter Mode 3 and drive three robots simultaneously in formation.
> #### Cleanup:
> 18. on robot: `Ctrl + C` on all three robots

## Simulator Configuration

A gazebo-based simulator is included in the meta-package. Please refer to the README of [OmniVeyor_Gazebo_World](https://github.com/HaoguangYang/omniveyor_gazebo_world) repository.
