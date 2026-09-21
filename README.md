1. PX4, at the version that matches your px4_msgs (~30 min, once)
git clone --recursive https://github.com/PX4/PX4-Autopilot ~/PX4-Autopilot
cd ~/PX4-Autopilot && git checkout a64536802b5a5b6ba8fe6ef1b7dcb6a54a0a99ea
git submodule update --init --recursive
bash Tools/setup/ubuntu.sh --no-nuttx      # then log out and back in
make px4_sitl                               # build only

2. The DDS agent
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent ~/xrce && cd ~/xrce
mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig

3. The workspace (the laptop must be on the same network as the Jetson)                               dir -p ~/ros2_ws/src && cd ~/ros2_ws
git clone ark-jetson-orin-2@172.29.71.25:offboard_imav26_test drone_testing                           t clone https://github.com/Legendpar
git clone https://github.com/PX4/px4_msgs.git && (cd px4_msgs && git checkout 86d8239)                 ~/ros2_ws && source /opt/ros/jazzy/
colcon build --packages-select px4_msgs imav_indoor_2026 drone_testing                                
4. Terminal 1: the simulator                                                                           ~/ros2_ws && source install/setup.b
ros2 launch drone_testing sitl.launch.py                                                              it until PX4 prints INFO [commander]watch the down camera athttp://localhost:8080.                                                                                
If PX4 refuses to arm because there's no RC or ground station connected, run this once and relaunch:   ~/PX4-Autopilot/build/px4_sitl_defa0" "NAV_DLL_ACT 0" "COM_RCL_EXCEPT 4";do bin/px4-param set $p; done                                                                         
5. Terminal 2: part 1                                                                                  ~/ros2_ws && source install/setup.b
ros2 run drone_testing mission_fsm_part1 --ros-args -p lateral_source:=flow -p window_marker_id:=2    u should see it climb to 2.10 m, flyUIRED at around 8 m, then CENTRED, then descend slowly and disarm.                                                                            
If it doesn't reach that point, send me:                                                              terminal 2's last ~40 lines
- the output of ros2 topic echo /fmu/out/vehicle_local_position --once | grep -E "xy_valid|dist_bottom
Limits of this test:                                                                                  lateral_source:=flow flies on PX4's S, not the RTAB-Map VIO the real droneuses. The VIO health checks aren't exercised.                                                       Touchdown will probably be confirmedscent check, not by PX4. PX4's defaultMPC_LAND_SPEED is much faster than the 0.10 m/s slow descent, so PX4 won't recognise the touchdown. Expect "Touchdown confirmed by stall
                                                                                                      Part 2 comes after part 1 works. It need at the sim camera topics, androom_mode:=box because the sim has no 2D lidar. The command will start with ros2 launch drone_testing sitl.launch.py x:=-2.0 y:=3.25.
