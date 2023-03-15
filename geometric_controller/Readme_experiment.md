# Experiment steps of using mavros_controller
## 1. Packages
- mavros_controllers [https://github.com/ZhongmouLi/mavros_controllers.git](https://github.com/ZhongmouLi/mavros_controllers.git)
- vicon_bridge [https://github.com/ZhongmouLi/vicon_bridge.git](https://github.com/ZhongmouLi/vicon_bridge.git)
- mocap_to_mavros
- mavros

### 1.1 mavros_controllers
In fact, we run the package **geometric_controller** of mavros_controllers to control a quadrotor.

The node **geometric_controller_node** in the package **geometric_controller** operates in two steps:
1. receive position, or position + vel, commands from ```/reference/setpoint``` 
2. TODO
    - map of functions
    - use vicon infor instead of local_position
3. send control input bodyrate to ```/mavros/setpoint_raw/attitude```

The node **geometric_controller_takeoff_node** publishes a position point to ```trajectory/setpoint```.
```shell
  <node pkg="geometric_controller" type="geometric_controller_takeoff_node" name="geometric_takeoff" output="screen">
  </node>  
```

### 1.2 vicon_bridge
It gets drone pose information from a VICON system and publish that into a ROS topic. It needs to be configured with information:
- ip address of wifi router
- ip address of VICON


### 1.3 mocap_to_mavros
It is suggested by PX4 that pose information form mocap should be sent by an onboard computer.

> It is highly recommended that you send motion capture data via an onboard computer (e.g Raspberry Pi, ODroid, etc.) for reliable communications. The onboard computer can be connected to the motion capture computer through WiFi, which offers reliable, high-bandwidth connection.


## Steps to run experiments

### 1. configure drone sensors

### 2. drone synch drone time
check the time of drone and base station
```shell
  date
```

### 3. base station runs **vicon_bridge** to get VICON information published to ROS
```shell
  roslaunch drone_experiment_tools drone_ex_F46_setup.launch
```

Note: in drone-ex_F46_setup.launch, modification may be needed
when robot name in VICON, like DRONE_z, is changed 

```shell
  <remap from="/vicon/DRONE_z/DRONE_z" to="/vicon/drone" /> 
```
### 4. drone runs onboard node to start mavros and mocap_to_mavros
ssh to log into drone
```shell
ssh drone1@drone1
```
Starting mavros and mocap_to_mavros by
```shell
roslaunch drone_experiment_tools drone_onboard.launch
```
or

```shell
roslaunch drone_experiment_tools drone_onboard_2.launch
```

### 5 record
```shell
roslaunch drone_experiment_tools drone_ex_record
```

### 6. base station run mavros_controller
```shell
roslaunch geometric_controller ex_geometric_controller
```

