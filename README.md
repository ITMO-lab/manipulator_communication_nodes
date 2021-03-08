# manipulator_communication_nodes

#### Brief description:

This is **ROS Node** for organizing message transfer (type is std_msgs/String) via **UDP** protocol between **two or more roscore** devices inside one **local network**.



#### Installation:

```bash
pip install -r requirements.txt

chmod +x scripts/*.py
```



#### Run communication node between web server and manipulators:

```bash
rosrun manipulator_communication_nodes web_to_manipulator.py
```



#### Run  communication node between two manipulators:

##### On the first manipulator:

FIRST_MANIPULATOR_RECV_PORT = SECOND_MANIPULATOR_SEND_PORT 

FIRST_MANIPULATOR_SEND_PORT = SECOND_MANIPULATOR_RECV_PORT 

```bash
rosparam set /communication_between_manipulators/remote_manipulator_ip {SECOND_MANIPULATOR_IP}

rosparam set /communication_between_manipulators/recv_port {FIRST_MANIPULATOR_RECV_PORT}

rosparam set /communication_between_manipulators/send_port {FIRST_MANIPULATOR_SEND_PORT}

rosrun manipulator_communication_nodes manipulator_to_manipulator.py
```

##### On the second manipulator:

```bash
rosparam set /communication_between_manipulators/remote_manipulator_ip {FIRST_MANIPULATOR_IP}

rosparam set /communication_between_manipulators/recv_port {SECOND_MANIPULATOR_RECV_PORT}

rosparam set /communication_between_manipulators/send_port {SECOND_MANIPULATOR_SEND_PORT}

rosrun manipulator_communication_nodes manipulator_to_manipulator.py
```