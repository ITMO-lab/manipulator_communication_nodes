# manipulator_communication_nodes

#### Install:

pip install -r requirements.txt

chmod +x scripts/*.py

#### Usage:

rosrun manipulator_communication_nodes web_to_manipulator.py

rosparam set /communication_between_manipulators/remote_manipulator_ip {REMOTE_MANIPULATOR_IP}

rosrun manipulator_communication_nodes manipulator_to_manipulator.py

