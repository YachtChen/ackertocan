# Ackertocan 
This package contains 3 nodes: 
- Ackerman.py: A test node that publish Ackermann steering message
- conversion.py: The main node that encoded the message   
- CAN.py: A test node that receive CAN message
## Resources needed
2 message packages are required
- Ackermann msg: https://github.com/rosdrivers/ackermann_msgs
- CAN msg: 
https://github.com/UOAFSAE/autonomous/tree/main/src/moa/moa_msgs

## Possible errors
1. setup.py install is deprecated
Downgrade the setup tool to 58.2.0
>pip install setuptools==58.2.0
