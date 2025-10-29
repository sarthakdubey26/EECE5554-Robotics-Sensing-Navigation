# Lab 3: VectorNav IMU Driver

## Development Process
This driver was developed with assistance from online resources and AI tools for:
- Understanding ROS2 message structure
- VNYMR parsing syntax
- Quaternion conversion using scipy
- Allan variance computation using allantools

All data collection, analysis, and experimental work was performed individually.

## How to Run
```bash
ros2 launch vn_driver driver.launch.py port:=/dev/ttyUSB1
```
