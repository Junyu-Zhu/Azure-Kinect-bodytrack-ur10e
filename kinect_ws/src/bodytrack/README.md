## Prerequisites

* Python 3
* opencv-python
* ur_rtde
* pillow
* pyyaml
* rosmsg, rospy

Azure Kienct DK official documents [here](https://docs.microsoft.com/zh-cn/azure/kinect-dk/)，about sensor SDK and bodytrack SDK。ur_rtde official documents [here](https://sdurobotics.gitlab.io/ur_rtde/introduction/introduction.html)，about Installation and Introduction to ur robotarm and robotiq gripper。In `scripts`,`config.py, _k4a.py, _k4abt.py, _k4abtTypes.py, _k4atypes.py, kinectBodyTracker.py, pyKinectAzure.py` are about kinect sensor and bodytrack, `robotiq_gripper_control.py, robotiq_preamble.py` are about robotiq gripper，`ros_publisher.py, ros_subscriber.py` start bodytrack, vedio and ur robotarm and gripper.

First, cd /Kinect_ws compile work space `catkin_make`

start bodytrack and show vedio

```bash
source activate your virtual environment (optional)
source devel/setup.bash
rosrun bodytrack ros_publisher.py
```

start ur10e
```bash
source activate your virtual environment (optional)
source devel/setup.bash
rosrun bodytrack ros_subscriber.py
```
