# visual-grasping

## Set up RealSense in ROS
```bash
$ sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
$ sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
$ sudo apt-get install librealsense2-dkms
$ sudo apt-get install librealsense2-utils
$ sudo apt-get install librealsense2-dev
$ cd ~/catkin_ws/src/
$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ cd realsense-ros/
$ git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
$ cd ..
$ catkin_init_workspace
$ cd ..
$ catkin_make clean
$ catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
$ catkin_make install
$ source devel/setup.bash
```

## Set up Kinova

### Set up Anaconda environment
Please use Anaconda to configure the environment (make your life easy):
```bash
$ conda create -n kinova python=3.5
$ conda activate kinova
$ pip install -U rospkg # This is for use Python3 and ROS libraries together
$ conda install -c anaconda opencv
```

Download Kortex API: [kortex_api](https://artifactory.kinovaapps.com/artifactory/generic-local-public/kortex/API/2.0.0/kortex_api_2.0.0.zip)  

Extract the ZIP file, and you will find the .whl file in the ```python``` folder.

```bash
$ python -m pip install <whl relative fullpath name>.whl
```
### Usage of code
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/boshenniu/visual-grasping.git
$ cd ~/catkin_ws
$ source /opt/ros/kinetic/setup.bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch ar_track_alvar boshen_test.launch
$ python src/visual-grasping/camera_calibration/calibration.py # in another terminal
```
