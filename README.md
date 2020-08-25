# add ros2 driver of hps3d160 lidar of hypersen
## test condition 
Note !
* software env: eloquent ubuntu18.04 
* hardware env: rk3399      arm64    
  if on x86,replace libhps3d.so for x86 lib,get by offical hypersen github site.

## get project files and script files

* mkdir ~/hypersen_ws/src
* cd ~/hypersen_ws/src
* git clone https://github.com/youngday/hypersen_lidar_ros2
* cd hypersen_lidar_ros2
* cp scripts/* ../../
* cd ~/hyerson_ws
## compile and run 
* compile
```bash
./compile.sh
```
* run 
```bash
./run.sh
```
* on another consle,run
```bash
./rviz.sh
```
## self yaml files
you can setup param by change yaml file of config.yaml
## reference 
* offical lib
https://github.com/hypersen/HPS3D_SDK 
* fork 
https://github.com/xmba15/hypersen_lidar
* ros2 examples
https://github.com/ros2/examples
## end
good luck ! Thanks for all !

