# Hypersen HPS-3D160 Solid-State Lidar ROS package #
## Preparation ##
- Check if the device is recognized correctly or not
```bash
    ls /dev/tty* | grep ACM0
    # You should see /dev/ttyACM0 if the device has been recognized
```
- Give permission to access the device
```bash
    sudo chmod 666 /dev/ttyACM0
    # After that cat /dev/ttyACM0 should give you a blank output
```
