# razor_imu

Sparkfun 9DoF Razor IMU M0 node for ROS.

![](./pics/1.jpg)

It seems that the magnetic field of Sparkfun firmware is not correct. The orientation estimation is also a tragedy. So, it is recommended to use the [imu-tools](http://wiki.ros.org/imu_tools) package to fuse the angular velocities, accelerations into the orientation by yourself.

> Note: don't fuse the magnetic field.

## 0. Requirement

```bash
sudo apt install ros-<distro>-imu-tools
```

## 1. Firmware

1. Following [\[9DoF Razor IMU M0 Hookup Guide\]](https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide). Start from step: "Installing the 9DoF Razor Arduino Core" to the step: "Select the Board and Serial Port".

2. Download [\[MPU-9250 DMP library\]](https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library) to your Arduino lib path (default to ~/Arduino/libraries/)

3. Use Arduino IDE to burn the firmware [\[9DoF_Razor_M0_Firmware\]](https://github.com/sparkfun/9DOF_Razor_IMU/tree/master/Firmware/_9DoF_Razor_M0_Firmware) into the sensor. (Disable NVRAM_STORAGE in config.h)

    ![](./pics/2.png)

My current specs:
- Ubuntu 16.04 (x86 64 bits)
- Arduino 1.8.11
- Arduino SAMD Boards 1.6.19
- SparkFun SAMD Boards 1.6.1

You might get following error message when using other versions.
> java.io.IOException: Cannot run program "{runtime.tools.bossac-1.7.0.path}/bossac.exe": CreateProcess error=2 ...

## 2. ROS

```
roslaunch razor_imu imu.launch
```

![](./pics/3.png)

![](./pics/4.png)
