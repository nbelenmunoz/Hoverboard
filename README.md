# Inverted pendulum hoverboard control

This is a project to control a hoverboard using two inverted pendulum to keep the hoverboard in a stable position.

The pendulums are controlled using a Raspberri pi and a L293D motor driver.

## Hardware

* 2x Inverted pendulum
* 1x Hoverboard
* 2x 12V linear actuator
* 2x IMU
* 2x linear encoder

## Software

* Python
* ROS

## Control options

* PID ( "Simple" solution )
  * Modern PID
* Deep learning ( Desired solution )
  * Double deep Q learning
    * Reinforcement learning

## Run on terminal before use (Raspberry Pi Zero 2 W):

  ```bash
  pip install mpu9250-jmdev
  sudo pigpiod
  ```

## Python dependencies:
  * gpiozero (https://gpiozero.readthedocs.io/en/stable/)
  * mpu9250-jmdev (https://pypi.org/project/mpu9250-jmdev/)

## Tested with:
  * Raspberry Pi Zero 2 W
  * Elecrow SM9250MPU 9DOF IMU
  * OKYSTAR TB6612FNG motor driver
  * Raspberry OS 11 (Linux kernel 5.10.103)
  * Thonny 3.3.14
  * Python 3.9.2
  * gpiozero 1.6.2
  * mpu9250-jmdev 1.0.12

## Files:
  * inverted_pendulum.py  |  main implementation
  * datalog.log           |  log from a long test run (shown in the video starting from 10:57)
  * log_to_graph.py       |  create matplotlib graph using datalog.dat (requires matplotlib)
  * log_to_video.py       |  create matplotlib graph video file using datalog.dat (requires matplotlib and ffmpeg)
  * test_*.py             |  test individual parts


## Links

* [Inverse Pendulum — Deep Reinforcement Learning](https://medium.com/mlearning-ai/inverse-pendulum-deep-reinforcement-learning-a22689e14e34)

* [MODELADO, SIMULACIÓN Y CONTROL DE UN PÉNDULOINVERTIDO USANDO COMPONENTES ANÁLOGOS SIMPLES](https://www.studocu.com/latam/document/universidad-autonoma-de-occidente/control-1/pendulo-invertido/10076977)

* [Drive Folder](https://drive.google.com/drive/u/1/folders/0ACoW_Sr1GQcIUk9PVA)