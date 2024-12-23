Standalone Mujoco Simulator

based on Mujoco 3.2.7, for ubuntu


glfw3 required.

```sh
~$ sudo apt install glfw3-dev
```

How to install 
```sh
~$ git clone -b new https://github.com/saga0619/mujoco_ros_sim
~$ cd mujoco_ros_sim
~/mujoco_ros_sim$ mkdir build && cd build
~/mujoco_ros_sim/build$ cmake .. && make
~/mujoco_ros_sim/build& ./simulate
```

