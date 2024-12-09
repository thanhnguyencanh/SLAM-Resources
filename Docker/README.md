# SLAM-dockerfile-cheatsheet

## Motivation  
SLAM(simultaneous localization and mapping) have different dependencies for each algorithm. Adjusting these all dependencies to the local computer is very challenging. [Docker](https://www.docker.com/) is a powerful tool, you could solve these problem at once.   

This repository helps you to make Dockerfile easier what you want.  
**Install necessary packages or algorithms using this Cheatsheet!**  😄  

## Table of Contents  
- [Useful Dockerfile commands](#useful-dockerfile-commands)  
- [Useful Docker image](#useful-docker-image)
- [Useful Packages](#useful-packages)  
  - [Ceres-solver](#ceres-solver)  
  - [GTSAM](#gtsam)  
  - [OpenCV](#opencv)
  - [Livox SDK](#livox-sdk)
  - [Livox ros driver](#livox-ros-driver)  
  - [Pangolin](#pangolin)

## Useful Dockerfile commands  
Supported by ChatGPT & [Reference](https://biocorecrg.github.io/CoursesCRG_Containers_Nextflow_May_2021/docker-recipes.html) 
- **`FROM`** : Specifies the base image that will be used for the Docker image.  
- **`WORKDIR`** : Sets the working directory for subsequent commands in the Dockerfile. 
- **`RUN`** : Runs a command inside the Docker image during the build process. This can be used to install packages, run scripts, and perform other setup tasks. 
- **`COPY`** or **`ADD`** :  Copies files from the host machine to the Docker image. This can be used to add application code, configuration files, and other assets to the image.
- **`ENV`** : Sets environment variables inside the Docker image.  
- **`CMD`** or **`ENTRYPOINT`**: Specifies the command that will be executed when the container is started. `CMD` is used to specify a default command, while `ENTRYPOINT` is used to specify a command that should always be run when the container is started.
 

## Useful Docker image  
- [OSRF Docker Images](https://github.com/osrf/docker_images)  

## Useful Packages

### Ceres-solver 
If you want to change the version of Ceres-solver, you just change `1.14.0` version you want.  
- Other versions : https://github.com/ceres-solver/ceres-solver/tags
```
# Install the required packages
RUN apt-get update && apt-get install libatlas-base-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev

# Install ceres-solver
WORKDIR /home/thirdParty
RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.tar.gz
RUN tar zxf 1.14.0.tar.gz
RUN cd ceres-solver-1.14.0
RUN mkdir build && cd build
RUN cmake -DCMAKE_BUILD_TYPE=Release ./ceres-solver-1.14.0 && make -j2 && make install
```

### GTSAM  
If you want to change the version of GTSAM, you just change `4.0.2` version you want.  
- Other versions : https://github.com/borglab/gtsam/tags  
```
# Install GTSAM
WORKDIR /home/thirdParty
RUN wget -O gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
RUN unzip gtsam.zip
WORKDIR /home/thirdParty/gtsam-4.0.2
RUN mkdir build && cd build
RUN cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF .. && make -j2 && make install

# Solving linking issue (optional)
# Fix error related to GTSAM 
# ref: https://github.com/borglab/gtsam/issues/380
export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
```

### OpenCV
If you want to change the version of OpenCV, you just change `3.4.13` version you want.  
- Other versions : https://github.com/opencv/opencv/tags 

```
WORKDIR /home/
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.13.zip
RUN wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.13.zip
RUN unzip opencv.zip && unzip opencv_contrib.zip

WORKDIR /home/opencv_build
RUN cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.4.13/modules -DOPENCV_ENABLE_NONFREE=ON ../opencv-3.4.13
RUN make -j$(nproc) && make install
```
### Livox SDK

More information for dependency at [Livox-SDK2 github](https://github.com/Livox-SDK/Livox-SDK2.git)

```
# Install livox SDK
WORKDIR /root/
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /root/Livox-SDK2
RUN mkdir build
WORKDIR /root/Livox-SDK2/build
RUN cmake .. && make -j2 && make install
```


### Livox ROS driver  

Create a folder for the livox ros driver in `catkin_ws/src` and build it  directly.  
If you want to change the version of livox ros driver, you just change `2.6.0` version you want.  
- Other version : https://github.com/Livox-SDK/livox_ros_driver/tags  


```
WORKDIR /home/catkin_ws/src
RUN wget https://github.com/Livox-SDK/livox_ros_driver/archive/refs/tags/v2.6.0.tar.gz
RUN tar zxf v2.6.0.tar.gz && rm -rf v2.6.0.tar.gz
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace; cd .. && catkin_make'
```

### Pangolin  

```
# Install Pangolin
WORKDIR /home/thirdParty
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
WORKDIR /home/thirdParty/Pangolin/build
RUN cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=../install/ ..
RUN make -j2 && make install
``` 

---
 

# VINS-Mono-docker-tutorial

There is a dockerfile on the [original repository](https://github.com/HKUST-Aerial-Robotics/VINS-Mono), but the explanation is unfriendly for beginners.  

I use another docker image and dataset.  

I've also uploaded a step-by-step tutorial on [YouTube](https://youtu.be/DzbHQzd45TU).  

## Requirements  
- [docker](https://www.docker.com/)
- [nvidia-docker](https://github.com/NVIDIA/nvidia-docker)  
- [terminator](https://www.geeksforgeeks.org/terminator-a-linux-terminal-emulator/) (High recommended)  

## Dataset  

- [OpenLORIS-Scene Dataset](https://lifelong-robotic-vision.github.io/dataset/scene.html)  

> If you want to use this dataset, you need to fix some bag files.  
> Please refer to [this issue](https://github.com/lifelong-robotic-vision/OpenLORIS-Scene/issues/15).  

**When you make dataset, you have to know where it is (Absolute path will be used).**  

```
@inproceedings{shi2019openlorisscene,
    title={Are We Ready for Service Robots? The {OpenLORIS-Scene} Datasets for Lifelong {SLAM}},
    author={Xuesong Shi and Dongjiang Li and Pengpeng Zhao and Qinbin Tian and Yuxin Tian and Qiwei Long and Chunhao Zhu and Jingwei Song and Fei Qiao and Le Song and Yangquan Guo and Zhigang Wang and Yimin Zhang and Baoxing Qin and Wei Yang and Fangshi Wang and Rosa H. M. Chan and Qi She},
    booktitle={2020 International Conference on Robotics and Automation (ICRA)},
    year={2020},
    pages={3139-3145},
}
```

## Clone VINS-Mono repository  

You could use original [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) repository,  
but I make my own [VINS-MONO-studying](https://github.com/Taeyoung96/VINS-MONO-studying).  

I added config file for OpenLORIS-Scene dataset, and change the file format of `vins_result_loop.csv` like **[TUM RGB-D dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats)**. 


```
git clone https://github.com/Taeyoung96/VINS-MONO-studying.git  
```
**Also, you should know where it is (Absolute path will be used).**  

## Pull the docker image  

I used [icra2018/vins-mono](https://hub.docker.com/r/icra2018/vins-mono) docker image.  

```
docker pull icra2018/vins-mono
```

When the Docker image pull is completed, the output is as follows when input as shown below.  

```
docker images
```
Output:
```
REPOSITORY           TAG                    IMAGE ID       CREATED        SIZE
icra2018/vins-mono   latest                 1dc54987f8fe   2 years ago    4.95GB
```

## Make your own container  

You have to use your own absolute path, when you run the command below.  

```
nvidia-docker run -it -p 8888:8888 -e DISPLAY -w /home/jovyan/catkin_ws/src -v /tmp/.X11-unix:/tmp/.X11-unix \
-v [Absolute path of Dataset]:/dataset \
-v [Absolute path of VINS-Mono repository]:/home/jovyan/catkin_ws/src/ \
--name vins-mono icra2018/vins-mono:latest /bin/bash
```

When you are done, the terminal will be changed like below.  

```
jovyan@e9663ba248d6:~/catkin_ws/src$ 
```
Type ls and you should see the VINS-Mono folder.  
```
jovyan@e9663ba248d6:~/catkin_ws/src$ ls
VINS-Mono
```

Change the directory and build it.  
```
jovyan@e9663ba248d6:~/catkin_ws/src$ cd ..
jovyan@e9663ba248d6:~/catkin_ws$ catkin_make 
```

When you are done you would see the outputs below.  
```
[  3%] Built target benchmark_publisher
[ 23%] Built target camera_model
[ 44%] Built target Calibration
[ 47%] Built target ar_demo_node
[ 53%] Built target feature_tracker
[ 78%] Built target vins_estimator
[100%] Built target pose_graph
...
```

Then run roscore.  

```
jovyan@e9663ba248d6:~/catkin_ws$ roscore  
```

Now you need to connect to the Docker container using the other 3 terminals.  

- 2nd Terminal : launch vins_estimator     
- 3rd Terminal : launch Rviz  
- 4nd Terminal : play rosbag file  

Before connecting other terminals to the container, type the command below to use the GUI.
```
xhost +local:docker
```

2nd Terminal :  
```
docker exec -it -w /home/jovyan/catkin_ws/ vins-mono /bin/bash
```
When connecting to a docker container,  
The launch file is in [VINS-MONO-studying](https://github.com/Taeyoung96/VINS-MONO-studying).  
```
roslaunch vins_estimator realsense_color_cafe.launch
```

3rd Terminal :  
```
docker exec -it -w /home/jovyan/catkin_ws/ vins-mono /bin/bash
```
When connecting to a docker container,  
```
roslaunch vins_estimator vins_rviz.launch
```

4nd Terminal :   
```
docker exec -it -w /dataset vins-mono /bin/bash  
```
When connecting to a docker container,  
```
rosbag play cafe1-1_with_imu.bag
```
This bag file was created by myself using the OpenLORIS-Scene dataset.  

## Result  

<p align="center"><img src="https://user-images.githubusercontent.com/41863759/158924202-3696a69c-bffe-44ee-86fd-e8f532f9b0bb.JPG" width = "600" ></p>  

## License  

Same License with original VINS-MONO.  

The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong QIN <tong.qinATconnect.ust.hk> or Peiliang LI <pliapATconnect.ust.hk>.

For commercial inquiries, please contact Shaojie SHEN <eeshaojieATust.hk>
