# apriltaglocal\_deploy

The goal of this project is to quickly build an accurate (calibration-free) localization system in indoor medium & large-scale scenarios.

In the project, we define the 0th Tag as the base tag for each Tag family. The output pose is related to the base tag. Thus, we use the method of common view to construct a series of April tag pose associations. After that, we will use the available tag in each frame to calculate the global camera pose and finish the localization task.

## Frame work

<img width="1212" alt="ExtendAprilTag 🏖" src="ExtendAprilTag 🏖.png">

The whole system consists of two parts: the Tag Generator module & the Localization module.

* The Tag Generator module will get tag information (Tag Family, Tag Size / Tag print PPI, Extend Dot size/position) from the YAML file and generate a <Ready to Print> Tag image with Extend dot.

* The localization module will get information (Tag info, Camera Intrinsic parameter/Distortion coefficient, Tag pre-calibration pose) from the YAML file and do the localization task. Finally, the output should be the camera pose related to the base tag.

![framework_2.png](framework_2.png)

## Environment

### Hardware & OS

Raspberry pi 3, 3b, 3b+, 4, Zero 2W

Raspberry pi Os 32bit LTS With Lagency Camera API

### DLL Required

- AprilTag
- Yaml-cpp
- Glog
- Ceres
- Opencv4
- Eigen3

#### Install dll with `apt-get`

```shell
sudo apt install -y libyaml-cpp-dev libgoogle-glog-dev libeigen3-dev libceres-dev
sudo apt install -y libopencv-dev git cmake
mkdir 3rdparty && cd 3rdparty
git clone https://github.com/AprilRobotics/apriltag && cd apriltag
cmake -B build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build build --target install
```

### Build Instruction (or direct run exe file with dll)

```shell
mkdir build && cd build
cmake ..
make
```

## Yaml Definition

   

| Name             | Discription                                                                                                                                                                                                                                                               | Module                    |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------- |
| TagFamily        | The TagFamily used by both localization module and generator                                                                                                                                                                                                              | localization &  generator |
| ImagePath        | The raw tag image path                                                                                                                                                                                                                                                    | generator                 |
| ImageCount       | The raw tag input image path                                                                                                                                                                                                                                              | generator                 |
| GenTags          | The tag id used by tag generator                                                                                                                                                                                                                                          | generator                 |
| GenPath          | The extend tag output image path                                                                                                                                                                                                                                          | generator                 |
| Tag ID as key    | File : The raw tag input image name<br/>Size : The size of tag edge (mm)<br/>Canvas : The size of canvas (mm) [W, H]<br/>Extand_Dot : The position and size of extend dot (mm) [X, Y, R]<br/>Pixpermm : PPI for image<br/>Extrinsic : Pre-calibrate pose for tag (option) | localization &  generator |
| ImageTopic       | The input image topic                                                                                                                                                                                                                                                     | localization (Ros only)   |
| ImageSize        | The input image size [W, H]                                                                                                                                                                                                                                               | localization              |
| ImageIntrinsic   | Intrinsic parameter of the camera  <br/>[$f_x$, 0,  $c_x$, 0,  $f_y$,  $c_y$, 0, 0, 1]                                                                                                                                                                                    | localization              |
| ImageDistrotion  | Distrotion parameter of the camera  <br/>[$k_1$ $k_2$ $p_1$ $p_2$ $(k_3)$]                                                                                                                                                                                                | localization              |
| T_b_c            | deprecate /                                                                                                                                                                                                                                                               | /                         |
| PosePublishTopic | The output camera pose topic                                                                                                                                                                                                                                              | localization              |
| DebugPlot        | Choose whether to plot the debug image (close to get better performance)                                                                                                                                                                                                  | localization(Pi only)     |

## How to run demo

`./localization`

or manually set config path by `./localization ${path_to_yaml_file}`

## Sample

![sample.gif](sample.gif)

## How to use it

### Directly add code onto the base

You can add code at the position we use cout to print the position to the terminal and add the user defined behavior code at that postion.

### Use it as a library

TBD

## Trouble shooting

### Check whether camera is working

Camera Type We test v1 & v2 raspberry camera. But it should work with all camera which support Lagency Camera API on Raspberry Pi

* Plugin camera

* Open Lagency Camera interface with rasp `raspi-config`

* Reboot

* Install opencv python to run test script `sudo apt install -y python3-opencv`

* Using test script `cameratest.py` to test whether camera workwell

### Check whether the dll is working

Run the release exe file after doing  **Install dll with `apt-get`**

### Simulate Test with video

### Camera calibration

You need to calibrate the camera manually. Ros `camera calibration` maybe a good choice. 

You need to get  $f_x$  $f_y$  $c_x$ $c_y$ for intrinsic and  $k_1$  $k_2$  $p_1$  $p_2$ $(k_3)$  for distortion coefficients. Tipically you should not use camera with high distortion.

### Output check

See the Image out check whether everything goes well and check whether the xyz on the left top corner make sence unit is (m)
