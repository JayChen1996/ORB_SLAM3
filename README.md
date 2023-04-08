# ORB-SLAM3

### V1.0, December 22th, 2021--2021年12月22日
**Authors:** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).--**作者** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).

The [Changelog](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Changelog.md) describes the features of each version.[修改日志](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Changelog.md) 描述了每个版本的特性.

ORB-SLAM3 is the first real-time SLAM library able to perform **Visual, Visual-Inertial and Multi-Map SLAM** with **monocular, stereo and RGB-D** cameras, using **pin-hole and fisheye** lens models. In all sensor configurations, ORB-SLAM3 is as robust as the best systems available in the literature, and significantly more accurate. ORB-SLAM3是首个**小孔和鱼眼**透镜模型的可以使用**单目，双目和RGB-D**执行**视觉，视觉-惯性和多地图SLAM**的实时SLAM库

We provide examples to run ORB-SLAM3 in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) using stereo or monocular, with or without IMU, and in the [TUM-VI dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) using fisheye stereo or monocular, with or without IMU. Videos of some example executions can be found at [ORB-SLAM3 channel](https://www.youtube.com/channel/UCXVt-kXG6T95Z4tVaYlU80Q).ORB-SLAM3提供了[EuRoC数据集](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (使用双目或单目，可提供IMU)和[TUM-VI数据集](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)(使用鱼眼双目或单目，可提供IMU)。一些运行视频可以查看Youtube的[ORB-SLAM3频道](https://www.youtube.com/channel/UCXVt-kXG6T95Z4tVaYlU80Q)。

This software is based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) developed by [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)).ORB-SLAM3是基于[Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) 和 [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))所开发的[ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)进行的。

<a href="https://youtu.be/HyLNq-98LRo" target="_blank"><img src="https://img.youtube.com/vi/HyLNq-98LRo/0.jpg" 
alt="ORB-SLAM3" width="240" height="180" border="10" /></a>

### Related Publications:相关文献

[ORB-SLAM3] Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós, **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**, *IEEE Transactions on Robotics 37(6):1874-1890, Dec. 2021*. **[PDF](https://arxiv.org/abs/2007.11898)**.

[IMU-Initialization] Carlos Campos, J. M. M. Montiel and Juan D. Tardós, **Inertial-Only Optimization for Visual-Inertial Initialization**, *ICRA 2020*. **[PDF](https://arxiv.org/pdf/2003.05766.pdf)**

[ORBSLAM-Atlas] Richard Elvira, J. M. M. Montiel and Juan D. Tardós, **ORBSLAM-Atlas: a robust and accurate multi-map system**, *IROS 2019*. **[PDF](https://arxiv.org/pdf/1908.11585.pdf)**.

[ORBSLAM-VI] Raúl Mur-Artal, and Juan D. Tardós, **Visual-inertial monocular SLAM with map reuse**, IEEE Robotics and Automation Letters, vol. 2 no. 2, pp. 796-803, 2017. **[PDF](https://arxiv.org/pdf/1610.05949.pdf)**. 

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://arxiv.org/pdf/1610.06475.pdf)**.

[Monocular] Raúl Mur-Artal, José M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](https://arxiv.org/pdf/1502.00956.pdf)**.

[DBoW2 Place Recognition] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp. 1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License许可证

ORB-SLAM3 is released under [GPLv3 license](https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md).ORB-SLAM3在[GPLv3协议](https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE)许可下发布。 对于所有的代码/库依赖清单(以及相应的许可证), 请查看[Dependencies.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM3 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.如需商业目的的闭源版本，请联系作者： orbslam (at) unizar (dot) es

If you use ORB-SLAM3 in an academic work, please cite:如果你的学术工作使用了ORB-SLAM3，请引用：
  
    @article{ORBSLAM3_TRO,
      title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
               and Multi-Map {SLAM}},
      author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
              Jos\'e M. M. AND Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics}, 
      volume={37},
      number={6},
      pages={1874-1890},
      year={2021}
     }

# 2. Prerequisites先决条件
We have tested the library in **Ubuntu 16.04** and **18.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.我们已经在**Ubuntu16.04**和**18.04**上进行了测试，但在其他平台上进行编译也很容易。一台高配置电脑(如i7)可以保证实时性能并提供更加稳定和准确的结果。

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.我们使用了C++11中的thread和chrono函数特性。

## PangolinPangolin（一个可视化界面库，可以理解成简单的Qt）
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.我们使用 [Pangolin](https://github.com/stevenlovegrove/Pangolin) 进行可视化和用户界面开发。下载和安装指令见：https://github.com/stevenlovegrove/Pangolin

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0 and 4.4.0**.我们使用[OpenCV](http://opencv.org) 以操控图片和特征。下载和安装指令见：http://opencv.org 。**要求至少3.0版本，在OpenCV 3.2.0和4.4.0上测试通过**

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.g2o(见下方)需要使用这个库。下载和安装指令见： http://eigen.tuxfamily.org 。**最低要求版本3.1.0**

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.我们修改了[DBoW2](https://github.com/dorian3d/DBoW2)和 [g2o](https://github.com/RainerKuemmerle/g2o) 库的版本以执行场景识别和非线性优化。两个被修改过的库(基于BSD协议)包含在*Thirdparty*目录中

## Python
Required to calculate the alignment of the trajectory with the ground truth. **Required Numpy module**.用于计算轨迹和真实值的对齐度。**需要Numpy模块**

* (win) http://www.python.org/downloads/windows
* (deb) `sudo apt install libpython2.7-dev`
* (mac) preinstalled with osx

## ROS (optional)

We provide some examples to process input of a monocular, monocular-inertial, stereo, stereo-inertial or RGB-D camera using ROS. Building these examples is optional. These have been tested with ROS Melodic under Ubuntu 18.04.我们提供了一些在ROS上处理单目、单目惯性、双目、双目惯性和RGB-D相机输入的例子。可以选择是否构建这些例子。已经在Ubuntu 18.04下的ROS Melodic下进行测试。

# 3. Building ORB-SLAM3 library and examples构建ORB-SLAM3库和示例

Clone the repository:
```
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM3*. Please make sure you have installed all required dependencies (see section 2).我们提供了脚本`build.sh`以构建*Thirdparty*库和*ORB-SLAM3*。
Execute:
```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM3.so**  at *lib* folder and the executables in *Examples* folder.这将会在*lib*目录下创建**libORB_SLAM3**，在*Examples*目录下面创建可执行文件。

# 4. Running ORB-SLAM3 with your camera用你的相机运行ORB-SLAM3

Directory `Examples` contains several demo programs and calibration files to run ORB-SLAM3 in all sensor configurations with Intel Realsense cameras T265 and D435i. The steps needed to use your own camera are: Example目录包含了一些demo程序和标定文件用于运行ORB-SLAM3，可以在包括T265和D435i等传感器配置下运行。使用自己的相机需要执行下面的步骤：

1. Calibrate your camera following `Calibration_Tutorial.pdf` and write your calibration file `your_camera.yaml`标定相机，按照`Calibration_Tutorial.pdf`并写下标定文件`your_camera.yaml`

2. Modify one of the provided demos to suit your specific camera model, and build it修改其中一个demo适应你的相机模式

3. Connect the camera to your computer using USB3 or the appropriate interface用USB3连接你的相机

4. Run ORB-SLAM3. For example, for our D435i camera, we would execute:运行ORB-SLAM3，下面是使用D435i运行的示例

```
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D435i.yaml
```

# 5. EuRoC Examples--EuRoC示例
[EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) was recorded with two pinhole cameras and an inertial sensor. We provide an example script to launch EuRoC sequences in all the sensor configurations.[EuRoC数据集](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) 是采用两个小孔相机和一个惯性传感器记录的。我们提供了一个示例脚本在所有传感器配置下启动EuRoC序列。

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets下载一个序列(ASL格式，下载下来就是zip压缩包)，地址：http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Open the script "euroc_examples.sh" in the root of the project. Change **pathDatasetEuroc** variable to point to the directory where the dataset has been uncompressed. 打开项目根目录下的脚本"euroc_examples.sh"。修改**pathDatasetEuroc**变量为指向下载数据集解压的目录。

3. Execute the following script to process all the sequences with all sensor configurations:执行下面的脚本以在所有传感器配置下处理所有序列
```
./euroc_examples
```

## Evaluation评估
EuRoC provides ground truth for each sequence in the IMU body reference. As pure visual executions report trajectories centered in the left camera, we provide in the "evaluation" folder the transformation of the ground truth to the left camera reference. Visual-inertial trajectories use the ground truth from the dataset.EuRoC在IMU body参考里为每个序列提供了真值。由于纯视觉的可执行文件报告的是以做相机为中心的轨迹，我们在"evaluation"里提供了从真值到左相机参考系的转换。视觉惯性轨迹使用数据集中的真值。

Execute the following script to process sequences and compute the RMS ATE:执行下面的脚本处理序列并计算ATE（绝对轨迹误差）的RMS(均方根)
```
./euroc_eval_examples
```

# 6. TUM-VI ExamplesTUM-VI示例
[TUM-VI dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) was recorded with two fisheye cameras and an inertial sensor.[TUM-VI数据集](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)采用两个鱼眼相机和一个惯性传感器采集。

1. Download a sequence from https://vision.in.tum.de/data/datasets/visual-inertial-dataset and uncompress it. 从 https://vision.in.tum.de/data/datasets/visual-inertial-dataset 下载一个序列并解压。

2. Open the script "tum_vi_examples.sh" in the root of the project. Change **pathDatasetTUM_VI** variable to point to the directory where the dataset has been uncompressed. 打开位于项目根目录的脚本 "tum_vi_examples.sh"。修改 **pathDatasetTUM_VI** 变量指向数据集解压的目录。

3. Execute the following script to process all the sequences with all sensor configurations:执行下面的脚本在所有传感器配置下处理所有序列。
```
./tum_vi_examples
```

## Evaluation评测
In TUM-VI ground truth is only available in the room where all sequences start and end. As a result the error measures the drift at the end of the sequence. 在TUM-VI中，真值仅在序列在房间中的起始和终止位置可获得。这使得仅能在序列结尾测量漂移误差。

Execute the following script to process sequences and compute the RMS ATE:执行下面的脚本处理序列并计算ATE的RMS:
```
./tum_vi_eval_examples
```

# 7. ROS ExamplesROS示例

### Building the nodes for mono, mono-inertial, stereo, stereo-inertial and RGB-D为单目，单目惯性，双目，双目惯性和RGB-D构建节点
Tested with ROS Melodic and ubuntu 18.04.在Ubuntu18.04下的ROS Melodic中测试通过。

1. Add the path including *Examples/ROS/ORB_SLAM3* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:向环境变量ROS_PACKAGE_PATH添加包含 *Examples/ROS/ORB_SLAM3* 的路径。打开.bashrc文件
  ```
  gedit ~/.bashrc
  ```
and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:在末行添加下面一行的代码。将路径为你克隆ORB_SLAM3的目录。

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script:执行`build_ros.sh`脚本：

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### Running Monocular Node运行单目节点
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM3/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.为了从topic`/camera/image_raw`进行单目输入以运行nodeORB_SLAM3/Mono。你需要提供视觉里程计文件和设置文件。查看上面的单目示例。

  ```
  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

### Running Monocular-Inertial Node运行单目惯性节点
For a monocular input from topic `/camera/image_raw` and an inertial input from topic `/imu`, run node ORB_SLAM3/Mono_Inertial. Setting the optional third argument to true will apply CLAHE equalization to images (Mainly for TUM-VI dataset).为了从topoic`/camera/image_raw`和topic`/imu`获取单目和惯性的输入，运行节点ORB_SLAM3/Mono_Inertial。设置可选的第三参数为true将在图像上应用CLAHE（直方图均衡化）等式(主要是为了TUM-VI数据集)。

  ```
  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE [EQUALIZATION]	
  ```

### Running Stereo Node运行双目结点
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node ORB_SLAM3/Stereo. You will need to provide the vocabulary file and a settings file. For Pinhole camera model, if you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**. For FishEye camera model, rectification is not required since system works with original images:对于从topic`/camera/left/image_raw`和`/camera/right/image_raw`进行双目输入运行结点ORB_SLAM3/Stereo。你需要提供视觉里程计文件和设置文件。对于小孔相机模型，如果你**提供矫正矩阵**（见Examples/Stereo/EuRoC.yaml示例），这个节点将在线矫正图像，**否则图像必须被提前矫正**。对于鱼眼相机模型，不需要进行校正，因为系统可以使用原始图像工作。

  ```
  rosrun ORB_SLAM3 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```

### Running Stereo-Inertial Node运行双目-惯性结点
For a stereo input from topics `/camera/left/image_raw` and `/camera/right/image_raw`, and an inertial input from topic `/imu`, run node ORB_SLAM3/Stereo_Inertial. You will need to provide the vocabulary file and a settings file, including rectification matrices if required in a similar way to Stereo case:为了从topic`/camera/left/image_raw`、`/camera/right/image_raw`获取双目和topic`/imu`获取惯性输入，运行结点ORB_SLAM3/Stereo_Inertial。你需要提供视觉里程计和设置文件，以及与双目示例类似的矫正矩阵。

  ```
  rosrun ORB_SLAM3 Stereo_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION [EQUALIZATION]	
  ```
  
### Running RGB_D Node运行RGB_D结点
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM3/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.为从topic`/camera/rgb/image_raw`和`/camera/depth_registered/image_raw`获取RGB-D输入。你需要提供视觉里程计文件和设置文件，见上文RGB-D示例。

  ```
  rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

**Running ROS example:** Download a rosbag (e.g. V1_02_medium.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab for a Stereo-Inertial configuration:运行ROS示例：**从EuRoC数据集(http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)下载一个ROS包(如 V1_02_medium.bag)。在终端打开3个标签页，在每个标签页分别运行下面的命令以满足视觉-惯性配置。
  ```
  roscore
  ```
  
  ```
  rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
  ```
  
Once ORB-SLAM3 has loaded the vocabulary, press space in the rosbag tab.一旦ORB-SLAM3已经加载了视觉里程计，在rosbag包标签页中按空格键。

**Remark:** For rosbags from TUM-VI dataset, some play issue may appear due to chunk size. One possible solution is to rebag them with the default chunk size, for example:**注**对于TUM-VI数据集中的rosbag，也许会由于截断尺寸出现一些显示问题。一个也许有用的解决方案是以默认截断尺寸重新打包，例如：
  ```
  rosrun rosbag fastrebag.py dataset-room1_512_16.bag dataset-room1_512_16_small_chunks.bag
  ```

# 8. Running time analysis运行时间分析
A flag in `include\Config.h` activates time measurements. It is necessary to uncomment the line `#define REGISTER_TIMES` to obtain the time stats of one execution which is shown at the terminal and stored in a text file(`ExecTimeMean.txt`).`include\Config.h`中的一个标记是时间测量的开关。这对取消`#define REGISTER_TIMES`的注释以获取一次运行的时间状态是必须的，该状态在终端中显示，存储于(`ExecTimeMean.txt`)。

# 9. Calibration标定
You can find a tutorial for visual-inertial calibration and a detailed description of the contents of valid configuration files at  `Calibration_Tutorial.pdf`按照`Calibration_Tutorial.pdf`中说的教程那样标定
