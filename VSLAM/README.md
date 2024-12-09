# The list of vision-based SLAM / Visual Odometry open source projects, libraries, dataset, tools, and studies

[![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/tzutalin/awesome-visual-slam)

## Index
* [Libraries](#libraries)
* [Dataset](#dataset)
* [Tools](#tools)
* [Projects](#projects)
* [Learn](pages/learn.md)
* [Miscellaneous](pages/miscellaneous.md)

## Libraries
###### Basic vision and trasformation libraries
- [OpenCV](http://opencv.org/)
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Sophus](https://github.com/strasdat/Sophus)
- [ROS](http://www.ros.org/)
- [PointCloud](http://pointclouds.org/)

###### Thread-safe queue libraries
- [concurrentqueue](https://github.com/cameron314/concurrentqueue)
- [Intel® TBB](https://www.threadingbuildingblocks.org/)
- [Facebook folly PC](https://github.com/facebook/folly/blob/master/folly/ProducerConsumerQueue.h)

###### Loop detection
- [dorian3d](https://github.com/dorian3d)

###### Graph Optimization
- [ceres-solver](https://github.com/ceres-solver/ceres-solver)
- [g2o](https://github.com/RainerKuemmerle/g2o)
- [gtsam](https://collab.cc.gatech.edu/borg/gtsam?destination=node%2F299)
- [Vertigo](http://openslam.org/vertigo.html)

###### Map library
- [ETHZ ASL/Grid Map](https://github.com/ethz-asl/grid_map)
- [OmniMapper](https://github.com/CognitiveRobotics/omnimapper/wiki)
- [OctoMap](https://github.com/OctoMap/octomap)

## Dataset

Dataset for benchmark/test/experiment/evalutation

- [TUM University](http://vision.in.tum.de/data/datasets/rgbd-dataset/download)
- [KITTI Vision benchmark](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
- [UNI-Freiburg](http://kaspar.informatik.uni-freiburg.de/~slamEvaluation/datasets.php)
- [ADVIO](https://github.com/AaltoVision/ADVIO)
- [Oxford RobotCar Dataset](https://robotcar-dataset.robots.ox.ac.uk/)
- [HRI (Honda Research Institute) Driving Datasets](https://usa.honda-ri.com/honda-driving-datasets)
- [Argoverse](https://www.argoverse.org/data.html)
- [nuScenes](https://www.nuscenes.org)
- [Waymo Open Dataset](https://waymo.com/open/)
- [Lyft Level 5 AV Dataset 2019](https://level5.lyft.com/dataset/)
- [KAIST Urban Dataset](https://irap.kaist.ac.kr/dataset/)

## Tools
- [rgbd-dataset tool from TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools)
- [evo - evaluation tool for different trajectory formats](https://github.com/MichaelGrupp/evo)
- [VDO_SLAM - A Visual Object-aware Dynamic SLAM library](https://github.com/halajun/vdo_slam) 

## Projects

###### RGB (Monocular):

- [Kimera](https://github.com/MIT-SPARK/Kimera). Available on ROS
> A. Rosinol, M. Abate, Y. Chang, L. Carlone. Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping. arXiv preprint arXiv:1910.02490.

- [PTAM](https://github.com/Oxford-PTAM/PTAM-GPL)
> [1] Georg Klein and David Murray, "Parallel Tracking and Mapping for Small AR Workspaces", Proc. ISMAR 2007
> [2] Georg Klein and David Murray, "Improving the Agility of Keyframe-based SLAM", Proc. ECCV 2008


- [DSO](https://github.com/JakobEngel/dso_ros). Available on ROS
>Direct Sparse Odometry, J. Engel, V. Koltun, D. Cremers, In arXiv:1607.02565, 2016
>A Photometrically Calibrated Benchmark For Monocular Visual Odometry, J. Engel, V. Usenko, D. Cremers, In arXiv:1607.02555, 2016

- [LSD-SLAM](https://github.com/tum-vision/lsd_slam). Available on ROS
>LSD-SLAM: Large-Scale Direct Monocular SLAM, J. Engel, T. Schöps, D. Cremers, ECCV '14
>Semi-Dense Visual Odometry for a Monocular Camera, J. Engel, J. Sturm, D. Cremers, ICCV '13

- [ORB-SLAM](https://github.com/raulmur/ORB_SLAM). Available on ROS
> [1] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. IEEE > Transactions on Robotics, vol. 31, no. 5, pp. 1147-1163, 2015. (2015 IEEE Transactions on Robotics Best Paper Award). PDF.
> [2] Dorian Gálvez-López and Juan D. Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences. IEEE > Transactions on Robotics, vol. 28, no. 5, pp. 1188-1197, 2012. PDF.

- [Nister's Five Point Algorithm for Essential Matrix estimation, and FAST features, with a KLT tracker](https://github.com/avisingh599/mono-vo)
>D. Nister, “An efficient solution to the five-point relative pose problem,” Pattern Analysis and Machine Intelligence, IEEE Transactions on, vol. 26, no. 6, pp. 756–770, 2004.

- [SVO-SLAM](https://github.com/uzh-rpg/rpg_svo). Available on ROS
> Christian Forster, Matia Pizzoli, Davide Scaramuzza, "SVO: Fast Semi-direct Monocular Visual Odometry," IEEE International Conference on Robotics and Automation, 2014.

###### RGB and Depth (Called RGBD):
- [OpenCV RGBD-Odometry (Visual Odometry based RGB-D images)](https://github.com/tzutalin/OpenCV-RgbdOdometry)
> Real-Time Visual Odometry from Dense RGB-D Images, F. Steinbucker, J. Strum, D. Cremers, ICCV, 2011

- [Dense Visual SLAM for RGB-D Cameras](https://github.com/tum-vision/dvo_slam). Available on ROS
>[1]Dense Visual SLAM for RGB-D Cameras (C. Kerl, J. Sturm, D. Cremers), In Proc. of the Int. Conf. on Intelligent Robot Systems (IROS), 2013.
[2]Robust Odometry Estimation for RGB-D Cameras (C. Kerl, J. Sturm, D. Cremers), In Proc. of the IEEE Int. Conf. on Robotics and Automation (ICRA), 2013
[3]Real-Time Visual Odometry from Dense RGB-D Images (F. Steinbruecker, J. Sturm, D. Cremers), In Workshop on Live Dense Reconstruction with Moving Cameras at the Intl. Conf. on Computer Vision (ICCV), 2011.


- [RTAB MAP - Real-Time Appearance-Based Mapping](https://github.com/introlab/rtabmap). Available on ROS
> Online Global Loop Closure Detection for Large-Scale Multi-Session Graph-Based SLAM, 2014
> Appearance-Based Loop Closure Detection for Online Large-Scale and Long-Term Operation, 2013

- [ORB2-SLAM](https://github.com/raulmur/ORB_SLAM2). Available on ROS
> [1] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. IEEE > Transactions on Robotics, vol. 31, no. 5, pp. 1147-1163, 2015. (2015 IEEE Transactions on Robotics Best Paper Award).
> [2] Dorian Gálvez-López and Juan D. Tardós. Bags of Binary Words for Fast Place Recognition in Image Sequences. IEEE Transactions on Robotics, vol. 28, no. 5, pp. 1188-1197, 2012.

- [InfiniTAM∞ v2](http://www.robots.ox.ac.uk/~victor/infinitam/index.html)
> Kahler, O. and Prisacariu, V.~A. and Ren, C.~Y. and Sun, X. and Torr, P.~H.~S and Murray, D.~W. Very High Frame Rate Volumetric Integration of Depth Images on Mobile Device. IEEE Transactions on Visualization and Computer Graphics (Proceedings International Symposium on Mixed and Augmented Reality 2015

- [Kintinuous](https://github.com/mp3guy/Kintinuous)
> Real-time Large Scale Dense RGB-D SLAM with Volumetric Fusion, T. Whelan, M. Kaess, H. Johannsson, M.F. Fallon, J. J. Leonard and J.B. McDonald, IJRR '14

- [ElasticFusion](https://github.com/mp3guy/ElasticFusion)
> [1] ElasticFusion: Real-Time Dense SLAM and Light Source Estimation, T. Whelan, R. F. Salas-Moreno, B. Glocker, A. J. Davison and S. Leutenegger, IJRR '16
> [2] ElasticFusion: Dense SLAM Without A Pose Graph, T. Whelan, S. Leutenegger, R. F. Salas-Moreno, B. Glocker and A. J. Davison, RSS '15

- [Co-Fusion](http://visual.cs.ucl.ac.uk/pubs/cofusion/index.html)
> Martin Rünz and Lourdes Agapito. Co-Fusion: Real-time Segmentation, Tracking and Fusion of Multiple Objects. 2017 IEEE International Conference on Robotics and Automation (ICRA)

###### RGBD and LIDAR:
- [Google's cartographer](https://github.com/googlecartographer/cartographer). Available on ROS


## Other open source projects
[DynaSLAM](https://github.com/BertaBescos/DynaSLAM) A SLAM system robust in dynamic environments for monocular, stereo and RGB-D setups

[openvslam](https://github.com/xdspacelab/openvslam) A Versatile Visual SLAM Framework


## License

[![CC0](http://mirrors.creativecommons.org/presskit/buttons/88x31/svg/cc-zero.svg)](https://creativecommons.org/publicdomain/zero/1.0/)

Learn
======================

#### Article & blogs
* [Introduction to Epipolar Geometry and the Fundamental Matrix](http://www.umiacs.umd.edu/~ramani/cmsc828d/lecture27.pdf)
* [Short introduction to descriptors](https://gilscvblog.com/2013/08/18/a-short-introduction-to-descriptors/)
* [Tutorial on binary descriptors](https://gilscvblog.com/2013/08/26/tutorial-on-binary-descriptors-part-1/)
* [Latch descriptor](https://gilscvblog.com/2015/11/07/performance-evaluation-of-binary-descriptor-introducing-the-latch-descriptor)
* [Robot Mapping - UniFreiburg](http://ais.informatik.uni-freiburg.de/teaching/ws15/mapping/)
* [Short introduction to signed distance function 1](http://rsdavis.mycpanel.princeton.edu/wp/?p=24)
* [Short introduction to signed distance function 2](http://www.personal.psu.edu/users/j/p/jpm5375/assignment6.html/)


### Online tutorial videos
* [Slambook in Chinese](https://space.bilibili.com/38737757#!/)

#### Online free courses
* [Introduction to Mobile Robotics - UniFreiburg](http://ais.informatik.uni-freiburg.de/teaching/ss16/robotics/)
* [Multiple View Geometry (Prof. D. Cremers) - TUM](https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4)
* [Robot Percetion](https://www.coursera.org/learn/robotics-perception)


#### Books
* [A tutorial on SE(3) transformation parameterizations and on-manifold optimization](https://pixhawk.org/_media/dev/know-how/jlblanco2010geometry3d_techrep.pdf) By JL Blanco, 2014
* [State Estimation for Robotic -- A Matrix Lie Group Approach](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser15.pdf) by Timothy D. Barfoot, 2016
* [Simultaneous Localization and Mapping for Mobile Robots: Introduction and Methods](http://www.igi-global.com/book/simultaneous-localization-mapping-mobile-robots/66380) by Juan-Antonio Fernández-Madrigal and José Luis Blanco Claraco, 2012
* [Probabilistic Robotics](http://www.probabilistic-robotics.org/) by Dieter Fox, Sebastian Thrun, and Wolfram Burgard, 2005
* [An Invitation to 3-D Vision -- from Images to Geometric Models](http://vision.ucla.edu/MASKS/) by Yi Ma, Stefano Soatto, Jana Kosecka and Shankar S. Sastry, 2005
* [Multiple View Geometry in Computer Vision](http://www.robots.ox.ac.uk/~vgg/hzbook/) by Richard Hartley and Andrew Zisserman, 2004
* [Numerical Optimization](http://home.agh.edu.pl/~pba/pdfdoc/Numerical_Optimization.pdf) by Jorge Nocedal and Stephen J. Wright, 1999
* [Real Time Monocular 3D Reconstruction](https://fradelg.gitbooks.io/real-time-3d-reconstruction-from-monocular-video/content/index.html) by Fco. Javier Delgado del Hoyo, 2015
* [Basic Knowlege on Visual Slam: From Theory to Practice](https://github.com/gaoxiang12/slambook-en) by Xiang Gao, Tao Zhang, Qinrui Yan and Yi Liu, 2021


#### Papers
- [CNN-SLAM: Real-time dense monocular SLAM with learned depth prediction](https://www.google.com.tw/url?sa=t&rct=j&q=&esrc=s&source=web&cd=3&cad=rja&uact=8&ved=0ahUKEwjZ-NWF_-zWAhXIEpQKHX_BCUAQFgg0MAI&url=http%3A%2F%2Fcampar.in.tum.de%2Fpub%2Ftateno2017cvpr%2Ftateno2017cvpr.pdf&usg=AOvVaw0O2qiB3IPWDg7rJYEs3OQU) (2017)
- [Online Photometric Calibration for Auto Exposure Video for Realtime Visual Odometry and SLAM](http://arxiv.org/pdf/1710.02081v1) (2017)
- [Visual-Inertial Direct SLAM](https://github.com/kanster/awesome-slam/blob/master/webdiis.unizar.es/~jcivera/papers/concha_etal_icra16.pdf) (2016)
- [Past, Present, and Future of Simultaneous Localization And Mapping: Towards the Robust-Perception Age](https://arxiv.org/abs/1606.05830) (2016)
- [Direct Sparse Odometry](https://arxiv.org/abs/1607.02565) (2016)
- [Modelling Uncertainty in Deep Learning for Camera Relocalization](https://arxiv.org/abs/1509.05909) (2016)
* [Convolutional Neural Network-Based Image Representation for Visual Loop Closure Detection](https://arxiv.org/pdf/1504.05241.pdf) (2015)
* [Lagrangian duality in 3D SLAM: Verification techniques and optimal solutions](http://arxiv.org/abs/1506.00746) (2015)
* [Lucas-Kanade 20 Years On: A Unifying Framework](http://www.ncorr.com/download/publications/bakerunify.pdf) (2004)

# Miscellaneous


Title | Description
--- | ---
[OpenSLAM](https://openslam.org/) | Provides a platform for SLAM researchers which gives them the possibility to publish their algorithms.
[awesome-computer-vision](https://github.com/jbhuang0604/awesome-computer-vision) | A curated list of awesome computer vision resources.
[StackExchange-Robotics](https://robotics.stackexchange.com) | StackExchange about Robotics
[cvprtum-Youtube](https://www.youtube.com/channel/UCRf1mhfcDeJS2HLP6YR07kA) | CVPRTUM Youtube channel
[Center For research in Computer Vision-Youtube](https://www.youtube.com/user/UCFCRCV) | CENTER FOR RESEARCH IN COMPUTER VISION Youtube channel