# JRL Perception software
Curation of the software used/developed at JRL for perception

## Human observation

## Object detection
- [Descriptor-based](https://github.com/TsuruMasato/OnlineObjectDetector) - 3D descriptor-based, within point cloud, if object shape known (Tsuru)

## Object tracking
- [big_object_tracking](https://gite.lirmm.fr/chappellet/big_object_tracking) - with Depth camera (Kevin)

## SLAM
- [RTAB-Map](https://github.com/introlab/rtabmap) - with RGB-D camera (integrated by Kevin, Tsuru)
- [StellaVSLAM](https://github.com/stella-cv/stella_vslam) - former OpenVSLAM
- [OpenVSLAM-omni](https://github.com/GuicarMIS/openvslam-omni) - former OpenVSLAM extended to Barreto's model for panoramic cameras (Guillaume - private repository)

## Mapping
- [ExOctomap](https://github.com/isri-aist/ex_octomap) - create 3D voxel map, with semantic segmentation. (Tsuru)
- [Localized Octomap](https://github.com/isri-aist/local_octomap) - accerelated Octomap, with a limited map size. (Tsuru)

## Perceptual features for control 
Proposed Perception-Control bridge for tactile sensing and visual servoing.

- [DirectVisualServoing](https://github.com/jrl-umi3218/DirectVisualServoing) - direct visual servoing with conventional (photometric visual servoing) or large aperture camera (defocus-based visual servoing) (Guillaume, Belinda)
- [ros_dvs_bridge](https://github.com/jrl-umi3218/ros_dvs_bridge) - same as above but within a ROS wrapping (Guillaume, Mitsuharu)
- [libPeR](https://github.com/PerceptionRobotique/libPeR) - library of robotic (visual) perception direct features for various types of camera, for visual servoing and camera motion estimation (Guillaume, Antoine Andre, ... - private repository)
- [libPeR_base](https://github.com/PerceptionRobotique/libPeR_base) - public (minor) part of the above, mostly used in public repositories of spherical image format management, i.e. [dualfisheye2equi](https://github.com/PerceptionRobotique/dualfisheye2equi), [equi2equi](https://github.com/PerceptionRobotique/equi2equi), ... (Guillaume, Antoine Andre, Samuel, ...)
