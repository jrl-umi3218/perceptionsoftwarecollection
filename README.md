# JRL Perception software
Curation of the software used/developed at JRL for perception

## Human observation
- [Body tracking split from Azure Kinect camera](https://github.com/GuicarMIS/Azure_Kinect_ROS_Driver/tree/splitBodyTrackingOption) - the body detection and tracking of the Azure Kinect camera split from the capture program to allow using it on a computer different than one to which the Azure Kinect is connected (Guillaume)
  
## Object detection
- [Descriptor-based](https://github.com/TsuruMasato/OnlineObjectDetector) - 3D descriptor-based, within point cloud, if object shape known (Tsuru)
- [2D-detection](https://github.com/isri-aist/2D-detection) - YOLOv8-based object detection and rough pose estimate (Virgile)
- [6D-pose-estimation](https://github.com/isri-aist/6D-pose-estimation) - adaptation of [big_object_tracking](https://gite.lirmm.fr/chappellet/big_object_tracking) (see the Object tracking section) to robust pose estimation (Virgile)
  
## Object tracking
- [big_object_tracking](https://gite.lirmm.fr/chappellet/big_object_tracking) - with Depth camera (Kevin, Virgile) 

## SLAM
- [RTAB-Map](https://github.com/introlab/rtabmap) - with RGB-D camera (integrated by Kevin, Tsuru)
- [StellaVSLAM](https://github.com/stella-cv/stella_vslam) - former OpenVSLAM
- [OpenVSLAM-omni/open2stella](https://github.com/GuicarMIS/openvslam-omni/tree/open2stella) - former OpenVSLAM extended to Barreto's model for panoramic cameras rebased on StellaVSLAM (Guillaume - private repository)

## Camera tracking
- [VisualGyroscope](https://github.com/PerceptionRobotique/VisualGyroscope) - camera 3D rotation estimation (Antoine Andre)

## Mapping
- [ExOctomap](https://github.com/isri-aist/ex_octomap) - create 3D voxel map, with semantic segmentation. (Tsuru)
- [Localized Octomap](https://github.com/isri-aist/local_octomap) - accerelated Octomap, with a limited map size. (Tsuru)

## Perceptual features for control 
Perception-Control bridge for tactile sensing and visual servoing:

- [DirectVisualServoing](https://github.com/jrl-umi3218/DirectVisualServoing) - direct visual servoing with conventional (photometric visual servoing) or large aperture camera (defocus-based visual servoing) (Guillaume, Belinda)
- [ros_dvs_bridge](https://github.com/jrl-umi3218/ros_dvs_bridge) - same as above but within a ROS wrapping (Guillaume, Mitsuharu)
- [libPeR](https://github.com/PerceptionRobotique/libPeR) - library of robotic (visual) perception direct features for various types of camera, for visual servoing and camera motion estimation (Guillaume, Antoine Andre, ... - private repository)
- [libPeR_base](https://github.com/PerceptionRobotique/libPeR_base) - public part of the above, used for [VisualGyroscope](https://github.com/PerceptionRobotique/VisualGyroscope) (see the Camera tracking Section), [VisualServoing](https://github.com/PerceptionRobotique/VisualServoing) (see below) and public repositories of spherical image format management, i.e. [dualfisheye2equi](https://github.com/PerceptionRobotique/dualfisheye2equi), [equi2equi](https://github.com/PerceptionRobotique/equi2equi), ... (Guillaume, Antoine Andre, ...)
- [VisualServoing](https://github.com/PerceptionRobotique/VisualServoing) - robot (arm) control from camera visual feedback (Antoine Andre, Guillaume, ...)

## Miscellaneous 
- Spherical image transformation tools:
  - [dualfisheye2equi](https://github.com/PerceptionRobotique/dualfisheye2equi) - warps a dualfisheye image to an equirectangular image
  - [equi2omni](https://github.com/PerceptionRobotique/equi2omni) - warps an equirectangular to an omnidirectional image
  - [equi2equi](https://github.com/PerceptionRobotique/equi2equi) - warps an equirectangular image to a sphere, transform it, then map back to the equirectangular plane
  - [dual2dual](https://github.com/PerceptionRobotique/dual2dual) - warps a dualfisheye image to a sphere, transform it, then map back as a dualfisheye image
 
- Masking depth map: [ros_mask_depth_map](https://github.com/isri-aist/ros_mask_depth_map) - use a dynamic mask to mask the depth map of an RGBD camera captured on the fly

- Fiducial markers for panoramic vision: [ArUcOmni](https://github.com/GuicarMIS/ArUcOmni) - detection and pose estimation of ArUco marker observed by a camera obeying the unified central model
