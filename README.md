# JRL Perception software
Curation of the software used/developed at JRL for perception

## Human observation
- [Body tracking split from Azure Kinect camera](https://github.com/GuicarMIS/Azure_Kinect_ROS_Driver/tree/splitBodyTrackingOption) - the body detection and tracking of the Azure Kinect camera split from the capture program to allow using it on a computer different than one to which the Azure Kinect is connected (Guillaume)
- [RGB-D Immersion for Unreal Engine 5.1 with Azure Kinect](https://github.com/isri-aist/rgbd-immersion) - Implementation of depth on a portion of a sphere to approximate reality and the FoV of the Azure Kinect camera (Raphaël)
  
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
- [OpenVSLAM-equiRGBD](https://github.com/GuicarMIS/openvslam-omni/tree/stellaEquiRGBD) - OpenVSLAM-omni extended to Equirectangular RGBD SLAM (Guillaume - private repository)

## Camera tracking
- [VisualGyroscope](https://github.com/PerceptionRobotique/VisualGyroscope) - camera 3D rotation estimation (Antoine Andre)
- [ros_omni_gyro_gpu](https://github.com/isri-aist/ros_omni_gyro_gpu) - gpu acceleration of [VisualGyroscope](https://github.com/PerceptionRobotique/VisualGyroscope) (Thomas)
- [VisualAlignment](https://github.com/PerceptionRobotique/VisualAlignment) - direct alignment of RGB 3D point cloud to a 360-degree equirectangular image (Guillaume, Dorian)

## Mapping
- [ExOctomap](https://github.com/isri-aist/ex_octomap) - create 3D voxel map, with semantic segmentation. (Tsuru)
- [Localized Octomap](https://github.com/isri-aist/local_octomap) - accerelated Octomap, with a limited map size. (Tsuru)
- [CALL-M_core](https://github.com/isri-aist/CALL-M_core) - perception-based navigation stack for the CALL-M mobile manipulator (Ocean, Rafa)

## Perceptual features for control 
Perception-Control bridge for tactile sensing and visual servoing:

- [DirectVisualServoing](https://github.com/jrl-umi3218/DirectVisualServoing) - direct visual servoing with conventional (photometric visual servoing) or large aperture camera (defocus-based visual servoing) (Guillaume, Belinda)
- [ros_direct_visual_servoing](https://github.com/isri-aist/ros_direct_visual_servoing) - same as above but within a ROS wrapping; public version of [ros_dvs_bridge](https://github.com/jrl-umi3218/ros_dvs_bridge) (Guillaume, Antoine, Sinta, Mitsuharu)
- [libPeR](https://github.com/PerceptionRobotique/libPeR) - library of robotic (visual) perception direct features for various types of camera, for visual servoing and camera motion estimation (Guillaume, Antoine Andre, ... - private repository)
- [libPeR_base](https://github.com/PerceptionRobotique/libPeR_base) - public part of the above, used for [VisualGyroscope](https://github.com/PerceptionRobotique/VisualGyroscope) (see the Camera tracking Section), [VisualServoing](https://github.com/PerceptionRobotique/VisualServoing) (see below) and public repositories of spherical image format management, i.e. [dualfisheye2equi](https://github.com/PerceptionRobotique/dualfisheye2equi), [equi2equi](https://github.com/PerceptionRobotique/equi2equi), ... (Guillaume, Antoine Andre, ...)
- [VisualServoing](https://github.com/PerceptionRobotique/VisualServoing) - robot (arm) control from camera visual feedback (Antoine Andre, Guillaume, ...)
- [visual_gyroscope_plugin](https://github.com/isri-aist/visual_gyroscope_plugin) - mc_rtc visual gyroscope plugin (Thomas, Antoine)
- [VisualServoing mc_rtc](https://github.com/PerceptionRobotique/VisualServoing_private/tree/MC_RTC_Integration) - visual servoing integration in mc_rtc (Thomas)
- [vs_tasks_generator_controller](https://github.com/isri-aist/vs_tasks_generator_controller) - mc_rtc visual servoing tasks generator (Thomas)
- [DHPGMVS](https://github.com/isri-aist/DHPGMVS) - Dual-Hemispherical Photometric Gaussian Mixture-based Visual Servoing extending [DHPVS](https://github.com/NathanCrombez/DHPVS) to the Photometric Gaussian Mixture feature thanks to [libPeR_base](https://github.com/PerceptionRobotique/libPeR_base) (Guillaume)

## Calibration
- [MIXEDVISION](https://github.com/PerceptionRobotique/MIXEDVISION) - multiple cameras simultaneous intrinsic and extrinsic calibration using boards, capable of mixing camera types. Allows dual configurations too with multi-plane calibration targets (Guillaume)
- [rgbd2rgbalign](https://github.com/isri-aist/rgbd2rgbalign) - direct (grayscale) alignment to improve the rigid transformation from an RGBD camera 3D point cloud to an equirectangular one (Guillaume)
- [rgbd2rgbdalign](https://github.com/isri-aist/rgbd2rgbdalign) - direct (Z values) alignment to calibrate a wide-angle depth camera from the depth map of an rgbd camera, matching the color image (Guillaume)
- [undistortCalibFromDistort](https://github.com/isri-aist/undistortCalibFromDistort) - compute undistortion parameters from distorsion ones ( (1) rational polynomial + tangential; (2) OpenCV's Fisheye Model)

## Miscellaneous 
- Spherical image transformation tools:
  - [dualfisheye2equi](https://github.com/PerceptionRobotique/dualfisheye2equi) - warps a dualfisheye image to an equirectangular image
  - [equi2omni](https://github.com/PerceptionRobotique/equi2omni) - warps an equirectangular to an omnidirectional image
  - [equi2equi](https://github.com/PerceptionRobotique/equi2equi) - warps an equirectangular image to a sphere, transform it, then map back to the equirectangular plane
  - [dual2dual](https://github.com/PerceptionRobotique/dual2dual) - warps a dualfisheye image to a sphere, transform it, then map back as a dualfisheye image
  - [persp2equi](https://github.com/PerceptionRobotique/equi2equi) - warps a perspective image to a sphere, transform it if necessary, then map back as an equirectangular image
 
- Masking depth map: [ros_mask_depth_map](https://github.com/isri-aist/ros_mask_depth_map) - use a dynamic mask to mask the depth map of an RGBD camera captured on the fly

- Fiducial markers for panoramic vision: [ArUcOmni](https://github.com/GuicarMIS/ArUcOmni) - detection and pose estimation of ArUco marker observed by a camera obeying the unified central model
-  [ACFITSIO](https://github.com/isri-aist/ACFITSIO) - Library to use FITS images.
-  [ids_driver_lib](https://github.com/isri-aist/ids_driver_lib) - C++ library for IDS camera
-  [IDS4HDR](https://github.com/isri-aist/IDS4HDR) - Program to record dual exposures images from the IDS camera in RAW format in FIT images (tested up to 30 FPS)
-  [Azure Kinect IR Image Streamer](https://github.com/isri-aist/AKIRS) - Preprocess passive mode IR images.

- mc_rtc tools:
  - [RecordingCamera](https://github.com/isri-aist/RecordingCamera) - Thread-safe camera manager 
  - [vs_xp_logger](https://github.com/isri-aist/vs_xp_logger) - visual servoing experiment logger 

- vs camera trajectory analysis tool: [evs](https://github.com/NathanCrombez/evs) - evaluation of visual servoing: computes an ideal path from the desired and initial poses and an achieved path

- [differentiableImage](https://github.com/isri-aist/differentiableImage) - tool to compute the minimal Gaussian spread that makes a difference between two images or a single image differentiable (Guillaume, Thomas)

- [FuseDPT](https://github.com/isri-aist/FuseDPT) - Dense depth prediction from RGB equirectangular image (Matheus, Nevrez)
