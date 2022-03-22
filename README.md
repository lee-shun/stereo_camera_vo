# stereo camera visual odometry

> A very lightweight visual odometry with local bundle adjustment (BA).

- Test running on [kitti dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php). Processing each
  frames with **20 ms**.

## <p align="center">![run_kitti](./doc/kitti_data_test.gif)</p>

- Test running on DJI M300 front stereo camera, the video is recorded around Concordia University, Montreal, CA.

## <p align="center">![run_m300](https://github.com/lee-shun/big_files/blob/master/images/stereo_camera_vo_images/cut.gif)</p>

**NOTE:**

- Green frame indicates current camera pose, while the red and black point clouds are the active and inactive landmarks
  respectively.

## Features

- Stereo Camera based visual odometry.
- LK optical flow is used to track the keypoints.
- Lost tracking then relocate.
- Insert keyframes according to the keypoints number.
- Local bundle adjustment.

## Dependencies

- [g2o](https://github.com/RainerKuemmerle/g2o)
  > optimization in local bundle adjustment and frontend camera pose estimation.

- [pangolin](https://github.com/stevenlovegrove/Pangolin)
  > visualization

- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) & [Sophus](https://github.com/strasdat/Sophus)
  > basic matrix operations and Lie Algebra operations.

- [OpenCV 4](https://opencv.org/)
  > general image processing methods and basic data structures.

## Future Work

Integrate with [forest fire detection system](https://github.com/lee-shun/forest_fire_detection_system) to locate in GPS
denied environments.

## Acknowledgement

[*"14 Lectures on Visual SLAM: From Theory to Practice"* Xiang Gao](https://github.com/gaoxiang12/slambook-en)
