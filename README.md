# ICP_ROS v2.0

C++ implementation of iterative closest point. Adapted from https://github.com/ClayFlannigan/icp and https://github.com/zjudmd1015/icp

This ROS node has support for sensor_msgs::LaserScan, sensor_msgs::PointCloud, and sensor_msgs::PointCloud2.
Depending on the specified mode, this node can subscribe to two input pointclouds at once (a to b), or match sequentially with the previous scan. The output is a 4x4 homogeneous transformation matrix between two pointclouds. There are also options to publish the input scans for debug purposes and an rviz config file for visualization with the default settings.

This library makes use of [Eigen3.3](https://gitlab.com/libeigen/eigen) and the [nanoflann header library](https://github.com/jlblancoc/nanoflann) in order to build a kdtree for efficient nearest neighbor lookup.

Command line option parsing provided via [cxxopts](https://github.com/jarro2783/cxxopts)
