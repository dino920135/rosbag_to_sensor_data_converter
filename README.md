# rosbag_to_sensor_data_converter

Convert sensor data (image, point cloud) in rosbag to common file type (.jpeg, .png, .pcd)

## Dependency

1. Install ROS 

2. Install CV & ros-related & pcl dependencies
```bash
sudo apt-get install ros-melodic-ros-numpy python-cv-bridge opencv-python
sudo apt-get install libpcl-dev pcl-tools
```

3. Install **python-pcl** from source according to https://iter01.com/572096.html


## Usage
### Check avalible Topics
```bash
rosbag info <bag_dir>
```
### Start Convert

```bash
python sensor_bag_convert.py <bag_dir> <topic> <output_dir>
```

## Support Messages
### Image
- sensor_msgs/CompressedImage -> .png (or .jpeg)
### Pointcloud
- sensor_msgs/PointCloud2 -> .pcd (or .ply)


