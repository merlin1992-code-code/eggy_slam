<!--
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-04 16:08:03
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-06 22:56:02
-->
# eggy_slam

### Build
You can install the Traj-LO project by following these steps:
```
./scripts/install_deps.sh # make sure we have all the dependency
mkdir build && cd build
cmake .. 
make -j8
```

### Run
After modifying the config file for your environment, you can run EGGY-SLAM. Here is an example to test it with Voyah RS128 lidar.
```
./eggy --lio ../config/lio.yaml --dyn ../config/m-detector.yaml --mode_fusion
```

### TOOLS
#### STITCH
stitch multi lidar frames to a map with pose
```
./stitch poses.yaml /path/to/pcd_dir output.pcd
```
#### ToRosbag
package lidar and IMU data to rosbag
```
./ToRosbag ../config/bag.yaml
```