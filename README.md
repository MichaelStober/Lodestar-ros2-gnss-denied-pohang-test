<div align="center">
  <h1>LodeStar — ROS 2 port</h1>
  <a href="/"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
  <a href="/"><img src="https://img.shields.io/badge/ROS%202-Jazzy-blue" alt="ROS 2 Jazzy" /></a>
  <a href="https://ieeexplore.ieee.org/document/10380692"><img src="https://img.shields.io/badge/Paper-PDF-yellow" alt="Paper" /></a>
  <a href="https://arxiv.org/abs/2403.02773"><img src="https://img.shields.io/badge/arXiv-2410.01325-b31b1b.svg?style=flat-square" alt="Arxiv" /></a>
  <a href="https://youtu.be/YRMNGUgaGSI?feature=shared"><img src="https://badges.aleen42.com/src/youtube.svg" alt="YouTube" /></a>
  <br />
  <br />
</div>

This repository contains the test code for the descriptor in the paper "LodeStar: Maritime Radar Descriptor for Semi-Direct Radar Odometry".

It mainly deals with the synthesis of the LodeStar descriptor and the point-normal matcher from [CFEAR](https://github.com/dan11003/CFEAR_Radarodometry_code_public.git).

This branch is the **ROS 2 port** of the original ROS 1 Noetic implementation. The odometry algorithm is unchanged; only the middleware layer was migrated. See `MIGRATION_PLAN_ROS2_JAZZY.md` for the full mapping from the ROS 1 sources.

![lodestar_results](https://github.com/hyesu-jang/LodeStar/assets/30336462/08def05c-b2ac-4d4d-aed5-bdd605084c0c)

## Prerequisites

* Ubuntu 24.04 with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
* Google Ceres solver 2.x, PCL 1.14, OpenCV 4, Eigen 3.4 (all available as Ubuntu packages)

```bash
sudo apt install ros-jazzy-desktop ros-dev-tools
sudo apt install \
  ros-jazzy-rosbag2-cpp ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-cv-bridge ros-jazzy-pcl-conversions \
  ros-jazzy-tf2-eigen ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-angles ros-jazzy-rviz2
sudo apt install libpcl-dev libopencv-dev libceres-dev libeigen3-dev \
  libboost-program-options-dev libomp-dev
```

On Ubuntu 22.04 use ROS 2 Humble instead; the only source-level difference is that `cv_bridge/cv_bridge.hpp` is called `cv_bridge/cv_bridge.h` there.

## Build with colcon

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this project>
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select lodestar_odometry --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Preparing the data: ROS 1 bag → ROS 2 bag

rosbag2 cannot read ROS 1 `.bag` files, so existing recordings have to be converted once. The [`rosbags`](https://ternaris.gitlab.io/rosbags/) package does this without needing a ROS 1 installation:

Ubuntu 24.04 ships a PEP 668 "externally managed" Python, so `pip install --user` is refused; use `pipx` (or a virtualenv):

```bash
sudo apt install pipx && pipx ensurepath   # then restart the shell
pipx install rosbags

rosbags-convert --src ~/ros1_data/odom_test/test_sequence3.bag \
                --dst ~/ros2_data/odom_test/test_sequence3
# optional: keep only the radar topic, and/or write MCAP instead of SQLite
rosbags-convert --src <in.bag> --dst <out_dir> --include-topic /radar_image_inrange
rosbags-convert --src <in.bag> --dst <out_dir> --dst-storage mcap
```

The result is a **directory** containing `metadata.yaml`; that directory path is what `--bag_path` expects. Verify with:

```bash
ros2 bag info ~/ros2_data/odom_test/test_sequence3
# Topic: /radar_image_inrange | Type: sensor_msgs/msg/Image | Count: N
```

The default radar topic is `/radar_image_inrange` and the image is expected to be a Cartesian radar image in `mono8`.

## Running

Offline estimation runs at maximum frequency; the bag is read frame by frame rather than played back.

```bash
# Option A: the convenience script (edit the paths at the top)
ros2 run lodestar_odometry run_lodestar_odom.sh

# Option B: launch file, RViz2 included
ros2 launch lodestar_odometry lodestar_odom.launch.py \
    bag_path:=$HOME/ros2_data/odom_test/test_sequence3 \
    est_directory:=$HOME/ros2_data/odom_test/test_sequence3_eval/ \
    sequence:=test_sequence3

# Option C: the node directly
ros2 run lodestar_odometry lodestar_odom \
    --bag_path <bag dir> --est_directory <out dir>/ --sequence <name> \
    --radar_topic /radar_image_inrange --contour_threshold 214 --k_nearest 20 \
    --cost_type P2P --submap_scan_size 1 --res 3 --z-min 60 \
    --weight_option 4 --weight_intensity true --range-res 0.05 \
    --soft_constraint false --disable_compensate true --dataset marine --job_nr 1
```

`--est_directory` must end with a trailing slash: the parameter dump is written to `<est_directory>../pars.txt`, as in the original.

Note that `roscore` is gone — ROS 2 needs no master.

## Published topics

| Topic | Type |
|---|---|
| `/lodestar_odom_node/radar_odom` | `nav_msgs/msg/Odometry` |
| `/lodestar_odom_node/radar_odom_keyframe` | `nav_msgs/msg/Odometry` |
| `/lodestar_odom_node/radar_registered` | `sensor_msgs/msg/PointCloud2` |
| `/lodestar_odom_node/radar_registered_keyframe` | `sensor_msgs/msg/PointCloud2` |
| `/marine/Filtered` | `sensor_msgs/msg/PointCloud2` |
| `/radar_imported` | `sensor_msgs/msg/Image` |
| `/rot_lodestar` | `nav_msgs/msg/Odometry` |
| `/current_normals` | `visualization_msgs/msg/MarkerArray` |
| `/tf` (when `publish_tf` is set) | `world` → `radar_link` |

These match the ROS 1 effective names, so the RViz configuration carries over unchanged. Under ROS 1 every publisher was created on a private node handle `ros::NodeHandle("~")`, which left names with a leading `/` absolute and prefixed all others with the node name. `lodestar_odom::PrivateTopic()` in `utils.h` reproduces exactly that rule by prepending `~/` to relative names only.

## Result Analysis

Odometry estimation results are saved in the eval folder in KITTI format (default: the path given via `--est_directory`). Evaluate with the [evo](https://github.com/MichaelGrupp/evo) tool:

```bash
pipx install evo
evo_traj kitti <est_directory>/01.txt --plot
evo_ape kitti <reference>/01.txt <est_directory>/01.txt
```

Without a reference trajectory, `scripts/check_trajectory.py` verifies the structural properties a correct run must have (finite values, orthonormal rotations, pose count, motion continuity):

```bash
python3 scripts/check_trajectory.py <est_directory>/01.txt --frames <N> --rate 4.0
```

## Sample Radar File

You can download and test the code with the example x-band radar file [Download](https://drive.google.com/drive/folders/12PN696UkMj0rJ62Ug7zIjDkgvIkPOzi1?usp=sharing) (ROS 1 bag — convert it as described above).

## Citation

```
@article{jang2024lodestar,
  title={LodeStar: Maritime Radar Descriptor for Semi-Direct Radar Odometry},
  author={Jang, Hyesu and Jung, Minwoo and Jeon, Myung-Hwan and Kim, Ayoung},
  journal={IEEE Robotics and Automation Letters},
  volume={9},
  number={2},
  pages={1684--1691},
  year={2024},
  publisher={IEEE}
}
```

## Contact

* Hyesu Jang (dortz at snu dot ac dot kr)
