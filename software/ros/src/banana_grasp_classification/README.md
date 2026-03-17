# banana_grasp_classification

Initial point-cloud preprocessing package for BananaHand grasp classification.

Current node:
- `ground_plane_removal_node`: finds a downsampled reconstruction `.ply`, removes the dominant plane with RANSAC, filters isolated point clusters with DBSCAN, and writes a new `.ply` beside the input file.

Supported input filenames:
- `final_output_cloud_downsampled.ply`
- `final_object_cloud_downsampled.ply`

Build:
```bash
cd software/ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select banana_grasp_classification
source install/setup.bash
```

Run:
```bash
ros2 run banana_grasp_classification ground_plane_removal_node --ros-args \
  -p input_dir:=/home/dbhaumik/banana_scans/cup
```

Useful parameters:
- `input_dir`: root directory to search
- `recursive_search`: search subdirectories recursively (`true` by default)
- `output_suffix`: suffix added before `.ply` in the output filename
- `plane_distance_threshold_m`: RANSAC inlier distance threshold
- `min_plane_inlier_ratio`: minimum inlier ratio required to accept the detected plane
- `ground_height_axis`: axis used as height for support-plane validation (`y` by default)
- `ground_axis_positive_is_lower`: when `true`, larger axis values are treated as lower/closer to the table
- `min_ground_axis_alignment`: candidate plane normal must align with the height axis by at least this amount
- `ground_plane_height_margin_m`: how far into the lower half of the cloud a plane must sit before it is accepted
- `max_ground_plane_candidates`: how many dominant planes to inspect before giving up
- `cluster_eps_m`: DBSCAN neighbor radius in meters
- `cluster_min_points`: minimum local density for DBSCAN clusters
- `min_cluster_size`: minimum accepted cluster size after DBSCAN
- `keep_largest_cluster_only`: keep only the largest surviving cluster (`true` by default)

Default output for the sample mug:
- `final_object_cloud_downsampled_ground_removed.ply`
