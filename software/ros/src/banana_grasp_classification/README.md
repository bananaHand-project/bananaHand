# banana_grasp_classification

Point-cloud preprocessing and rule-based grasp classification for BananaHand.

The package currently contains two one-shot ROS 2 nodes:
- `ground_plane_removal_node`: finds a downsampled reconstruction `.ply`, removes a support-like plane with RANSAC when one is found, filters isolated DBSCAN clusters, writes a new `.ply`, and exits. If no valid support plane is found, it writes an unchanged passthrough copy with the configured ground-removed suffix so downstream steps can still run.
- `grasp_rule_classifier_node`: finds a ground-removed `.ply`, extracts robust geometry descriptors, scores the five grip classes with explicit rules, writes a detailed JSON result, and exits.

Supported inputs:

Ground-plane removal node:
- `final_output_cloud_downsampled.ply`
- `final_object_cloud_downsampled.ply`

Rule-based classifier node by default:
- `final_output_cloud_downsampled_ground_removed.ply`
- `final_object_cloud_downsampled_ground_removed.ply`

Default outputs:
- `final_object_cloud_downsampled_ground_removed.ply`
- `final_object_cloud_downsampled_ground_removed_grasp.json`

The classifier writes the JSON beside the input `.ply` using:
- `<input_stem>_grasp.json`

The classifier is deterministic and rule-based. It does not use ML inference. It scores:
- `pinch`
- `tripod`
- `cylindrical`
- `spherical`
- `hook`

`hook` remains in the output but is heavily suppressed by an explicit penalty so it is very unlikely to win.

## Build

```bash
cd software/ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select banana_grasp_classification
source install/setup.bash
```

## Run Ground Plane Removal

```bash
ros2 run banana_grasp_classification ground_plane_removal_node --ros-args \
  -p input_dir:=/home/dbhaumik/banana_scans/cup
```

Useful parameters:
- `input_dir`: root directory to search
- `recursive_search`: search subdirectories recursively
- `output_suffix`: suffix added before `.ply` in the output filename
- `plane_distance_threshold_m`: RANSAC inlier distance threshold
- `min_plane_inlier_ratio`: minimum inlier ratio required to accept a ground-like plane
- `ground_height_axis`: axis used as height for support-plane validation
- `ground_axis_positive_is_lower`: when `true`, larger axis values are treated as lower/closer to the table
- `min_ground_axis_alignment`: candidate plane normal must align with the height axis by at least this amount
- `ground_plane_height_margin_m`: plane must lie on the low side of the cloud by at least this margin
- `max_ground_plane_candidates`: number of dominant planes to inspect before giving up
- `cluster_eps_m`: DBSCAN neighbor radius
- `cluster_min_points`: minimum local density for DBSCAN clusters
- `min_cluster_size`: minimum accepted cluster size after DBSCAN
- `keep_largest_cluster_only`: keep only the largest surviving cluster

## Run Rule-Based Grasp Classification

```bash
ros2 run banana_grasp_classification grasp_rule_classifier_node --ros-args \
  -p input_dir:=/home/dbhaumik/banana_scans/cup
```

The classifier expects a ground-removed input by default. It performs:
- conservative statistical and optional radius outlier cleanup
- robust PCA extents using percentiles
- oriented bounding box extents
- local surface variation and normal-dispersion measurements
- trimmed cylinder-like fitting for partial bottle / mug style clouds
- least-squares sphere fitting for compact round objects
- explicit per-grip rule scoring and JSON serialization

Important classifier parameters:
- `input_dir`: root directory to search
- `recursive_search`: search subdirectories recursively
- `ground_removed_suffix`: required suffix on the input `.ply` stem, default `_ground_removed`
- `output_suffix`: JSON suffix added to the input stem, default `_grasp`
- `opening_margin_m`: added to the selected grasp span basis, default `0.03`
- `max_hand_opening_m`: optional clamp for the final recommended opening, `0.0` disables clamping
- `small_object_max_span_m`: upper span for clear precision-object evidence
- `tripod_object_max_span_m`: round-object size threshold where tripod is preferred over spherical
- `power_grasp_min_span_m`: lower span threshold for power-grasp evidence
- `spherical_extent_ratio_max`: extent-similarity threshold for spherical evidence
- `spherical_fit_error_max_m`: sphere-fit error threshold
- `cylindrical_radius_cv_max`: cylinder radial-variation threshold
- `cylindrical_fit_error_max_m`: cylinder fit error threshold
- `box_flatness_ratio_max`: flatness threshold that contributes to box-like evidence
- `box_power_grasp_min_major_m`: major-axis size threshold for large box-like power grasps
- `box_power_grasp_min_middle_m`: middle-axis size threshold for large box-like power grasps
- `hook_score_penalty`: explicit suppression prior applied to hook

Cleanup and descriptor parameters:
- `statistical_nb_neighbors`
- `statistical_std_ratio`
- `radius_outlier_nb_points`
- `radius_outlier_radius_m`
- `use_radius_outlier_filter`
- `extent_percentile_low`
- `extent_percentile_high`
- `surface_variation_k_neighbors`
- `normal_estimation_radius_m`
- `normal_estimation_max_nn`
- `cylinder_axis_trim_percent`
- `cylinder_radial_outlier_percent`
- `sphere_fit_inlier_percentile`

## Combined Launch Pipeline

`grasp_pipeline.launch.py` runs the existing ground-plane removal node first and only starts the classifier after it exits.

```bash
ros2 launch banana_grasp_classification grasp_pipeline.launch.py \
  input_dir:=/home/dbhaumik/banana_scans/cup
```

Useful launch arguments:
- `input_dir`
- `recursive_search`
- `output_suffix`: ground-removal suffix, default `_ground_removed`
- `classifier_output_suffix`: JSON suffix, default `_grasp`
- `opening_margin_m`
- `max_hand_opening_m`
- `small_object_max_span_m`
- `tripod_object_max_span_m`
- `power_grasp_min_span_m`

## Output JSON

The classifier JSON is intentionally verbose so the decision stays inspectable. It includes:
- selected grip and confidence
- recommended initial hand opening
- grasp-span basis and opening margin
- per-grip scores
- intermediate family evidence
- decision path and score terms
- PCA and OBB dimensions
- cylinder and sphere descriptor values
- local surface descriptors
- point cloud stats and cleanup counts
- parameter values used for the decision

Practical intended behavior:
- large cereal box / Lego box -> `cylindrical` using thickness-like span
- bottle / olive oil bottle / spray bottle / mug -> `cylindrical` using estimated body diameter when available
- orange / apple -> `spherical` when large enough
- ping pong ball / lime -> `tripod`
- glue stick / small box / dice -> `pinch`
- `hook` almost never wins
