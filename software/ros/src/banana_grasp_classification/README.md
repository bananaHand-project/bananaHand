# banana_grasp_classification

Point-cloud preprocessing and rule-based grasp classification for BananaHand.

This package contains three ROS 2 nodes:
- `ground_plane_removal_node`
- `grasp_rule_classifier_node`
- `scan_grasp_pipeline_node`

It also contains two convenience launch files:
- `grasp_pipeline.launch.py`
- `scan_to_grasp.launch.py`

The intended workflow is:
1. start from a reconstructed downsampled object cloud
2. remove the table / ground plane when one is detectable
3. classify the remaining object cloud into an initial grasp type
4. save both the processed `.ply` and a detailed grasp JSON beside it

The scan-to-grasp path keeps the existing one-shot tools intact:
- `banana_3d_reconstruction/object_scan_node` still captures and saves the scan folder
- `scan_grasp_pipeline_node` listens for each completed scan folder, runs ground removal, runs classification, then publishes the selected grasp/opening as a typed ROS topic

## Reference Results

The scan/result set used during classifier testing is stored in:
- `resource/banana_scans`

Those folders mirror the test objects we ran through the pipeline and keep the saved outputs beside each scan, including:
- ground-removed `.ply` clouds
- grasp-classification `.json` results

## Nodes

### `ground_plane_removal_node`

This node searches an input directory for a reconstructed point cloud, fits candidate support planes with RANSAC on a downsampled copy, scores those candidates using:
- inlier ratio
- alignment with the configured height axis
- whether the plane lies toward the expected table side of the cloud

If a valid table-like plane is found, the node:
- keeps only points on the object side of that plane
- applies DBSCAN cluster filtering
- optionally applies statistical outlier removal
- writes `<input_stem>_ground_removed.ply`

If no valid table-like plane is found, the node does not fail the pipeline anymore. It writes an unchanged passthrough copy using the same output suffix so downstream classification can still run.

Supported input filenames:
- `final_output_cloud_downsampled.ply`
- `final_object_cloud_downsampled.ply`

Default output filename:
- `final_object_cloud_downsampled_ground_removed.ply`

### `grasp_rule_classifier_node`

This node searches for an already ground-removed cloud, computes geometric descriptors, runs a deterministic scored rules engine, writes a JSON result beside the cloud, and exits.

Supported input filenames by default:
- `final_output_cloud_downsampled_ground_removed.ply`
- `final_object_cloud_downsampled_ground_removed.ply`

Default output filename:
- `final_object_cloud_downsampled_ground_removed_grasp.json`

The classifier is rule-based only. It does not use ML inference.

Grip classes:
- `pinch`
- `tripod`
- `cylindrical`
- `spherical`
- `hook`

`hook` remains in the output but is strongly suppressed with an explicit penalty so it almost never wins unless the evidence is unusually strong.

### `scan_grasp_pipeline_node`

This node subscribes to the completed-scan directory topic from `banana_3d_reconstruction`, then runs:
1. `ground_plane_removal_node`
2. `grasp_rule_classifier_node`

After the grasp JSON is written, it publishes:
- `/grasp_classification/recommendation` (`banana_interfaces/msg/GraspRecommendation`)

Message fields:
- `selected_grip`
- `recommended_opening_m`

### `grasp_executor`

This node subscribes to:
- `/grasp_classification/recommendation` (`banana_interfaces/msg/GraspRecommendation`)
- `/grasp_classification/execute_grasp` (`std_msgs/msg/Empty`)
- `/grasp_classification/release` (`std_msgs/msg/Empty`)

It publishes:
- `/tx_positions` (`std_msgs/msg/UInt16MultiArray`)

Behavior:
- caches the latest command derived from `selected_grip` and `recommended_opening_m`
- waits for an execute trigger before publishing that cached command
- publishes the open pose and clears the cached grasp when a release trigger arrives
- keeps thumb revolve at the grip-specific preset
- linearly interpolates the other actuators from an "open" pose at `0.15 m` to the full grip pose at `0.03 m`
- clamps openings outside that range to the nearest endpoint

By default it listens on:
- `/object_scan/completed_scan_dir` (`std_msgs/msg/String`)

## Classifier Behavior

The classifier is designed for partial single-view RealSense object clouds. It uses:
- conservative statistical cleanup
- optional radius outlier cleanup
- robust PCA extents using percentiles
- oriented bounding box extents
- normal reuse or adaptive estimation
- local surface variation as a direct flat-vs-curved surface cue
- normal-direction concentration
- trimmed cylinder fitting
- least-squares sphere fitting with a centroid-offset sanity check
- explicit box-like vs cylinder-like vs sphere-like intermediate evidence
- explicit per-grip score terms saved into JSON

Additional classifier details reflected in the current code:
- `surface_variation_median` plays a direct role in separating flat box faces from curved cylindrical or spherical surfaces
- `normal_estimation_radius_m` is an upper-bound cap, not a fixed radius; the actual radius is scaled from the object's minor extent
- sphere fits that drift too far from the cloud centroid are rejected so partial hemispheres do not create unrealistic large sphere diameters
- compact round objects use an explicit small-sphere cue so ping-pong-ball-scale objects favor `tripod` instead of falling through to `pinch`
- larger round objects require both strong round-fit evidence and size before `spherical` gets boosted
- cylindrical power grasps get extra support from tall curved bodies, while medium / large box-like prisms still map to `cylindrical`
- `pinch` depends on a genuinely small usable body span, not just a tiny shell depth, so wide hollow objects like cups do not collapse to precision grasps
- compact non-spherical objects with both small height and small width get an explicit `pinch` boost, which helps cases like small rounded cases that are not truly spherical
- `small_body_dimension_max_m` defaults to `0.06`, and is used as the explicit body-size split for the compact-object rules
- sphere-like objects with curved surfaces, similar height/width, and both dimensions below that 6 cm split get an explicit `tripod` boost
- sphere-like objects with curved surfaces and both height/width at or above that 6 cm split get extra support for `spherical`
- non-spherical objects whose visible width and thickness both stay below that 6 cm split get an explicit `pinch` boost, which keeps thin small cylinders and compact cases out of wrap grasps
- mildly taller-than-wide round shells that are large enough for a wrap grasp but not sphere-dominant get an extra cylindrical bias, which keeps cups and similar hollow objects from being treated as spheres
- tall curved bodies whose height is noticeably larger than their width lose spherical support and gain cylindrical support, which keeps cups and similar objects from being treated as spheres
- when that tall curved-body rule wins, the cylindrical opening uses the visible cylindrical diameter span instead of the sphere-fit diameter
- the JSON output intentionally keeps the `single_view_partial_cloud` assumption enabled for this capture pipeline

Important practical behavior:
- large cereal box / Lego box -> usually `cylindrical` using a thickness-like span
- bottle / mug / spray bottle / olive oil bottle -> usually `cylindrical`
- orange / apple -> usually `spherical` if large enough
- ping pong ball / lime -> usually `tripod`
- glue stick / small box / dice -> usually `pinch`

The recommended opening is:
- `recommended_opening_m = grasp_span_basis_m + opening_margin_m`
- for round-like `cylindrical` or `spherical` grasps, `grasp_span_basis_m` uses the cloud's `width_like` span by default
- box-like power grasps still keep their existing thickness-like span logic

By default:
- `opening_margin_m = 0.03`

If `max_hand_opening_m > 0`, the final opening is clamped.

## Build

```bash
cd /home/dbhaumik/BananaHand/software/ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  banana_interfaces banana_3d_reconstruction banana_grasp_classification
source install/setup.bash
```

## Run Ground Plane Removal

```bash
ros2 run banana_grasp_classification ground_plane_removal_node --ros-args \
  -p input_dir:=/home/dbhaumik/banana_scans/cup
```

Useful ground-removal parameters:
- `input_dir`: directory to search
- `recursive_search`: whether to search subdirectories, default `true`
- `output_suffix`: output stem suffix, default `_ground_removed`
- `ground_height_axis`: axis treated as height, one of `x`, `y`, `z`
- `table_is_positive_direction_along_axis`: whether the table is toward the positive direction of that axis
- `min_ground_axis_alignment`: minimum normal alignment with the chosen axis
- `voxel_size_m`: downsampling used only for plane fitting
- `plane_distance_threshold_m`: RANSAC inlier threshold
- `plane_ransac_n`
- `plane_num_iterations`
- `max_ground_plane_candidates`
- `min_plane_inlier_ratio`
- `plane_clearance_above_table_m`: clearance margin above the detected table plane
- `remove_statistical_outliers`
- `sor_nb_neighbors`
- `sor_std_ratio`
- `cluster_eps_m`
- `cluster_min_points`
- `min_cluster_size`
- `keep_largest_cluster_only`

Notes:
- if no valid plane is found, the node writes a passthrough copy and returns success
- if DBSCAN rejects everything, the node keeps the current cloud instead of destroying it

## Run Rule-Based Grasp Classification

```bash
ros2 run banana_grasp_classification grasp_rule_classifier_node --ros-args \
  -p input_dir:=/home/dbhaumik/banana_scans/cup
```

Useful classifier parameters:
- `input_dir`: directory to search
- `recursive_search`: whether to search subdirectories, default `true`
- `ground_removed_suffix`: required suffix for the classifier input, default `_ground_removed`
- `output_suffix`: JSON output suffix, default `_grasp`
- `opening_margin_m`
- `max_hand_opening_m`
- `small_object_max_span_m`
- `tripod_object_max_span_m`
- `power_grasp_min_span_m`
- `small_body_dimension_max_m`
- `spherical_extent_ratio_max`
- `spherical_fit_error_max_m`
- `cylindrical_radius_cv_max`
- `cylindrical_fit_error_max_m`
- `box_flatness_ratio_max`
- `box_power_grasp_min_major_m`
- `box_power_grasp_min_middle_m`
- `hook_score_penalty`

Cleanup and descriptor parameters:
- `statistical_nb_neighbors`
- `statistical_std_ratio`
- `radius_outlier_nb_points`
- `radius_outlier_radius_m`
- `use_radius_outlier_filter`
- `extent_percentile_low`
- `extent_percentile_high`
- `surface_variation_k_neighbors`
- `normal_estimation_radius_m`: upper cap for adaptive normal estimation radius
- `normal_estimation_max_nn`
- `cylinder_axis_trim_percent`
- `cylinder_radial_outlier_percent`
- `sphere_fit_inlier_percentile`

Note:
- this list is for running `grasp_rule_classifier_node` directly
- `scan_grasp_pipeline_node` and `scan_to_grasp.launch.py` currently forward only:
  `opening_margin_m`, `max_hand_opening_m`, `small_object_max_span_m`,
  `tripod_object_max_span_m`, and `power_grasp_min_span_m`
- parameters like `small_body_dimension_max_m` still use the classifier's built-in default when you run the end-to-end scan-to-grasp launch

## Combined Launch Pipeline

`grasp_pipeline.launch.py` runs ground removal first and then classification in one launch command.

```bash
ros2 launch banana_grasp_classification grasp_pipeline.launch.py \
  input_dir:=/home/dbhaumik/banana_scans/cup
```

Exposed launch arguments:
- `input_dir`
- `recursive_search`
- `output_suffix`: ground-removal suffix, default `_ground_removed`
- `classifier_output_suffix`: classifier JSON suffix, default `_grasp`
- `opening_margin_m`
- `max_hand_opening_m`
- `small_object_max_span_m`
- `tripod_object_max_span_m`
- `power_grasp_min_span_m`

Behavior:
- if ground removal finds a table-like plane, the classifier uses the filtered cloud
- if ground removal does not find a valid plane, the classifier still runs on the passthrough `_ground_removed.ply`

## End-to-End Scan To Grasp

`scan_to_grasp.launch.py` starts the scan node and automatically runs ground removal plus grasp classification after each completed scan.

Unlike `grasp_pipeline.launch.py`, this launch is meant to stay running until the operator stops it. You can keep the launch open and repeatedly trigger new scans with:

```bash
ros2 service call /object_scan/start_scan std_srvs/srv/Trigger "{}"
```

Each successful scan save publishes a new scan directory on `/object_scan/completed_scan_dir`, and the full post-processing pipeline runs again for that new scan.

```bash
ros2 launch banana_grasp_classification scan_to_grasp.launch.py
```

Useful overrides:

```bash
ros2 launch banana_grasp_classification scan_to_grasp.launch.py \
  output_dir:=/tmp/banana_scans \
  show_preview:=true \
  enable_roi_selection:=true \
  result_topic:=/grasp_classification/recommendation
```

Exposed launch arguments:
- `params_file`
- `show_preview`
- `enable_roi_selection`
- `device_serial`
- `output_dir`
- `result_topic`
- `ground_removed_suffix`
- `classifier_output_suffix`
- `opening_margin_m`
- `max_hand_opening_m`
- `small_object_max_span_m`
- `tripod_object_max_span_m`
- `power_grasp_min_span_m`

These are the only classifier-threshold arguments currently exposed through the end-to-end launch. Other classifier parameters, including `small_body_dimension_max_m`, are not forwarded by `scan_grasp_pipeline_node` right now and therefore stay at their node defaults during `scan_to_grasp.launch.py`.

Behavior:
- when `/object_scan/start_scan` finishes successfully, the scan node saves the usual `.ply` and metadata files
- when ROI selection is enabled, the preview starts with a full-frame ROI and you can drag a box so only depth points inside that ROI feed the scan
- the scan node then publishes the saved scan directory on `/object_scan/completed_scan_dir`
- `scan_grasp_pipeline_node` immediately runs the same ground-removal and classification executables used by the manual pipeline
- after the grasp JSON is saved, the pipeline node publishes `/grasp_classification/recommendation`

## JSON Output

The classifier writes:
- `<input_stem>_grasp.json`

The JSON is intentionally verbose and includes:
- selected grip
- confidence
- recommended opening
- grasp span basis
- all grip scores
- intermediate family evidence
- object family label
- decision path
- per-grip score terms
- PCA and OBB dimensions
- cylinder and sphere fit metrics
- local surface descriptors
- point cloud cleanup statistics
- assumptions used by the classifier
- parameter values used to produce the result

Example output pair:
- `final_object_cloud_downsampled_ground_removed.ply`
- `final_object_cloud_downsampled_ground_removed_grasp.json`
