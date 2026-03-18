"""ROS 2 node that removes a table/ground plane from a .ply cloud."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import rclpy
from rclpy.node import Node

try:
    import open3d as o3d
except ImportError as exc:
    raise SystemExit("open3d is not installed. Try: pip install open3d") from exc


SUPPORTED_FILENAMES = (
    "final_output_cloud_downsampled.ply",
    "final_object_cloud_downsampled.ply",
)


@dataclass(frozen=True)
class PlaneCandidate:
    plane_model: np.ndarray
    inlier_count: int
    inlier_ratio: float
    axis_alignment: float
    axis_median: float
    extremeness: float
    score: float


class GroundPlaneRemovalNode(Node):
    """Load a point cloud from disk, remove the table plane, save the result, and exit."""

    def __init__(self) -> None:
        super().__init__("ground_plane_removal_node")

        self.declare_parameter("input_dir", "")
        self.declare_parameter("recursive_search", True)
        self.declare_parameter("output_suffix", "_ground_removed")

        # Frame / axis settings
        self.declare_parameter("ground_height_axis", "y")
        self.declare_parameter("table_is_positive_direction_along_axis", True)
        self.declare_parameter("min_ground_axis_alignment", 0.85)

        # Plane fitting
        self.declare_parameter("voxel_size_m", 0.003)
        self.declare_parameter("plane_distance_threshold_m", 0.006)
        self.declare_parameter("plane_ransac_n", 3)
        self.declare_parameter("plane_num_iterations", 3000)
        self.declare_parameter("max_ground_plane_candidates", 5)
        self.declare_parameter("min_plane_inlier_ratio", 0.05)

        # Actual removal margin above the plane.
        # Increase this to remove more table fuzz; decrease it to preserve more of the object's bottom.
        self.declare_parameter("plane_clearance_above_table_m", 0.003)

        # Cleanup
        self.declare_parameter("remove_statistical_outliers", True)
        self.declare_parameter("sor_nb_neighbors", 20)
        self.declare_parameter("sor_std_ratio", 2.0)

        # Cluster filtering
        self.declare_parameter("cluster_eps_m", 0.012)
        self.declare_parameter("cluster_min_points", 10)
        self.declare_parameter("min_cluster_size", 80)
        self.declare_parameter("keep_largest_cluster_only", True)

    def run(self) -> int:
        input_dir_text = str(self.get_parameter("input_dir").value).strip()
        recursive_search = bool(self.get_parameter("recursive_search").value)
        output_suffix = str(self.get_parameter("output_suffix").value)

        ground_height_axis = str(self.get_parameter("ground_height_axis").value).strip()
        table_is_positive_direction_along_axis = bool(
            self.get_parameter("table_is_positive_direction_along_axis").value
        )
        min_ground_axis_alignment = float(
            self.get_parameter("min_ground_axis_alignment").value
        )

        voxel_size_m = float(self.get_parameter("voxel_size_m").value)
        plane_distance_threshold_m = float(
            self.get_parameter("plane_distance_threshold_m").value
        )
        plane_ransac_n = int(self.get_parameter("plane_ransac_n").value)
        plane_num_iterations = int(self.get_parameter("plane_num_iterations").value)
        max_ground_plane_candidates = int(
            self.get_parameter("max_ground_plane_candidates").value
        )
        min_plane_inlier_ratio = float(
            self.get_parameter("min_plane_inlier_ratio").value
        )
        plane_clearance_above_table_m = float(
            self.get_parameter("plane_clearance_above_table_m").value
        )

        remove_statistical_outliers = bool(
            self.get_parameter("remove_statistical_outliers").value
        )
        sor_nb_neighbors = int(self.get_parameter("sor_nb_neighbors").value)
        sor_std_ratio = float(self.get_parameter("sor_std_ratio").value)

        cluster_eps_m = float(self.get_parameter("cluster_eps_m").value)
        cluster_min_points = int(self.get_parameter("cluster_min_points").value)
        min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        keep_largest_cluster_only = bool(
            self.get_parameter("keep_largest_cluster_only").value
        )

        if not input_dir_text:
            self.get_logger().error("Parameter 'input_dir' is required.")
            return 1

        input_dir = Path(input_dir_text).expanduser()
        if not input_dir.is_dir():
            self.get_logger().error(f"Input directory does not exist: {input_dir}")
            return 1

        try:
            cloud_path = self._find_cloud_path(input_dir, recursive_search)
            axis_index = self._axis_to_index(ground_height_axis)
        except (RuntimeError, ValueError) as exc:
            self.get_logger().error(str(exc))
            return 1

        self.get_logger().info(f"Reading point cloud: {cloud_path}")
        cloud = o3d.io.read_point_cloud(str(cloud_path))
        cloud = cloud.remove_non_finite_points()
        cloud = cloud.remove_duplicated_points()

        input_points = len(cloud.points)
        if input_points < max(plane_ransac_n, 50):
            self.get_logger().error(
                f"Point cloud has too few valid points after cleanup: {input_points}"
            )
            return 1

        # Fit plane on a downsampled copy for speed/stability, but classify/remove on the original cloud.
        plane_fit_cloud = cloud
        if voxel_size_m > 0.0:
            plane_fit_cloud = cloud.voxel_down_sample(voxel_size_m)

        if len(plane_fit_cloud.points) < max(plane_ransac_n, 50):
            self.get_logger().error(
                "Point cloud is too small after optional downsampling for plane fitting."
            )
            return 1

        selected_plane, plane_summary = self._select_ground_plane(
            cloud=plane_fit_cloud,
            axis_index=axis_index,
            table_is_positive_direction_along_axis=table_is_positive_direction_along_axis,
            distance_threshold_m=plane_distance_threshold_m,
            ransac_n=plane_ransac_n,
            num_iterations=plane_num_iterations,
            max_ground_plane_candidates=max_ground_plane_candidates,
            min_plane_inlier_ratio=min_plane_inlier_ratio,
            min_ground_axis_alignment=min_ground_axis_alignment,
        )

        output_path = cloud_path.with_name(
            f"{cloud_path.stem}{output_suffix}{cloud_path.suffix}"
        )
        if selected_plane is None:
            if not o3d.io.write_point_cloud(str(output_path), cloud):
                self.get_logger().error(
                    "No valid table-like plane was found, and writing the passthrough "
                    f"cloud failed: {output_path}"
                )
                return 1

            self.get_logger().warn(
                "No valid table-like plane was found. "
                "Saving an unchanged copy of the input cloud with the configured "
                f"ground-removed suffix instead. {plane_summary}"
            )
            self.get_logger().info(
                f"Saved passthrough point cloud to: {output_path}"
            )
            return 0

        object_cloud, side_summary = self._extract_points_on_object_side_of_plane(
            cloud=cloud,
            plane_model=selected_plane.plane_model,
            axis_index=axis_index,
            table_is_positive_direction_along_axis=table_is_positive_direction_along_axis,
            clearance_m=plane_clearance_above_table_m,
        )
        after_plane_points = len(object_cloud.points)
        if after_plane_points == 0:
            self.get_logger().error(
                "Plane-sided filtering removed all points. "
                "Lower plane_clearance_above_table_m or verify axis/frame settings."
            )
            return 1

        clustered_cloud, cluster_summary, used_cluster_fallback = self._remove_isolated_clusters(
            filtered_cloud=object_cloud,
            cluster_eps_m=cluster_eps_m,
            cluster_min_points=cluster_min_points,
            min_cluster_size=min_cluster_size,
            keep_largest_cluster_only=keep_largest_cluster_only,
        )

        if remove_statistical_outliers and len(clustered_cloud.points) > sor_nb_neighbors:
            before_sor = len(clustered_cloud.points)
            clustered_cloud, _ = clustered_cloud.remove_statistical_outlier(
                nb_neighbors=sor_nb_neighbors,
                std_ratio=sor_std_ratio,
                print_progress=False,
            )
            self.get_logger().info(
                "Statistical outlier removal: "
                f"nb_neighbors={sor_nb_neighbors}, std_ratio={sor_std_ratio:.3f}, "
                f"removed_points={before_sor - len(clustered_cloud.points)}."
            )

        output_points = len(clustered_cloud.points)
        if output_points == 0:
            self.get_logger().error("Post-processing removed all points.")
            return 1

        if not o3d.io.write_point_cloud(str(output_path), clustered_cloud):
            self.get_logger().error(f"Failed to write filtered point cloud: {output_path}")
            return 1

        plane_unit = np.asarray(selected_plane.plane_model[:3], dtype=np.float64)
        plane_unit /= np.linalg.norm(plane_unit)

        self.get_logger().info(
            "Ground plane removed successfully. "
            f"input_points={input_points}, "
            f"after_plane_side_filter={after_plane_points}, "
            f"final_points={output_points}, "
            f"selected_plane_score={selected_plane.score:.3f}, "
            f"selected_plane_inlier_ratio={selected_plane.inlier_ratio:.1%}, "
            f"selected_plane_alignment={selected_plane.axis_alignment:.3f}, "
            f"selected_plane_axis_median={selected_plane.axis_median:.5f}."
        )
        self.get_logger().info(
            "Selected plane model: "
            f"a={selected_plane.plane_model[0]:.6f}, "
            f"b={selected_plane.plane_model[1]:.6f}, "
            f"c={selected_plane.plane_model[2]:.6f}, "
            f"d={selected_plane.plane_model[3]:.6f}, "
            f"unit_normal={plane_unit.tolist()}"
        )
        self.get_logger().info(plane_summary)
        self.get_logger().info(side_summary)
        if used_cluster_fallback:
            self.get_logger().warn(cluster_summary)
        else:
            self.get_logger().info(cluster_summary)
        self.get_logger().info(f"Saved filtered point cloud to: {output_path}")
        return 0

    def _find_cloud_path(self, input_dir: Path, recursive_search: bool) -> Path:
        matches: list[Path] = []
        for candidate_name in SUPPORTED_FILENAMES:
            matches.extend(self._glob_matches(input_dir, candidate_name, recursive_search))

        unique_matches = sorted({path.resolve() for path in matches})
        if not unique_matches:
            supported = ", ".join(SUPPORTED_FILENAMES)
            raise RuntimeError(
                f"No supported point-cloud file found under {input_dir}. "
                f"Expected one of: {supported}"
            )

        if len(unique_matches) > 1:
            listed = ", ".join(str(path) for path in unique_matches)
            raise RuntimeError(
                "Found multiple candidate point clouds. Narrow 'input_dir' or disable "
                f"recursive search. Matches: {listed}"
            )

        return unique_matches[0]

    def _glob_matches(
        self, input_dir: Path, filename: str, recursive_search: bool
    ) -> Iterable[Path]:
        pattern = f"**/{filename}" if recursive_search else filename
        return input_dir.glob(pattern)

    def _axis_to_index(self, axis_name: str) -> int:
        axis_lookup = {"x": 0, "y": 1, "z": 2}
        try:
            return axis_lookup[axis_name.lower()]
        except KeyError as exc:
            raise ValueError(
                f"Unsupported ground_height_axis '{axis_name}'. Expected one of x, y, z."
            ) from exc

    def _score_candidate(
        self,
        axis_median: float,
        axis_min: float,
        axis_max: float,
        inlier_ratio: float,
        axis_alignment: float,
        table_is_positive_direction_along_axis: bool,
    ) -> tuple[float, float]:
        axis_span = max(axis_max - axis_min, 1e-9)
        extremeness = (
            (axis_median - axis_min) / axis_span
            if table_is_positive_direction_along_axis
            else (axis_max - axis_median) / axis_span
        )
        extremeness = float(np.clip(extremeness, 0.0, 1.0))

        # Heavier weight on support, then alignment, then being near the "table side" of the cloud.
        score = float((2.0 * inlier_ratio) + (1.0 * axis_alignment) + (1.0 * extremeness))
        return extremeness, score

    def _select_ground_plane(
        self,
        cloud: o3d.geometry.PointCloud,
        axis_index: int,
        table_is_positive_direction_along_axis: bool,
        distance_threshold_m: float,
        ransac_n: int,
        num_iterations: int,
        max_ground_plane_candidates: int,
        min_plane_inlier_ratio: float,
        min_ground_axis_alignment: float,
    ) -> tuple[PlaneCandidate | None, str]:
        points = np.asarray(cloud.points)
        axis_values = points[:, axis_index]
        axis_min = float(np.min(axis_values))
        axis_max = float(np.max(axis_values))

        working_cloud = cloud
        candidates: list[PlaneCandidate] = []
        candidate_text: list[str] = []

        total_points = len(points)
        for candidate_index in range(max(max_ground_plane_candidates, 1)):
            if len(working_cloud.points) < max(ransac_n, 50):
                break

            plane_model, inliers = working_cloud.segment_plane(
                distance_threshold=distance_threshold_m,
                ransac_n=ransac_n,
                num_iterations=num_iterations,
            )
            if not inliers:
                break

            inlier_points = np.asarray(working_cloud.points)[np.asarray(inliers, dtype=np.int64)]
            normal = np.asarray(plane_model[:3], dtype=np.float64)
            normal_norm = np.linalg.norm(normal)
            if normal_norm <= 0.0:
                break
            normal /= normal_norm

            inlier_count = len(inliers)
            inlier_ratio = inlier_count / max(total_points, 1)
            axis_alignment = float(abs(normal[axis_index]))
            axis_median = float(np.median(inlier_points[:, axis_index]))
            extremeness, score = self._score_candidate(
                axis_median=axis_median,
                axis_min=axis_min,
                axis_max=axis_max,
                inlier_ratio=inlier_ratio,
                axis_alignment=axis_alignment,
                table_is_positive_direction_along_axis=table_is_positive_direction_along_axis,
            )

            accepted = (
                inlier_ratio >= min_plane_inlier_ratio
                and axis_alignment >= min_ground_axis_alignment
            )
            candidate_text.append(
                f"candidate_{candidate_index}: points={inlier_count}, "
                f"ratio={inlier_ratio:.1%}, alignment={axis_alignment:.3f}, "
                f"axis_median={axis_median:.5f}, extremeness={extremeness:.3f}, "
                f"score={score:.3f}, accepted={accepted}"
            )

            if accepted:
                candidates.append(
                    PlaneCandidate(
                        plane_model=np.asarray(plane_model, dtype=np.float64),
                        inlier_count=inlier_count,
                        inlier_ratio=inlier_ratio,
                        axis_alignment=axis_alignment,
                        axis_median=axis_median,
                        extremeness=extremeness,
                        score=score,
                    )
                )

            # Remove this candidate plane and look for the next one.
            working_cloud = working_cloud.select_by_index(inliers, invert=True)

        if not candidates:
            summary = "No valid table-like plane was found. " + " ".join(candidate_text)
            return None, summary

        selected = max(candidates, key=lambda candidate: candidate.score)
        summary = (
            "Ground plane candidate selection: "
            f"axis_min={axis_min:.5f}, axis_max={axis_max:.5f}. "
            + " ".join(candidate_text)
        )
        return selected, summary

    def _orient_plane_normal_toward_object(
        self,
        plane_model: np.ndarray,
        axis_index: int,
        table_is_positive_direction_along_axis: bool,
    ) -> np.ndarray:
        plane = np.asarray(plane_model, dtype=np.float64).copy()
        normal_norm = np.linalg.norm(plane[:3])
        if normal_norm <= 0.0:
            raise ValueError("Plane normal has zero length.")

        plane /= normal_norm

        # We want signed_distance > 0 on the object side.
        # If the table is toward the positive axis direction, the object side is toward the negative axis direction.
        desired_sign = -1.0 if table_is_positive_direction_along_axis else 1.0

        if abs(plane[axis_index]) > 1e-12:
            if np.sign(plane[axis_index]) != np.sign(desired_sign):
                plane *= -1.0

        return plane

    def _extract_points_on_object_side_of_plane(
        self,
        cloud: o3d.geometry.PointCloud,
        plane_model: np.ndarray,
        axis_index: int,
        table_is_positive_direction_along_axis: bool,
        clearance_m: float,
    ) -> tuple[o3d.geometry.PointCloud, str]:
        plane = self._orient_plane_normal_toward_object(
            plane_model=plane_model,
            axis_index=axis_index,
            table_is_positive_direction_along_axis=table_is_positive_direction_along_axis,
        )

        points = np.asarray(cloud.points)
        signed_distances = points @ plane[:3] + plane[3]

        # Keep only points strictly on the object side of the plane,
        # with a small clearance margin to kill table fuzz/noise.
        keep_mask = signed_distances > max(clearance_m, 0.0)
        keep_indices = np.flatnonzero(keep_mask).tolist()
        filtered_cloud = cloud.select_by_index(keep_indices)

        summary = (
            "Plane-sided filtering: "
            f"clearance_m={clearance_m:.5f}, "
            f"kept_points={len(keep_indices)}, "
            f"removed_points={len(points) - len(keep_indices)}, "
            f"min_signed_distance={float(np.min(signed_distances)):.5f}, "
            f"max_signed_distance={float(np.max(signed_distances)):.5f}."
        )
        return filtered_cloud, summary

    def _remove_isolated_clusters(
        self,
        filtered_cloud: o3d.geometry.PointCloud,
        cluster_eps_m: float,
        cluster_min_points: int,
        min_cluster_size: int,
        keep_largest_cluster_only: bool,
    ) -> tuple[o3d.geometry.PointCloud, str, bool]:
        if len(filtered_cloud.points) == 0:
            return filtered_cloud, "Cluster filtering skipped: no points remain.", True

        labels = np.asarray(
            filtered_cloud.cluster_dbscan(
                eps=cluster_eps_m,
                min_points=cluster_min_points,
                print_progress=False,
            )
        )
        valid_mask = labels >= 0
        if not np.any(valid_mask):
            return (
                filtered_cloud,
                "Cluster filtering found no valid clusters with the current DBSCAN settings; "
                "keeping the plane-filtered cloud unchanged.",
                True,
            )

        valid_labels = labels[valid_mask]
        unique_labels, counts = np.unique(valid_labels, return_counts=True)
        large_labels = unique_labels[counts >= min_cluster_size]
        if large_labels.size == 0:
            return (
                filtered_cloud,
                "Cluster filtering rejected all clusters as too small; "
                f"largest cluster had {int(np.max(counts))} points, below "
                f"min_cluster_size={min_cluster_size}. Keeping the plane-filtered cloud unchanged.",
                True,
            )

        kept_labels = large_labels
        if keep_largest_cluster_only:
            largest_index = int(np.argmax(counts))
            kept_labels = np.asarray([unique_labels[largest_index]])

        kept_mask = np.isin(labels, kept_labels)
        kept_indices = np.flatnonzero(kept_mask).tolist()
        kept_cloud = filtered_cloud.select_by_index(kept_indices)

        cluster_sizes = ", ".join(
            f"{int(label)}:{int(count)}" for label, count in zip(unique_labels, counts)
        )
        kept_labels_text = ", ".join(str(int(label)) for label in kept_labels)
        summary = (
            "Cluster filtering summary: "
            f"clusters={len(unique_labels)}, sizes=[{cluster_sizes}], "
            f"kept_labels=[{kept_labels_text}], "
            f"discarded_points={len(filtered_cloud.points) - len(kept_cloud.points)}."
        )
        return kept_cloud, summary, False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GroundPlaneRemovalNode()
    exit_code = 1
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)
