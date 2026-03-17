"""One-shot ROS 2 node that removes the dominant ground plane from a .ply cloud."""

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
    """A RANSAC plane candidate extracted from the cloud."""

    plane_model: np.ndarray
    inlier_indices: list[int]
    point_indices: np.ndarray
    axis_alignment: float
    axis_median: float
    axis_mean: float


class GroundPlaneRemovalNode(Node):
    """Load a point cloud from disk, remove the dominant plane, save the result, and exit."""

    def __init__(self) -> None:
        super().__init__("ground_plane_removal_node")

        self.declare_parameter("input_dir", "")
        self.declare_parameter("recursive_search", True)
        self.declare_parameter("output_suffix", "_ground_removed")
        self.declare_parameter("plane_distance_threshold_m", 0.005)
        self.declare_parameter("plane_ransac_n", 3)
        self.declare_parameter("plane_num_iterations", 2000)
        self.declare_parameter("min_plane_inlier_ratio", 0.20)
        self.declare_parameter("ground_height_axis", "y")
        self.declare_parameter("ground_axis_positive_is_lower", True)
        self.declare_parameter("min_ground_axis_alignment", 0.75)
        self.declare_parameter("ground_plane_height_margin_m", 0.003)
        self.declare_parameter("max_ground_plane_candidates", 5)
        self.declare_parameter("cluster_eps_m", 0.0125)
        self.declare_parameter("cluster_min_points", 8)
        self.declare_parameter("min_cluster_size", 100)
        self.declare_parameter("keep_largest_cluster_only", True)

    def run(self) -> int:
        input_dir_text = str(self.get_parameter("input_dir").value).strip()
        recursive_search = bool(self.get_parameter("recursive_search").value)
        output_suffix = str(self.get_parameter("output_suffix").value)
        distance_threshold_m = float(
            self.get_parameter("plane_distance_threshold_m").value
        )
        ransac_n = int(self.get_parameter("plane_ransac_n").value)
        num_iterations = int(self.get_parameter("plane_num_iterations").value)
        min_plane_inlier_ratio = float(
            self.get_parameter("min_plane_inlier_ratio").value
        )
        ground_height_axis = str(self.get_parameter("ground_height_axis").value).strip()
        ground_axis_positive_is_lower = bool(
            self.get_parameter("ground_axis_positive_is_lower").value
        )
        min_ground_axis_alignment = float(
            self.get_parameter("min_ground_axis_alignment").value
        )
        ground_plane_height_margin_m = float(
            self.get_parameter("ground_plane_height_margin_m").value
        )
        max_ground_plane_candidates = int(
            self.get_parameter("max_ground_plane_candidates").value
        )
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
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return 1

        self.get_logger().info(f"Reading point cloud: {cloud_path}")
        cloud = o3d.io.read_point_cloud(str(cloud_path))
        point_count = len(cloud.points)
        if point_count == 0:
            self.get_logger().error(f"Point cloud is empty: {cloud_path}")
            return 1

        if point_count < ransac_n:
            self.get_logger().error(
                "Point cloud does not have enough points for RANSAC plane fitting."
            )
            return 1

        try:
            axis_index = self._axis_to_index(ground_height_axis)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return 1

        filtered_cloud, plane_summary, selected_plane = self._remove_ground_plane(
            cloud=cloud,
            axis_index=axis_index,
            axis_name=ground_height_axis,
            axis_positive_is_lower=ground_axis_positive_is_lower,
            distance_threshold_m=distance_threshold_m,
            ransac_n=ransac_n,
            num_iterations=num_iterations,
            min_plane_inlier_ratio=min_plane_inlier_ratio,
            min_ground_axis_alignment=min_ground_axis_alignment,
            ground_plane_height_margin_m=ground_plane_height_margin_m,
            max_ground_plane_candidates=max_ground_plane_candidates,
        )
        filtered_point_count = len(filtered_cloud.points)
        if filtered_point_count == 0:
            self.get_logger().error("Plane removal removed all points from the cloud.")
            return 1

        clustered_cloud, cluster_summary, used_cluster_fallback = self._remove_isolated_clusters(
            filtered_cloud=filtered_cloud,
            cluster_eps_m=cluster_eps_m,
            cluster_min_points=cluster_min_points,
            min_cluster_size=min_cluster_size,
            keep_largest_cluster_only=keep_largest_cluster_only,
        )
        clustered_point_count = len(clustered_cloud.points)
        if clustered_point_count == 0:
            self.get_logger().error(
                "Cluster filtering removed all remaining points after plane removal."
            )
            return 1

        output_path = cloud_path.with_name(
            f"{cloud_path.stem}{output_suffix}{cloud_path.suffix}"
        )
        if not o3d.io.write_point_cloud(str(output_path), clustered_cloud):
            self.get_logger().error(f"Failed to write filtered point cloud: {output_path}")
            return 1

        if selected_plane is None:
            self.get_logger().info(
                "No ground-like plane was accepted. "
                f"Input points: {point_count}, "
                f"remaining after plane stage: {filtered_point_count}, "
                f"remaining after cluster filtering: {clustered_point_count}."
            )
        else:
            plane_normal = np.asarray(selected_plane.plane_model[:3], dtype=np.float64)
            plane_normal_norm = np.linalg.norm(plane_normal)
            if plane_normal_norm > 0.0:
                plane_normal = plane_normal / plane_normal_norm

            self.get_logger().info(
                "Ground plane removed successfully. "
                f"Input points: {point_count}, "
                f"plane points removed: {len(selected_plane.inlier_indices)}, "
                f"remaining after plane removal: {filtered_point_count}, "
                f"remaining after cluster filtering: {clustered_point_count}."
            )
            self.get_logger().info(
                "Accepted plane model: "
                f"a={selected_plane.plane_model[0]:.6f}, "
                f"b={selected_plane.plane_model[1]:.6f}, "
                f"c={selected_plane.plane_model[2]:.6f}, "
                f"d={selected_plane.plane_model[3]:.6f}, "
                f"unit_normal={plane_normal.tolist()}"
            )

        self.get_logger().info(plane_summary)
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

    def _remove_ground_plane(
        self,
        cloud: o3d.geometry.PointCloud,
        axis_index: int,
        axis_name: str,
        axis_positive_is_lower: bool,
        distance_threshold_m: float,
        ransac_n: int,
        num_iterations: int,
        min_plane_inlier_ratio: float,
        min_ground_axis_alignment: float,
        ground_plane_height_margin_m: float,
        max_ground_plane_candidates: int,
    ) -> tuple[o3d.geometry.PointCloud, str, PlaneCandidate | None]:
        points = np.asarray(cloud.points)
        cloud_axis_median = float(np.median(points[:, axis_index]))
        working_cloud = cloud
        working_indices = np.arange(len(points), dtype=np.int64)
        candidates: list[PlaneCandidate] = []

        for _ in range(max(max_ground_plane_candidates, 1)):
            if len(working_cloud.points) < max(ransac_n, 50):
                break

            plane_model, inliers = working_cloud.segment_plane(
                distance_threshold=distance_threshold_m,
                ransac_n=ransac_n,
                num_iterations=num_iterations,
            )
            if not inliers:
                break

            point_indices = working_indices[np.asarray(inliers, dtype=np.int64)]
            inlier_points = points[point_indices]
            plane_normal = np.asarray(plane_model[:3], dtype=np.float64)
            plane_normal_norm = np.linalg.norm(plane_normal)
            if plane_normal_norm <= 0.0:
                break
            plane_normal /= plane_normal_norm

            candidates.append(
                PlaneCandidate(
                    plane_model=np.asarray(plane_model, dtype=np.float64),
                    inlier_indices=list(point_indices.tolist()),
                    point_indices=point_indices,
                    axis_alignment=float(abs(plane_normal[axis_index])),
                    axis_median=float(np.median(inlier_points[:, axis_index])),
                    axis_mean=float(np.mean(inlier_points[:, axis_index])),
                )
            )

            working_cloud = working_cloud.select_by_index(inliers, invert=True)
            retained_mask = np.ones(len(working_indices), dtype=bool)
            retained_mask[np.asarray(inliers, dtype=np.int64)] = False
            working_indices = working_indices[retained_mask]

        accepted: list[PlaneCandidate] = []
        candidate_summaries: list[str] = []
        for index, candidate in enumerate(candidates):
            inlier_ratio = len(candidate.inlier_indices) / max(len(points), 1)
            height_ok = (
                candidate.axis_median >= cloud_axis_median + ground_plane_height_margin_m
                if axis_positive_is_lower
                else candidate.axis_median <= cloud_axis_median - ground_plane_height_margin_m
            )
            accepted_candidate = (
                inlier_ratio >= min_plane_inlier_ratio
                and candidate.axis_alignment >= min_ground_axis_alignment
                and height_ok
            )
            candidate_summaries.append(
                f"candidate_{index}: points={len(candidate.inlier_indices)}, "
                f"ratio={inlier_ratio:.1%}, "
                f"{axis_name}_alignment={candidate.axis_alignment:.3f}, "
                f"{axis_name}_median={candidate.axis_median:.4f}, "
                f"accepted={accepted_candidate}"
            )
            if accepted_candidate:
                accepted.append(candidate)

        if not accepted:
            summary = (
                "No ground-like plane candidate passed validation. "
                f"cloud_{axis_name}_median={cloud_axis_median:.4f}. "
                + " ".join(candidate_summaries)
            )
            return cloud, summary, None

        selected_plane = max(
            accepted,
            key=lambda candidate: candidate.axis_median
            if axis_positive_is_lower
            else -candidate.axis_median,
        )
        filtered_cloud = cloud.select_by_index(selected_plane.inlier_indices, invert=True)
        summary = (
            "Ground plane candidate selection: "
            f"cloud_{axis_name}_median={cloud_axis_median:.4f}. "
            + " ".join(candidate_summaries)
        )
        return filtered_cloud, summary, selected_plane

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
