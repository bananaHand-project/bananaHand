"""Reusable RGB-D point cloud helpers for burst-mode tabletop object scanning."""

from __future__ import annotations

import copy
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

try:
    import open3d as o3d
except ImportError as exc:
    raise SystemExit("open3d is not installed. Try: pip install open3d") from exc


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float


@dataclass
class ClusterCandidate:
    label: int
    point_cloud: o3d.geometry.PointCloud
    centroid: np.ndarray
    image_centroid: np.ndarray
    bbox_uv: tuple[int, int, int, int]
    extent_xyz: np.ndarray
    point_count: int
    score: float
    degenerate: bool
    degeneracy_reason: str
    metrics: dict[str, Any]


@dataclass
class SegmentationResult:
    raw_cloud: o3d.geometry.PointCloud
    plane_removed_cloud: o3d.geometry.PointCloud
    selected_cloud: Optional[o3d.geometry.PointCloud]
    selected_cluster: Optional[ClusterCandidate]
    clusters: list[ClusterCandidate]
    plane_model: Optional[np.ndarray]
    selection_reason: str


@dataclass
class RegistrationResult:
    transformation: np.ndarray
    fitness: float
    inlier_rmse: float


def camera_intrinsics_from_realsense(rs_intrinsics: object) -> CameraIntrinsics:
    """Convert RealSense intrinsics to a serializable helper object."""
    return CameraIntrinsics(
        width=int(rs_intrinsics.width),
        height=int(rs_intrinsics.height),
        fx=float(rs_intrinsics.fx),
        fy=float(rs_intrinsics.fy),
        ppx=float(rs_intrinsics.ppx),
        ppy=float(rs_intrinsics.ppy),
    )


def scale_camera_intrinsics(
    intrinsics: CameraIntrinsics, width: int, height: int
) -> CameraIntrinsics:
    """Scale intrinsics when a filtered depth image changes resolution."""
    scale_x = float(width) / max(float(intrinsics.width), 1.0)
    scale_y = float(height) / max(float(intrinsics.height), 1.0)
    return CameraIntrinsics(
        width=int(width),
        height=int(height),
        fx=intrinsics.fx * scale_x,
        fy=intrinsics.fy * scale_y,
        ppx=intrinsics.ppx * scale_x,
        ppy=intrinsics.ppy * scale_y,
    )


def resolve_output_dir(path_text: str) -> Path:
    """Expand a user-provided output directory path."""
    return Path(path_text).expanduser()


def point_count(cloud: o3d.geometry.PointCloud) -> int:
    """Return the number of points in an Open3D point cloud."""
    return len(cloud.points)


def clone_cloud(cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """Deep-copy an Open3D point cloud."""
    return copy.deepcopy(cloud)


def voxel_downsample(
    cloud: o3d.geometry.PointCloud, voxel_size_m: float
) -> o3d.geometry.PointCloud:
    """Voxel-downsample a point cloud when the voxel size is positive."""
    if point_count(cloud) == 0 or voxel_size_m <= 0.0:
        return clone_cloud(cloud)
    return cloud.voxel_down_sample(voxel_size_m)


def depth_image_to_point_cloud(
    color_image_bgr: np.ndarray,
    depth_image_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    min_depth_m: float,
    max_depth_m: float,
) -> o3d.geometry.PointCloud:
    """Create a colored Open3D point cloud directly from aligned RGB-D images."""
    depth = depth_image_m.astype(np.float32, copy=False)
    valid_mask = np.isfinite(depth)
    valid_mask &= depth >= min_depth_m
    valid_mask &= depth <= max_depth_m

    cloud = o3d.geometry.PointCloud()
    if not np.any(valid_mask):
        return cloud

    ys, xs = np.nonzero(valid_mask)
    z = depth[ys, xs].astype(np.float64)
    x = (xs.astype(np.float64) - intrinsics.ppx) / intrinsics.fx * z
    y = (ys.astype(np.float64) - intrinsics.ppy) / intrinsics.fy * z
    points = np.column_stack((x, y, z))
    colors_rgb = color_image_bgr[ys, xs][:, ::-1].astype(np.float64) / 255.0

    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors_rgb)
    return cloud


def project_points(
    points_xyz: np.ndarray, intrinsics: CameraIntrinsics
) -> np.ndarray:
    """Project 3D camera-frame points into image coordinates."""
    if points_xyz.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    z = points_xyz[:, 2]
    uv = np.empty((points_xyz.shape[0], 2), dtype=np.float64)
    uv[:, 0] = intrinsics.fx * (points_xyz[:, 0] / np.maximum(z, 1e-6)) + intrinsics.ppx
    uv[:, 1] = intrinsics.fy * (points_xyz[:, 1] / np.maximum(z, 1e-6)) + intrinsics.ppy
    return uv


def project_point(point_xyz: np.ndarray, intrinsics: CameraIntrinsics) -> np.ndarray:
    """Project a single 3D point into image coordinates."""
    return project_points(point_xyz.reshape(1, 3), intrinsics)[0]


def orient_plane_toward_camera(plane_model: np.ndarray) -> np.ndarray:
    """Flip a plane so the camera origin lies on the positive side."""
    plane = plane_model.astype(np.float64, copy=True)
    if plane[3] < 0.0:
        plane *= -1.0
    return plane


def _cluster_bbox(
    cluster_cloud: o3d.geometry.PointCloud, intrinsics: CameraIntrinsics
) -> tuple[int, int, int, int]:
    points = np.asarray(cluster_cloud.points)
    if points.size == 0:
        return (0, 0, 0, 0)

    uv = project_points(points, intrinsics)
    uv[:, 0] = np.clip(uv[:, 0], 0.0, intrinsics.width - 1.0)
    uv[:, 1] = np.clip(uv[:, 1], 0.0, intrinsics.height - 1.0)
    return (
        int(np.floor(np.min(uv[:, 0]))),
        int(np.floor(np.min(uv[:, 1]))),
        int(np.ceil(np.max(uv[:, 0]))),
        int(np.ceil(np.max(uv[:, 1]))),
    )


def _cluster_extent_xyz(cluster_cloud: o3d.geometry.PointCloud) -> np.ndarray:
    if point_count(cluster_cloud) == 0:
        return np.zeros(3, dtype=np.float64)
    aabb = cluster_cloud.get_axis_aligned_bounding_box()
    return np.asarray(aabb.get_extent(), dtype=np.float64)


def _degeneracy_check(
    extent_xyz: np.ndarray,
    point_count_value: int,
    dbscan_min_points: int,
) -> tuple[bool, str]:
    sorted_extent = np.sort(np.asarray(extent_xyz, dtype=np.float64))
    smallest = float(sorted_extent[0])
    middle = float(sorted_extent[1])
    largest = float(sorted_extent[2])

    reasons: list[str] = []
    if largest < 0.020:
        reasons.append("tiny_cluster")
    if smallest < 0.003 and middle < 0.012:
        reasons.append("thin_two_dimensions")

    middle_safe = max(middle, 1e-4)
    smallest_safe = max(smallest, 1e-4)
    if largest / middle_safe > 10.0 and middle / smallest_safe > 3.0 and smallest < 0.010:
        reasons.append("line_like")

    if point_count_value < max(int(dbscan_min_points * 1.10), 70) and middle < 0.015:
        reasons.append("sparse_fragment")

    return bool(reasons), "+".join(reasons) if reasons else "ok"


def _extent_reasonable_score(extent_xyz: np.ndarray) -> float:
    sorted_extent = np.sort(np.asarray(extent_xyz, dtype=np.float64))
    smallest = float(sorted_extent[0])
    middle = float(sorted_extent[1])
    largest = float(sorted_extent[2])

    def bell(value: float, target: float, width: float) -> float:
        return math.exp(-(((value - target) / max(width, 1e-4)) ** 2))

    score = 0.0
    score += 0.9 * bell(largest, 0.18, 0.18)
    score += 0.8 * bell(middle, 0.08, 0.08)
    score += 0.4 * bell(max(smallest, 0.0), 0.03, 0.05)
    if largest > 0.60:
        score -= 1.0
    return float(score)


def cluster_candidate_to_dict(candidate: ClusterCandidate) -> dict[str, Any]:
    return {
        "label": int(candidate.label),
        "centroid": candidate.centroid.tolist(),
        "image_centroid": candidate.image_centroid.tolist(),
        "bbox_uv": [int(value) for value in candidate.bbox_uv],
        "extent_xyz": candidate.extent_xyz.tolist(),
        "point_count": int(candidate.point_count),
        "score": float(candidate.score),
        "degenerate": bool(candidate.degenerate),
        "degeneracy_reason": candidate.degeneracy_reason,
        "metrics": candidate.metrics,
    }


def _evaluate_cluster_candidate(
    label: int,
    candidate_cloud: o3d.geometry.PointCloud,
    intrinsics: CameraIntrinsics,
    dbscan_min_points: int,
) -> ClusterCandidate:
    points = np.asarray(candidate_cloud.points)
    centroid = np.mean(points, axis=0)
    image_centroid = project_point(centroid, intrinsics)
    bbox_uv = _cluster_bbox(candidate_cloud, intrinsics)
    extent_xyz = _cluster_extent_xyz(candidate_cloud)

    image_center = np.array(
        [intrinsics.width / 2.0, intrinsics.height / 2.0], dtype=np.float64
    )
    center_distance_px = float(np.linalg.norm(image_centroid - image_center))
    center_radius_px = max(0.45 * min(intrinsics.width, intrinsics.height), 1.0)
    center_score = math.exp(-((center_distance_px / center_radius_px) ** 2))

    size_score = min(3.0, len(points) / max(float(dbscan_min_points * 3), 1.0))
    extent_score = _extent_reasonable_score(extent_xyz)
    degenerate, degeneracy_reason = _degeneracy_check(
        extent_xyz=extent_xyz,
        point_count_value=len(points),
        dbscan_min_points=dbscan_min_points,
    )

    score = 1.8 * size_score + 1.6 * center_score + 1.0 * extent_score
    if degenerate:
        score -= 4.0

    metrics = {
        "size_score": float(size_score),
        "center_distance_px": float(center_distance_px),
        "center_score": float(center_score),
        "extent_reasonable_score": float(extent_score),
    }
    return ClusterCandidate(
        label=label,
        point_cloud=clone_cloud(candidate_cloud),
        centroid=centroid,
        image_centroid=image_centroid,
        bbox_uv=bbox_uv,
        extent_xyz=extent_xyz,
        point_count=point_count(candidate_cloud),
        score=float(score),
        degenerate=bool(degenerate),
        degeneracy_reason=degeneracy_reason,
        metrics=metrics,
    )


def segment_tabletop_object(
    raw_cloud: o3d.geometry.PointCloud,
    intrinsics: CameraIntrinsics,
    plane_distance_threshold_m: float,
    min_object_height_above_plane_m: float,
    dbscan_eps_m: float,
    dbscan_min_points: int,
    voxel_size_m: float,
    plane_ransac_iterations: int = 300,
) -> SegmentationResult:
    """Fit the tabletop plane on the full frame and pick the best object-like cluster."""
    working_voxel = max(voxel_size_m * 0.5, plane_distance_threshold_m * 0.75, 0.0015)
    working_cloud = voxel_downsample(raw_cloud, working_voxel)

    empty_cloud = o3d.geometry.PointCloud()
    result = SegmentationResult(
        raw_cloud=working_cloud,
        plane_removed_cloud=empty_cloud,
        selected_cloud=None,
        selected_cluster=None,
        clusters=[],
        plane_model=None,
        selection_reason="no_points",
    )

    if point_count(working_cloud) < max(dbscan_min_points, 20):
        return result

    plane_removed_cloud = working_cloud
    plane_model_np: Optional[np.ndarray] = None
    try:
        plane_model, _ = working_cloud.segment_plane(
            distance_threshold=plane_distance_threshold_m,
            ransac_n=3,
            num_iterations=max(int(plane_ransac_iterations), 50),
        )
        plane_model_np = orient_plane_toward_camera(
            np.asarray(plane_model, dtype=np.float64)
        )
        signed_distance = (
            np.asarray(working_cloud.points) @ plane_model_np[:3] + plane_model_np[3]
        )
        above_indices = np.flatnonzero(signed_distance > min_object_height_above_plane_m)
        if above_indices.size > 0:
            plane_removed_cloud = working_cloud.select_by_index(above_indices.tolist())
            result.selection_reason = "plane_removed"
        else:
            result.selection_reason = "plane_removed_empty"
    except RuntimeError:
        plane_model_np = None
        result.selection_reason = "plane_fit_failed"

    result.plane_model = plane_model_np
    result.plane_removed_cloud = plane_removed_cloud

    if point_count(plane_removed_cloud) < max(dbscan_min_points, 20):
        return result

    labels = np.asarray(
        plane_removed_cloud.cluster_dbscan(
            eps=dbscan_eps_m,
            min_points=dbscan_min_points,
            print_progress=False,
        )
    )
    if labels.size == 0:
        result.selection_reason = "dbscan_failed"
        return result

    candidates: list[ClusterCandidate] = []
    for label in sorted({int(label) for label in labels if label >= 0}):
        cluster_indices = np.flatnonzero(labels == label)
        if cluster_indices.size < dbscan_min_points:
            continue
        cluster_cloud = plane_removed_cloud.select_by_index(cluster_indices.tolist())
        candidates.append(
            _evaluate_cluster_candidate(
                label=label,
                candidate_cloud=cluster_cloud,
                intrinsics=intrinsics,
                dbscan_min_points=dbscan_min_points,
            )
        )

    candidates.sort(key=lambda item: (item.score, item.point_count), reverse=True)
    result.clusters = candidates
    if not candidates:
        result.selection_reason = "no_clusters"
        return result

    valid_candidates = [candidate for candidate in candidates if not candidate.degenerate]
    if not valid_candidates:
        result.selection_reason = "all_candidates_degenerate"
        return result

    selected_candidate = max(
        valid_candidates, key=lambda item: (item.score, item.point_count)
    )
    result.selected_cluster = selected_candidate
    result.selected_cloud = clone_cloud(selected_candidate.point_cloud)
    result.selection_reason = "largest_nondegenerate_near_center"
    return result


def _estimate_normals(cloud: o3d.geometry.PointCloud, voxel_size_m: float) -> None:
    radius = max(voxel_size_m * 2.5, 0.01)
    cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30)
    )


def register_cloud_icp(
    source_cloud: o3d.geometry.PointCloud,
    target_cloud: o3d.geometry.PointCloud,
    voxel_size_m: float,
    max_correspondence_distance_m: float,
    initial_transform: Optional[np.ndarray] = None,
) -> Optional[RegistrationResult]:
    """Register one segmented object cloud against another with conservative ICP."""
    if point_count(source_cloud) < 20 or point_count(target_cloud) < 20:
        return None

    source_ds = voxel_downsample(source_cloud, max(voxel_size_m, 0.0015))
    target_ds = voxel_downsample(target_cloud, max(voxel_size_m, 0.0015))
    if point_count(source_ds) < 20 or point_count(target_ds) < 20:
        return None

    _estimate_normals(source_ds, voxel_size_m)
    _estimate_normals(target_ds, voxel_size_m)

    transform0 = (
        np.eye(4, dtype=np.float64)
        if initial_transform is None
        else np.asarray(initial_transform, dtype=np.float64)
    )

    try:
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        coarse = o3d.pipelines.registration.registration_icp(
            source_ds,
            target_ds,
            max_correspondence_distance_m * 2.0,
            transform0,
            estimation,
        )
        fine = o3d.pipelines.registration.registration_icp(
            source_ds,
            target_ds,
            max_correspondence_distance_m,
            coarse.transformation,
            estimation,
        )
    except RuntimeError:
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        coarse = o3d.pipelines.registration.registration_icp(
            source_ds,
            target_ds,
            max_correspondence_distance_m * 2.0,
            transform0,
            estimation,
        )
        fine = o3d.pipelines.registration.registration_icp(
            source_ds,
            target_ds,
            max_correspondence_distance_m,
            coarse.transformation,
            estimation,
        )

    return RegistrationResult(
        transformation=np.asarray(fine.transformation, dtype=np.float64),
        fitness=float(fine.fitness),
        inlier_rmse=float(fine.inlier_rmse),
    )


def transform_cloud_copy(
    cloud: o3d.geometry.PointCloud, transform: np.ndarray
) -> o3d.geometry.PointCloud:
    """Return a transformed copy of a point cloud."""
    transformed = clone_cloud(cloud)
    transformed.transform(np.asarray(transform, dtype=np.float64))
    return transformed


def extract_largest_cluster(
    cloud: o3d.geometry.PointCloud, eps_m: float, min_points: int
) -> o3d.geometry.PointCloud:
    """Keep only the largest connected cluster in a point cloud."""
    if point_count(cloud) < max(min_points, 10):
        return clone_cloud(cloud)

    labels = np.asarray(
        cloud.cluster_dbscan(
            eps=max(eps_m, 1e-4),
            min_points=max(min_points, 5),
            print_progress=False,
        )
    )
    valid_labels = [int(label) for label in labels if label >= 0]
    if not valid_labels:
        return clone_cloud(cloud)

    largest_label = max(set(valid_labels), key=valid_labels.count)
    indices = np.flatnonzero(labels == largest_label)
    if indices.size == 0:
        return clone_cloud(cloud)
    return cloud.select_by_index(indices.tolist())


def postprocess_object_cloud(
    cloud: o3d.geometry.PointCloud,
    voxel_size_m: float,
    dbscan_eps_m: float,
    dbscan_min_points: int,
    plane_distance_threshold_m: float,
    min_object_height_above_plane_m: float,
    plane_ransac_iterations: int = 300,
) -> o3d.geometry.PointCloud:
    """Apply aggressive cleanup to the fused burst cloud."""
    processed = clone_cloud(cloud)
    if point_count(processed) == 0:
        return processed

    try:
        plane_model, _ = processed.segment_plane(
            distance_threshold=plane_distance_threshold_m,
            ransac_n=3,
            num_iterations=max(int(plane_ransac_iterations), 50),
        )
        plane_model_np = orient_plane_toward_camera(
            np.asarray(plane_model, dtype=np.float64)
        )
        signed_distance = (
            np.asarray(processed.points) @ plane_model_np[:3] + plane_model_np[3]
        )
        above_indices = np.flatnonzero(
            signed_distance > min_object_height_above_plane_m
        )
        retained_fraction = above_indices.size / max(point_count(processed), 1)
        if (
            above_indices.size >= max(dbscan_min_points, 20)
            and retained_fraction >= 0.50
            and retained_fraction < 0.98
        ):
            processed = processed.select_by_index(above_indices.tolist())
    except RuntimeError:
        pass

    processed = voxel_downsample(processed, voxel_size_m)
    if point_count(processed) == 0:
        return processed

    processed, _ = processed.remove_statistical_outlier(
        nb_neighbors=20,
        std_ratio=1.8,
    )
    if point_count(processed) == 0:
        return processed

    processed, _ = processed.remove_radius_outlier(
        nb_points=max(8, dbscan_min_points // 4),
        radius=max(dbscan_eps_m * 0.80, voxel_size_m * 3.5),
    )
    if point_count(processed) == 0:
        return processed

    processed = extract_largest_cluster(
        processed,
        eps_m=max(dbscan_eps_m, voxel_size_m * 2.5),
        min_points=max(10, dbscan_min_points // 2),
    )
    if point_count(processed) > 0:
        _estimate_normals(processed, voxel_size_m)
    return processed


def compute_cloud_metadata(cloud: o3d.geometry.PointCloud) -> dict:
    """Compute metadata that is useful for later geometric classification."""
    points = np.asarray(cloud.points)
    metadata = {
        "point_count": int(points.shape[0]),
        "centroid": [0.0, 0.0, 0.0],
        "aabb": None,
        "obb": None,
    }
    if points.size == 0:
        return metadata

    centroid = np.mean(points, axis=0)
    aabb = cloud.get_axis_aligned_bounding_box()
    metadata["centroid"] = centroid.tolist()
    metadata["aabb"] = {
        "min_bound": np.asarray(aabb.min_bound).tolist(),
        "max_bound": np.asarray(aabb.max_bound).tolist(),
        "extent": np.asarray(aabb.get_extent()).tolist(),
    }

    try:
        obb = cloud.get_oriented_bounding_box(robust=True)
        metadata["obb"] = {
            "center": np.asarray(obb.center).tolist(),
            "extent": np.asarray(obb.extent).tolist(),
            "rotation": np.asarray(obb.R).tolist(),
        }
    except RuntimeError:
        metadata["obb"] = None

    return metadata


def write_metadata_json(path: Path, metadata: dict) -> None:
    """Write metadata to JSON with a stable, readable layout."""
    path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_point_cloud(path: Path, cloud: o3d.geometry.PointCloud) -> bool:
    """Persist an Open3D point cloud as PLY."""
    return bool(o3d.io.write_point_cloud(str(path), cloud, write_ascii=False))


def open3d_to_pointcloud2(
    cloud: o3d.geometry.PointCloud, stamp: object, frame_id: str
) -> PointCloud2:
    """Convert an Open3D point cloud into a ROS 2 PointCloud2 message."""
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id

    points = np.asarray(cloud.points, dtype=np.float32)
    if points.size == 0:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return pc2.create_cloud(header, fields, [])

    if cloud.has_colors():
        colors = np.clip(np.asarray(cloud.colors), 0.0, 1.0)
        colors_uint8 = (colors * 255.0).astype(np.uint8)
        packed_rgb = (
            (colors_uint8[:, 0].astype(np.uint32) << 16)
            | (colors_uint8[:, 1].astype(np.uint32) << 8)
            | colors_uint8[:, 2].astype(np.uint32)
        )
        packed_rgb_float = np.array(
            [struct.unpack("f", struct.pack("I", int(value)))[0] for value in packed_rgb],
            dtype=np.float32,
        )
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        rows = np.column_stack((points, packed_rgb_float)).tolist()
        return pc2.create_cloud(header, fields, rows)

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    return pc2.create_cloud(header, fields, points.tolist())
