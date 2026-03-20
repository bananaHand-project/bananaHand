"""ROS 2 node that classifies a ground-removed point cloud into a grasp."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np
import rclpy
from rclpy.node import Node

try:
    import open3d as o3d
except ImportError as exc:
    raise SystemExit("open3d is not installed. Try: pip install open3d") from exc


SUPPORTED_INPUT_STEMS = (
    "final_output_cloud_downsampled",
    "final_object_cloud_downsampled",
)
GRIP_PRIORITY = (
    "cylindrical",
    "spherical",
    "tripod",
    "pinch",
    "hook",
)
EPSILON = 1.0e-9


# Geometry evidence thresholds used to keep object-family labels stable.
BOX_DOMINANCE_FOR_ROUND_SUPPRESSION = 0.70
PLANARITY_FOR_ROUND_SUPPRESSION = 0.45
NORMAL_AXIS_FOR_BOX_CONFIDENCE = 0.92
MIN_BOX_CORNER_RATIO = 0.45


@dataclass(frozen=True)
class CleanupSummary:
    """Conservative cleanup result prior to descriptor extraction."""

    cloud: o3d.geometry.PointCloud
    removed_outliers: int
    statistical_removed: int
    radius_removed: int
    warnings: tuple[str, ...]


@dataclass(frozen=True)
class NormalsBundle:
    """Normals used for local descriptor extraction."""

    normals: np.ndarray
    used_normals_from_file: bool
    recomputed_normals: bool


@dataclass(frozen=True)
class CanonicalDimensions:
    """Robust object dimensions derived from PCA and OBB extents."""

    centroid: np.ndarray
    eigenvalues: np.ndarray
    principal_axes: np.ndarray
    pca_extents: np.ndarray
    obb_extents: np.ndarray
    major_extent_m: float
    middle_extent_m: float
    minor_extent_m: float
    height_like_m: float
    width_like_m: float
    thickness_like_m: float
    small_span_m: float


@dataclass(frozen=True)
class SurfaceDescriptor:
    """Local surface descriptors computed from normals and neighborhoods."""

    surface_variation_median: float
    normal_dispersion: float
    normal_axis_alignment: float
    view_bias: float
    is_partial_view: bool


@dataclass(frozen=True)
class CylinderDescriptor:
    """Partial-view cylinder descriptor based on a trimmed cross-sectional fit."""

    diameter_estimate_m: float
    radius_cv: float
    fit_error_m: float
    radial_outlier_fraction: float


@dataclass(frozen=True)
class SphereDescriptor:
    """Least-squares sphere fit descriptor."""

    center: np.ndarray
    diameter_estimate_m: float
    fit_error_m: float
    normalized_fit_error: float


@dataclass(frozen=True)
class GeometryBundle:
    """All derived geometry descriptors used by the rule engine."""

    point_count: int
    dimensions: CanonicalDimensions
    normals: NormalsBundle
    surface: SurfaceDescriptor
    cylinder: CylinderDescriptor
    sphere: SphereDescriptor
    elongation_ratio: float
    flatness_ratio: float
    mid_to_minor_ratio: float
    linearity: float
    planarity: float
    scattering: float


@dataclass(frozen=True)
class ClassificationResult:
    """Final grasp classification output."""

    selected_grip: str
    confidence: float
    scores: dict[str, float]
    score_terms: dict[str, dict[str, float]]
    family_evidence: dict[str, float]
    object_family: str
    decision_path: tuple[str, ...]
    grasp_span_basis_m: float
    recommended_opening_m: float
    basis_reason: str
    power_span_reference_m: float


def clamp01(value: float) -> float:
    """Clamp a scalar score into [0, 1]."""

    if value <= 0.0:
        return 0.0
    if value >= 1.0:
        return 1.0
    return float(value)


def finite_or(value: float, default: float = 0.0) -> float:
    """Return a finite float or a default fallback."""

    if np.isfinite(value):
        return float(value)
    return float(default)


def safe_div(numerator: float, denominator: float, default: float = 0.0) -> float:
    """Safely divide with a configurable fallback for tiny denominators."""

    if abs(denominator) <= EPSILON:
        return float(default)
    return float(numerator / denominator)


def ramp(value: float, low: float, high: float) -> float:
    """Linear ramp that maps [low, high] into [0, 1]."""

    if high <= low:
        return 1.0 if value >= high else 0.0
    return clamp01((value - low) / (high - low))


def inverse_ramp(value: float, low: float, high: float) -> float:
    """Inverse linear ramp that maps [low, high] into [1, 0]."""

    if high <= low:
        return 1.0 if value <= low else 0.0
    return clamp01((high - value) / (high - low))


def normalize_rows(vectors: np.ndarray) -> np.ndarray:
    """Normalize row vectors while leaving zero rows untouched."""

    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    safe_norms = np.where(norms > EPSILON, norms, 1.0)
    return vectors / safe_norms


def sorted_extents_descending(extents: np.ndarray) -> np.ndarray:
    """Sort extent magnitudes from largest to smallest."""

    return np.sort(np.asarray(extents, dtype=np.float64))[::-1]


def spread_score(value: float, target: float, tolerance: float) -> float:
    """Return a score near 1 when value is close to target and 0 when far away."""

    tolerance = max(float(tolerance), EPSILON)
    return clamp01(1.0 - abs(float(value) - float(target)) / tolerance)


class GraspRuleClassifierNode(Node):
    """Load a ground-removed cloud, classify it, emit JSON, and exit."""

    def __init__(self) -> None:
        super().__init__("grasp_rule_classifier_node")

        self.declare_parameter("input_dir", "")
        self.declare_parameter("recursive_search", True)
        self.declare_parameter("ground_removed_suffix", "_ground_removed")
        self.declare_parameter("output_suffix", "_grasp")

        self.declare_parameter("statistical_nb_neighbors", 20)
        self.declare_parameter("statistical_std_ratio", 2.0)
        self.declare_parameter("radius_outlier_nb_points", 10)
        self.declare_parameter("radius_outlier_radius_m", 0.015)
        self.declare_parameter("use_radius_outlier_filter", True)

        self.declare_parameter("extent_percentile_low", 5.0)
        self.declare_parameter("extent_percentile_high", 95.0)
        self.declare_parameter("surface_variation_k_neighbors", 20)
        # NOTE: normal_estimation_radius_m is used as an upper-bound cap only;
        # the actual search radius is computed adaptively from the object's minor
        # extent so that normals remain meaningful across the full range of object
        # sizes (glue stick ↔ cereal box).
        self.declare_parameter("normal_estimation_radius_m", 0.02)
        self.declare_parameter("normal_estimation_max_nn", 30)
        self.declare_parameter("cylinder_axis_trim_percent", 10.0)
        self.declare_parameter("cylinder_radial_outlier_percent", 95.0)
        self.declare_parameter("sphere_fit_inlier_percentile", 90.0)

        self.declare_parameter("opening_margin_m", 0.03)
        self.declare_parameter("max_hand_opening_m", 0.0)
        self.declare_parameter("small_object_max_span_m", 0.045)
        self.declare_parameter("tripod_object_max_span_m", 0.065)
        self.declare_parameter("power_grasp_min_span_m", 0.045)
        self.declare_parameter("small_body_dimension_max_m", 0.06)
        self.declare_parameter("spherical_extent_ratio_max", 1.30)
        self.declare_parameter("spherical_fit_error_max_m", 0.015)
        self.declare_parameter("cylindrical_radius_cv_max", 0.30)
        self.declare_parameter("cylindrical_fit_error_max_m", 0.020)
        self.declare_parameter("box_flatness_ratio_max", 0.40)
        self.declare_parameter("box_power_grasp_min_major_m", 0.10)
        self.declare_parameter("box_power_grasp_min_middle_m", 0.06)
        self.declare_parameter("hook_score_penalty", 0.60)

    def run(self) -> int:
        input_dir_text = str(self.get_parameter("input_dir").value).strip()
        recursive_search = bool(self.get_parameter("recursive_search").value)
        ground_removed_suffix = str(
            self.get_parameter("ground_removed_suffix").value
        ).strip()
        output_suffix = str(self.get_parameter("output_suffix").value).strip()

        if not input_dir_text:
            self.get_logger().error("Parameter 'input_dir' is required.")
            return 1

        input_dir = Path(input_dir_text).expanduser()
        if not input_dir.is_dir():
            self.get_logger().error(f"Input directory does not exist: {input_dir}")
            return 1

        extent_percentile_low = float(self.get_parameter("extent_percentile_low").value)
        extent_percentile_high = float(
            self.get_parameter("extent_percentile_high").value
        )
        if extent_percentile_low < 0.0 or extent_percentile_high > 100.0:
            self.get_logger().error(
                "Extent percentile parameters must remain within [0, 100]."
            )
            return 1
        if extent_percentile_low >= extent_percentile_high:
            self.get_logger().error(
                "'extent_percentile_low' must be smaller than 'extent_percentile_high'."
            )
            return 1

        threshold_params = {
            "opening_margin_m": float(self.get_parameter("opening_margin_m").value),
            "max_hand_opening_m": float(
                self.get_parameter("max_hand_opening_m").value
            ),
            "small_object_max_span_m": float(
                self.get_parameter("small_object_max_span_m").value
            ),
            "tripod_object_max_span_m": float(
                self.get_parameter("tripod_object_max_span_m").value
            ),
            "power_grasp_min_span_m": float(
                self.get_parameter("power_grasp_min_span_m").value
            ),
            "small_body_dimension_max_m": float(
                self.get_parameter("small_body_dimension_max_m").value
            ),
            "spherical_extent_ratio_max": float(
                self.get_parameter("spherical_extent_ratio_max").value
            ),
            "spherical_fit_error_max_m": float(
                self.get_parameter("spherical_fit_error_max_m").value
            ),
            "cylindrical_radius_cv_max": float(
                self.get_parameter("cylindrical_radius_cv_max").value
            ),
            "cylindrical_fit_error_max_m": float(
                self.get_parameter("cylindrical_fit_error_max_m").value
            ),
            "box_flatness_ratio_max": float(
                self.get_parameter("box_flatness_ratio_max").value
            ),
            "box_power_grasp_min_major_m": float(
                self.get_parameter("box_power_grasp_min_major_m").value
            ),
            "box_power_grasp_min_middle_m": float(
                self.get_parameter("box_power_grasp_min_middle_m").value
            ),
            "hook_score_penalty": float(
                self.get_parameter("hook_score_penalty").value
            ),
        }

        try:
            cloud_path = self._find_cloud_path(
                input_dir=input_dir,
                recursive_search=recursive_search,
                ground_removed_suffix=ground_removed_suffix,
            )
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return 1

        self.get_logger().info(f"Reading point cloud: {cloud_path}")
        original_cloud = o3d.io.read_point_cloud(str(cloud_path))
        input_point_count = len(original_cloud.points)
        if input_point_count == 0:
            self.get_logger().error(f"Point cloud is empty: {cloud_path}")
            return 1
        if input_point_count < 30:
            self.get_logger().error(
                f"Point cloud only has {input_point_count} points; classification would "
                "be unstable."
            )
            return 1

        cleanup = self._cleanup_cloud(original_cloud)
        for warning_text in cleanup.warnings:
            self.get_logger().warn(warning_text)

        geometry = self._compute_geometry_bundle(cleanup.cloud)
        classification = self._classify_grasp(geometry, threshold_params)

        output_path = cloud_path.with_name(f"{cloud_path.stem}{output_suffix}.json")
        output_data = self._build_output_json(
            cloud_path=cloud_path,
            input_point_count=input_point_count,
            cleanup=cleanup,
            geometry=geometry,
            classification=classification,
            threshold_params=threshold_params,
            output_suffix=output_suffix,
            ground_removed_suffix=ground_removed_suffix,
            recursive_search=recursive_search,
        )

        output_path.write_text(
            json.dumps(output_data, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        dimensions = geometry.dimensions
        self.get_logger().info(
            "Canonical extents (major, middle, minor) = "
            f"({dimensions.major_extent_m:.4f}, "
            f"{dimensions.middle_extent_m:.4f}, "
            f"{dimensions.minor_extent_m:.4f}) m"
        )
        self.get_logger().info(
            f"Selected grip: {classification.selected_grip} "
            f"(confidence={classification.confidence:.2f})"
        )
        self.get_logger().info(
            "Recommended initial opening: "
            f"{classification.recommended_opening_m:.4f} m "
            f"(basis={classification.grasp_span_basis_m:.4f} m, "
            f"reason={classification.basis_reason})"
        )
        self.get_logger().info(
            "Decision path: " + " -> ".join(classification.decision_path)
        )
        self.get_logger().info(f"Saved grasp classification JSON to: {output_path}")
        return 0

    def _find_cloud_path(
        self,
        input_dir: Path,
        recursive_search: bool,
        ground_removed_suffix: str,
    ) -> Path:
        matches: list[Path] = []
        for candidate_name in self._supported_filenames(ground_removed_suffix):
            matches.extend(self._glob_matches(input_dir, candidate_name, recursive_search))

        unique_matches = sorted({path.resolve() for path in matches})
        if not unique_matches:
            supported = ", ".join(self._supported_filenames(ground_removed_suffix))
            raise RuntimeError(
                f"No supported ground-removed point-cloud file found under {input_dir}. "
                f"Expected one of: {supported}"
            )

        if len(unique_matches) > 1:
            listed = ", ".join(str(path) for path in unique_matches)
            raise RuntimeError(
                "Found multiple candidate ground-removed point clouds. Narrow "
                f"'input_dir' or disable recursive search. Matches: {listed}"
            )

        return unique_matches[0]

    def _supported_filenames(self, ground_removed_suffix: str) -> tuple[str, ...]:
        return tuple(
            f"{stem}{ground_removed_suffix}.ply" for stem in SUPPORTED_INPUT_STEMS
        )

    def _glob_matches(
        self, input_dir: Path, filename: str, recursive_search: bool
    ) -> Iterable[Path]:
        pattern = f"**/{filename}" if recursive_search else filename
        return input_dir.glob(pattern)

    def _cleanup_cloud(self, cloud: o3d.geometry.PointCloud) -> CleanupSummary:
        warnings: list[str] = []
        working_cloud = cloud
        original_count = len(cloud.points)

        statistical_removed = 0
        radius_removed = 0

        try:
            filtered_cloud, kept_indices = working_cloud.remove_statistical_outlier(
                nb_neighbors=int(self.get_parameter("statistical_nb_neighbors").value),
                std_ratio=float(self.get_parameter("statistical_std_ratio").value),
            )
            kept_count = len(kept_indices)
            removed_count = len(working_cloud.points) - kept_count
            if self._cleanup_step_is_acceptable(
                original_count=len(working_cloud.points),
                kept_count=kept_count,
                minimum_fraction=0.55,
            ):
                working_cloud = filtered_cloud
                statistical_removed = removed_count
            elif removed_count > 0:
                warnings.append(
                    "Skipped statistical outlier filtering because it removed too many "
                    "points for a single-view object cloud."
                )
        except RuntimeError as exc:
            warnings.append(f"Statistical outlier filtering failed; continuing. {exc}")

        if bool(self.get_parameter("use_radius_outlier_filter").value):
            try:
                filtered_cloud, kept_indices = working_cloud.remove_radius_outlier(
                    nb_points=int(
                        self.get_parameter("radius_outlier_nb_points").value
                    ),
                    radius=float(
                        self.get_parameter("radius_outlier_radius_m").value
                    ),
                )
                kept_count = len(kept_indices)
                removed_count = len(working_cloud.points) - kept_count
                if self._cleanup_step_is_acceptable(
                    original_count=len(working_cloud.points),
                    kept_count=kept_count,
                    minimum_fraction=0.65,
                ):
                    working_cloud = filtered_cloud
                    radius_removed = removed_count
                elif removed_count > 0:
                    warnings.append(
                        "Skipped radius outlier filtering because it removed too many "
                        "points for a conservative classification pass."
                    )
            except RuntimeError as exc:
                warnings.append(f"Radius outlier filtering failed; continuing. {exc}")

        removed_total = original_count - len(working_cloud.points)
        if len(working_cloud.points) < 30:
            warnings.append(
                "Cleanup reduced the cloud too aggressively; reverting to the original "
                "ground-removed cloud."
            )
            return CleanupSummary(
                cloud=cloud,
                removed_outliers=0,
                statistical_removed=0,
                radius_removed=0,
                warnings=tuple(warnings),
            )

        return CleanupSummary(
            cloud=working_cloud,
            removed_outliers=removed_total,
            statistical_removed=statistical_removed,
            radius_removed=radius_removed,
            warnings=tuple(warnings),
        )

    def _cleanup_step_is_acceptable(
        self,
        original_count: int,
        kept_count: int,
        minimum_fraction: float,
    ) -> bool:
        return kept_count >= max(30, int(math.ceil(original_count * minimum_fraction)))

    def _compute_geometry_bundle(self, cloud: o3d.geometry.PointCloud) -> GeometryBundle:
        points = np.asarray(cloud.points, dtype=np.float64)
        dimensions = self._compute_canonical_dimensions(cloud, points)
        normals_bundle = self._get_or_estimate_normals(cloud, dimensions)
        surface = self._compute_surface_descriptor(
            points=points,
            normals=normals_bundle.normals,
            principal_axes=dimensions.principal_axes,
        )
        cylinder = self._compute_cylinder_descriptor(points, dimensions)
        sphere = self._compute_sphere_descriptor(points, dimensions)

        eigenvalues = np.maximum(dimensions.eigenvalues, 0.0)
        linearity = safe_div(eigenvalues[0] - eigenvalues[1], eigenvalues[0], 0.0)
        planarity = safe_div(eigenvalues[1] - eigenvalues[2], eigenvalues[0], 0.0)
        scattering = safe_div(eigenvalues[2], eigenvalues[0], 0.0)

        elongation_ratio = safe_div(
            dimensions.major_extent_m,
            dimensions.middle_extent_m,
            default=0.0,
        )
        flatness_ratio = safe_div(
            dimensions.minor_extent_m,
            dimensions.middle_extent_m,
            default=0.0,
        )
        mid_to_minor_ratio = safe_div(
            dimensions.middle_extent_m,
            dimensions.minor_extent_m,
            default=0.0,
        )

        return GeometryBundle(
            point_count=len(points),
            dimensions=dimensions,
            normals=normals_bundle,
            surface=surface,
            cylinder=cylinder,
            sphere=sphere,
            elongation_ratio=elongation_ratio,
            flatness_ratio=flatness_ratio,
            mid_to_minor_ratio=mid_to_minor_ratio,
            linearity=linearity,
            planarity=planarity,
            scattering=scattering,
        )

    def _compute_canonical_dimensions(
        self, cloud: o3d.geometry.PointCloud, points: np.ndarray
    ) -> CanonicalDimensions:
        centroid = np.mean(points, axis=0)
        centered = points - centroid

        covariance = np.cov(centered, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        order = np.argsort(eigenvalues)[::-1]
        eigenvalues = np.maximum(eigenvalues[order], 0.0)
        principal_axes = eigenvectors[:, order]

        projected = centered @ principal_axes
        low_percentile = float(self.get_parameter("extent_percentile_low").value)
        high_percentile = float(self.get_parameter("extent_percentile_high").value)
        lower = np.percentile(projected, low_percentile, axis=0)
        upper = np.percentile(projected, high_percentile, axis=0)
        pca_extents = np.maximum(upper - lower, EPSILON)

        try:
            obb = cloud.get_oriented_bounding_box(robust=True)
        except TypeError:
            obb = cloud.get_oriented_bounding_box()
        except RuntimeError:
            obb = None

        if obb is None:
            obb_extents = pca_extents.copy()
        else:
            obb_extents = np.maximum(
                sorted_extents_descending(np.asarray(obb.extent, dtype=np.float64)),
                EPSILON,
            )

        sorted_pca_extents = sorted_extents_descending(pca_extents)
        major_extent_m = float(sorted_pca_extents[0])
        middle_extent_m = float(sorted_pca_extents[1])
        minor_extent_m = float(sorted_pca_extents[2])

        return CanonicalDimensions(
            centroid=centroid,
            eigenvalues=eigenvalues,
            principal_axes=principal_axes,
            pca_extents=sorted_pca_extents,
            obb_extents=obb_extents,
            major_extent_m=major_extent_m,
            middle_extent_m=middle_extent_m,
            minor_extent_m=minor_extent_m,
            height_like_m=float(max(major_extent_m, obb_extents[0])),
            width_like_m=float(max(middle_extent_m, obb_extents[1])),
            thickness_like_m=float(max(minor_extent_m, obb_extents[2])),
            small_span_m=minor_extent_m,
        )

    def _get_or_estimate_normals(
        self,
        cloud: o3d.geometry.PointCloud,
        dimensions: CanonicalDimensions,
    ) -> NormalsBundle:
        """Estimate surface normals with an adaptive search radius.

        The search radius is computed as a fixed fraction of the object's minor
        extent so that it stays meaningful across the full object-size range —
        from a small glue stick (where a fixed 2 cm radius would span the entire
        object) to a large cereal box (where a very small radius produces noisy
        patches).  The user-supplied ``normal_estimation_radius_m`` acts as an
        upper-bound cap so the behaviour can still be tightened via a parameter
        if needed.
        """
        point_count = len(cloud.points)
        if cloud.has_normals() and len(cloud.normals) == point_count:
            normals = np.asarray(cloud.normals, dtype=np.float64)
            normalized_normals = normalize_rows(normals)
            return NormalsBundle(
                normals=normalized_normals,
                used_normals_from_file=True,
                recomputed_normals=False,
            )

        # Adaptive radius: 35 % of the minor extent, clamped between 8 mm and
        # the user-supplied cap (default 20 mm).
        param_cap_m = float(self.get_parameter("normal_estimation_radius_m").value)
        adaptive_radius_m = float(
            np.clip(dimensions.small_span_m * 0.35, 0.008, param_cap_m)
        )

        cloud_for_normals = o3d.geometry.PointCloud(cloud)
        cloud_for_normals.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=adaptive_radius_m,
                max_nn=int(self.get_parameter("normal_estimation_max_nn").value),
            )
        )
        normals = np.asarray(cloud_for_normals.normals, dtype=np.float64)
        if normals.shape[0] != point_count:
            normals = np.zeros((point_count, 3), dtype=np.float64)

        return NormalsBundle(
            normals=normalize_rows(normals),
            used_normals_from_file=False,
            recomputed_normals=True,
        )

    def _compute_surface_descriptor(
        self,
        points: np.ndarray,
        normals: np.ndarray,
        principal_axes: np.ndarray,
    ) -> SurfaceDescriptor:
        if len(points) < 5:
            return SurfaceDescriptor(
                surface_variation_median=0.0,
                normal_dispersion=0.0,
                normal_axis_alignment=0.0,
                view_bias=1.0,
                is_partial_view=True,
            )

        k_neighbors = int(self.get_parameter("surface_variation_k_neighbors").value)
        k_neighbors = max(5, min(k_neighbors, len(points) - 1))

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        tree = o3d.geometry.KDTreeFlann(cloud)

        sample_count = min(len(points), 2000)
        sample_indices = np.unique(
            np.linspace(0, len(points) - 1, num=sample_count, dtype=np.int64)
        )
        variations: list[float] = []
        for point_index in sample_indices:
            _, neighbor_indices, _ = tree.search_knn_vector_3d(
                points[int(point_index)], k_neighbors + 1
            )
            if len(neighbor_indices) <= 3:
                continue
            neighbor_points = points[np.asarray(neighbor_indices[1:], dtype=np.int64)]
            local_center = np.mean(neighbor_points, axis=0)
            local_centered = neighbor_points - local_center
            local_cov = np.cov(local_centered, rowvar=False)
            local_eigenvalues = np.maximum(
                np.linalg.eigvalsh(local_cov),
                0.0,
            )
            variations.append(
                safe_div(local_eigenvalues[0], np.sum(local_eigenvalues), 0.0)
            )

        surface_variation_median = (
            float(np.median(variations)) if variations else 0.0
        )

        if len(normals) == 0:
            normal_dispersion = 0.0
            normal_axis_alignment = 0.0
        else:
            second_moment = (normals.T @ normals) / max(len(normals), 1)
            normal_eigenvalues = np.linalg.eigvalsh(second_moment)
            normal_dispersion = clamp01(1.0 - float(np.max(normal_eigenvalues)))
            axis_alignment = np.abs(normals @ principal_axes)
            normal_axis_alignment = float(np.median(np.max(axis_alignment, axis=1)))

        centroid = np.mean(points, axis=0)
        centered = points - centroid
        radial_norms = np.linalg.norm(centered, axis=1)
        valid_mask = radial_norms > EPSILON
        if np.any(valid_mask):
            unit_rays = centered[valid_mask] / radial_norms[valid_mask, np.newaxis]
            view_bias = float(np.linalg.norm(np.mean(unit_rays, axis=0)))
        else:
            view_bias = 1.0

        # This pipeline explicitly assumes a single-view partial cloud in its output JSON,
        # so do not let a low-symmetry heuristic override that known capture condition.
        is_partial_view = True if len(points) > 0 else bool(view_bias >= 0.20)

        return SurfaceDescriptor(
            surface_variation_median=surface_variation_median,
            normal_dispersion=normal_dispersion,
            normal_axis_alignment=normal_axis_alignment,
            view_bias=view_bias,
            is_partial_view=is_partial_view,
        )

    def _compute_cylinder_descriptor(
        self, points: np.ndarray, dimensions: CanonicalDimensions
    ) -> CylinderDescriptor:
        centered = points - dimensions.centroid
        projected = centered @ dimensions.principal_axes
        axis_coordinates = projected[:, 0]

        trim_percent = float(self.get_parameter("cylinder_axis_trim_percent").value)
        trim_percent = min(max(trim_percent, 0.0), 45.0)
        low = np.percentile(axis_coordinates, trim_percent)
        high = np.percentile(axis_coordinates, 100.0 - trim_percent)
        central_mask = (axis_coordinates >= low) & (axis_coordinates <= high)
        if np.count_nonzero(central_mask) < 25:
            central_mask = np.ones(len(points), dtype=bool)

        radial_points = projected[central_mask, 1:3]
        initial_fit = self._fit_circle_2d(radial_points)
        if initial_fit is None:
            return CylinderDescriptor(
                diameter_estimate_m=0.0,
                radius_cv=1.0,
                fit_error_m=1.0,
                radial_outlier_fraction=0.0,
            )

        center_2d, _ = initial_fit
        radii = np.linalg.norm(radial_points - center_2d, axis=1)
        if len(radii) < 5:
            return CylinderDescriptor(
                diameter_estimate_m=0.0,
                radius_cv=1.0,
                fit_error_m=1.0,
                radial_outlier_fraction=0.0,
            )

        radial_percentile = float(
            self.get_parameter("cylinder_radial_outlier_percent").value
        )
        radial_percentile = min(max(radial_percentile, 50.0), 100.0)
        cutoff = np.percentile(radii, radial_percentile)
        inlier_mask = radii <= cutoff
        if np.count_nonzero(inlier_mask) >= 12:
            refined_fit = self._fit_circle_2d(radial_points[inlier_mask])
            if refined_fit is not None:
                center_2d, _ = refined_fit
                radii = np.linalg.norm(radial_points - center_2d, axis=1)
                cutoff = np.percentile(radii, radial_percentile)
                inlier_mask = radii <= cutoff

        inlier_radii = radii[inlier_mask]
        if len(inlier_radii) < 5:
            inlier_radii = radii

        radius_estimate = float(np.median(inlier_radii))
        fit_error_m = float(
            np.sqrt(np.mean(np.square(inlier_radii - radius_estimate)))
        )
        radius_cv = safe_div(
            float(np.std(inlier_radii)),
            float(np.mean(inlier_radii)),
            default=1.0,
        )
        radial_outlier_fraction = safe_div(
            float(len(radii) - np.count_nonzero(inlier_mask)),
            float(len(radii)),
            default=0.0,
        )

        return CylinderDescriptor(
            diameter_estimate_m=max(2.0 * radius_estimate, 0.0),
            radius_cv=max(radius_cv, 0.0),
            fit_error_m=max(fit_error_m, 0.0),
            radial_outlier_fraction=clamp01(radial_outlier_fraction),
        )

    def _fit_circle_2d(self, points_2d: np.ndarray) -> tuple[np.ndarray, float] | None:
        if len(points_2d) < 3:
            return None

        design = np.column_stack(
            (2.0 * points_2d[:, 0], 2.0 * points_2d[:, 1], np.ones(len(points_2d)))
        )
        target = np.sum(np.square(points_2d), axis=1)
        try:
            solution, _, _, _ = np.linalg.lstsq(design, target, rcond=None)
        except np.linalg.LinAlgError:
            return None

        center = solution[:2]
        radius_sq = float(solution[2] + np.dot(center, center))
        if radius_sq <= EPSILON or not np.isfinite(radius_sq):
            return None

        return center, math.sqrt(radius_sq)

    def _compute_sphere_descriptor(
        self,
        points: np.ndarray,
        dimensions: CanonicalDimensions,
    ) -> SphereDescriptor:
        """Fit a sphere and validate that the inferred center is plausible.

        For single-view partial hemispheres the algebraic least-squares system
        is ill-conditioned: the fit center can land far outside the visible shell,
        producing a radius that bears little relation to the real object diameter.
        The sanity check below rejects fits whose center drifts more than 60 % of
        the major extent away from the cloud centroid, returning a zero-diameter
        sentinel so that downstream callers fall back gracefully.
        """
        initial_fit = self._fit_sphere(points)
        if initial_fit is None:
            return SphereDescriptor(
                center=np.zeros(3, dtype=np.float64),
                diameter_estimate_m=0.0,
                fit_error_m=1.0,
                normalized_fit_error=1.0,
            )

        center, radius = initial_fit
        distances = np.linalg.norm(points - center, axis=1)
        residuals = np.abs(distances - radius)
        inlier_percentile = float(
            self.get_parameter("sphere_fit_inlier_percentile").value
        )
        inlier_percentile = min(max(inlier_percentile, 50.0), 100.0)
        cutoff = np.percentile(residuals, inlier_percentile)
        inlier_mask = residuals <= cutoff
        if np.count_nonzero(inlier_mask) >= 10:
            refined_fit = self._fit_sphere(points[inlier_mask])
            if refined_fit is not None:
                center, radius = refined_fit
                distances = np.linalg.norm(points - center, axis=1)
                residuals = np.abs(distances - radius)
                cutoff = np.percentile(residuals, inlier_percentile)
                inlier_mask = residuals <= cutoff

        inlier_residuals = residuals[inlier_mask]
        if len(inlier_residuals) == 0:
            inlier_residuals = residuals

        fit_error_m = float(np.sqrt(np.mean(np.square(inlier_residuals))))
        normalized_fit_error = safe_div(fit_error_m, radius, default=1.0)

        # Sanity-check: the fitted center must lie close to the cloud centroid.
        # A single-view hemisphere fit is well-posed only when the centre stays
        # within the rough footprint of the object; a displaced centre means the
        # algebraic system collapsed to a degenerate solution.
        cloud_centroid = np.mean(points, axis=0)
        center_offset = float(np.linalg.norm(center - cloud_centroid))
        max_allowed_offset = max(dimensions.major_extent_m * 0.60, 0.03)
        if center_offset > max_allowed_offset:
            return SphereDescriptor(
                center=center,
                diameter_estimate_m=0.0,
                fit_error_m=fit_error_m,
                normalized_fit_error=1.0,
            )

        return SphereDescriptor(
            center=center,
            diameter_estimate_m=max(2.0 * radius, 0.0),
            fit_error_m=max(fit_error_m, 0.0),
            normalized_fit_error=max(normalized_fit_error, 0.0),
        )

    def _fit_sphere(self, points: np.ndarray) -> tuple[np.ndarray, float] | None:
        if len(points) < 4:
            return None

        design = np.column_stack((2.0 * points, np.ones(len(points))))
        target = np.sum(np.square(points), axis=1)
        try:
            solution, _, _, _ = np.linalg.lstsq(design, target, rcond=None)
        except np.linalg.LinAlgError:
            return None

        center = solution[:3]
        radius_sq = float(solution[3] + np.dot(center, center))
        if radius_sq <= EPSILON or not np.isfinite(radius_sq):
            return None

        return center, math.sqrt(radius_sq)

    def _classify_grasp(
        self,
        geometry: GeometryBundle,
        thresholds: dict[str, float],
    ) -> ClassificationResult:
        dimensions = geometry.dimensions
        cylinder = geometry.cylinder
        sphere = geometry.sphere
        surface = geometry.surface

        major_extent = dimensions.major_extent_m
        middle_extent = dimensions.middle_extent_m
        minor_extent = dimensions.minor_extent_m

        major_to_middle = safe_div(major_extent, middle_extent, 99.0)
        middle_to_minor = safe_div(middle_extent, minor_extent, 99.0)
        max_extent_ratio = max(major_to_middle, middle_to_minor)

        # ------------------------------------------------------------------ #
        # Surface curvature signal.                                            #
        # surface_variation_median is the key discriminator between flat       #
        # faces (box) and curved surfaces (cylinder, sphere). A flat patch     #
        # has all eigenvalues concentrated in one direction → low variation;   #
        # a curved surface spreads eigenvalues → higher variation.             #
        # Thresholds are calibrated to typical single-view household objects:  #
        #   cereal box face  ≈ 0.02–0.06                                       #
        #   water bottle arc ≈ 0.10–0.20                                       #
        #   orange / apple   ≈ 0.12–0.22                                       #
        # ------------------------------------------------------------------ #
        curvature_score = ramp(surface.surface_variation_median, 0.07, 0.20)
        flat_surface_score = inverse_ramp(surface.surface_variation_median, 0.06, 0.18)

        # Round partial-shell cues. These specifically protect partial balls and other
        # compact round objects from being treated as thin precision objects just because
        # the visible shell has a tiny PCA minor extent.
        height_to_width_ratio = safe_div(
            dimensions.height_like_m,
            max(dimensions.width_like_m, EPSILON),
            99.0,
        )
        height_width_similarity_score = spread_score(height_to_width_ratio, 1.0, 0.22)
        sphere_height_balance_score = inverse_ramp(height_to_width_ratio, 1.10, 1.28)
        round_xy_balance = spread_score(major_to_middle, 1.0, 0.22)
        round_shell_partiality = 1.0 if surface.is_partial_view else 0.0
        sphere_fit_score = inverse_ramp(
            sphere.fit_error_m,
            thresholds["spherical_fit_error_max_m"],
            thresholds["spherical_fit_error_max_m"] * 2.0,
        )
        normalized_sphere_fit_score = inverse_ramp(
            sphere.normalized_fit_error,
            0.06,
            0.18,
        )
        sphere_extent_balance = inverse_ramp(
            max_extent_ratio,
            thresholds["spherical_extent_ratio_max"],
            thresholds["spherical_extent_ratio_max"] + 0.30,
        )
        compact_visible_ratio_score = inverse_ramp(
            height_to_width_ratio,
            thresholds["spherical_extent_ratio_max"],
            thresholds["spherical_extent_ratio_max"] + 0.45,
        )
        tall_body_score = ramp(height_to_width_ratio, 1.30, 2.30)
        flat_round_penalty = clamp01(
            ramp(surface.normal_axis_alignment, 0.93, 0.98)
            * inverse_ramp(geometry.flatness_ratio, 0.28, 0.50)
        )

        # Prefer sphere diameters that match compact cross-sectional extents, and
        # penalize fits whose diameter mostly tracks the major axis of an elongated object.
        sphere_visible_size_consistency = inverse_ramp(
            abs(sphere.diameter_estimate_m - max(major_extent, middle_extent)),
            0.018,
            0.080,
        )
        sphere_cross_section_consistency = inverse_ramp(
            abs(sphere.diameter_estimate_m - max(middle_extent, cylinder.diameter_estimate_m)),
            0.020,
            0.080,
        )
        sphere_major_axis_penalty = ramp(
            safe_div(sphere.diameter_estimate_m, max(major_extent, EPSILON), 0.0),
            0.85,
            1.05,
        )
        elongated_non_sphere_penalty = ramp(geometry.elongation_ratio, 1.80, 2.60)

        compact_round_shell_score = clamp01(
            0.24 * sphere_fit_score
            + 0.18 * normalized_sphere_fit_score
            + 0.16 * round_xy_balance
            + 0.14 * compact_visible_ratio_score
            + 0.12 * sphere_cross_section_consistency
            + 0.10 * sphere_visible_size_consistency
            + 0.10 * round_shell_partiality
            - 0.20 * elongated_non_sphere_penalty
            - 0.16 * sphere_major_axis_penalty
        )

        flat_box_score = inverse_ramp(
            geometry.flatness_ratio,
            thresholds["box_flatness_ratio_max"],
            thresholds["box_flatness_ratio_max"] * 1.75,
        )
        planarity_score = ramp(geometry.planarity, 0.20, 0.60)
        normal_axis_score = ramp(surface.normal_axis_alignment, 0.80, 0.96)
        rectangular_face_score = spread_score(major_to_middle, 1.0, 0.55)
        thin_box_score = ramp(middle_to_minor, 2.5, 6.0)
        large_face_score = clamp01(
            0.55 * ramp(major_extent, thresholds["box_power_grasp_min_major_m"], thresholds["box_power_grasp_min_major_m"] + 0.10)
            + 0.45 * ramp(middle_extent, thresholds["box_power_grasp_min_middle_m"], thresholds["box_power_grasp_min_middle_m"] + 0.06)
        )
        box_face_consistency_score = clamp01(
            0.60 * rectangular_face_score
            + 0.40 * inverse_ramp(major_to_middle, 1.8, 2.8)
        )

        # flat_surface_score replaces and supplements planarity_score in the box
        # evidence: a truly flat face has low surface variation, which is a more
        # direct measure than PCA planarity alone.
        box_like_raw = clamp01(
            0.18 * flat_box_score
            + 0.17 * planarity_score
            + 0.14 * flat_surface_score          # NEW: direct curvature evidence
            + 0.14 * normal_axis_score
            + 0.13 * rectangular_face_score
            + 0.10 * thin_box_score
            + 0.08 * large_face_score
            + 0.06 * box_face_consistency_score
        )
        box_round_shell_penalty = clamp01(
            0.55 * compact_round_shell_score
            + 0.20 * inverse_ramp(surface.normal_axis_alignment, 0.82, 0.96)
            + 0.15 * inverse_ramp(geometry.planarity, 0.18, 0.45)
            + 0.10 * curvature_score             # NEW: curved surface suppresses box
        )
        box_like = clamp01(box_like_raw * (1.0 - 0.80 * box_round_shell_penalty))

        cylindrical_cross_section_roundness = spread_score(major_to_middle, 1.0, 0.30)
        cylinder_cv_score = inverse_ramp(
            cylinder.radius_cv,
            thresholds["cylindrical_radius_cv_max"],
            thresholds["cylindrical_radius_cv_max"] * 1.75,
        )
        cylinder_fit_score = inverse_ramp(
            cylinder.fit_error_m,
            thresholds["cylindrical_fit_error_max_m"],
            thresholds["cylindrical_fit_error_max_m"] * 1.75,
        )
        cylinder_outlier_score = inverse_ramp(
            cylinder.radial_outlier_fraction,
            0.10,
            0.30,
        )
        cylinder_size_consistency = inverse_ramp(
            abs(cylinder.diameter_estimate_m - middle_extent),
            0.02,
            0.10,
        )
        round_fit_cylinder_cv_score = inverse_ramp(
            cylinder.radius_cv,
            thresholds["cylindrical_radius_cv_max"],
            thresholds["cylindrical_radius_cv_max"] + 0.16,
        )
        small_body_dimension_max_m = thresholds["small_body_dimension_max_m"]
        small_body_dimension_soft_m = max(small_body_dimension_max_m - 0.005, 0.0)
        wrap_body_size_score = ramp(
            max(dimensions.width_like_m, dimensions.thickness_like_m),
            small_body_dimension_max_m,
            small_body_dimension_max_m + 0.015,
        )
        compact_round_body_size_score = inverse_ramp(
            max(dimensions.height_like_m, dimensions.width_like_m),
            thresholds["tripod_object_max_span_m"],
            thresholds["tripod_object_max_span_m"] + 0.020,
        )
        large_round_body_size_score = ramp(
            max(dimensions.height_like_m, dimensions.width_like_m),
            thresholds["tripod_object_max_span_m"],
            thresholds["tripod_object_max_span_m"] + 0.030,
        )
        small_sphere_candidate = clamp01(
            compact_round_body_size_score
            * normalized_sphere_fit_score
            * round_fit_cylinder_cv_score
            * sphere_height_balance_score
            * inverse_ramp(geometry.elongation_ratio, 1.55, 2.20)
            * (1.0 - 0.55 * flat_round_penalty)
        )
        large_sphere_candidate = clamp01(
            large_round_body_size_score
            * normalized_sphere_fit_score
            * sphere_height_balance_score
            * clamp01(
                0.65 * compact_visible_ratio_score
                + 0.35 * inverse_ramp(geometry.elongation_ratio, 1.45, 2.10)
            )
            * (1.0 - 0.85 * flat_round_penalty)
        )
        # curvature_score directly captures the arc-shaped normals of a cylinder
        # face and is a strong signal that the flat-face interpretation is wrong.
        cylinder_like_raw = clamp01(
            0.25 * cylinder_cv_score
            + 0.23 * cylinder_fit_score
            + 0.16 * cylindrical_cross_section_roundness
            + 0.12 * cylinder_outlier_score
            + 0.12 * cylinder_size_consistency
            + 0.12 * curvature_score             # NEW: curved surface supports cylinder
        )
        round_penalty_from_box = clamp01(
            0.55 * box_like
            + 0.25 * ramp(geometry.planarity, PLANARITY_FOR_ROUND_SUPPRESSION, 0.75)
            + 0.20 * ramp(surface.normal_axis_alignment, NORMAL_AXIS_FOR_BOX_CONFIDENCE, 0.99)
        )
        box_override_penalty = clamp01(
            0.60 * box_like
            + 0.40 * large_face_score * thin_box_score
        )
        tall_round_body_cylinder_support = clamp01(
            ramp(height_to_width_ratio, 1.18, 1.45)
            * max(
                cylinder_cv_score,
                cylinder_fit_score,
                cylinder_like_raw,
                compact_round_shell_score,
            )
            * wrap_body_size_score
        )
        cylinder_like = clamp01(
            cylinder_like_raw * (1.0 - 0.85 * round_penalty_from_box)
            + 0.14
            * tall_body_score
            * max(cylinder_cv_score, cylinder_fit_score)
            + 0.12
            * compact_round_shell_score
            * spread_score(cylinder.diameter_estimate_m, middle_extent, 0.04)
            + 0.16 * tall_round_body_cylinder_support
            - 0.18 * box_override_penalty
            - 0.14
            * max(small_sphere_candidate, large_sphere_candidate)
            * inverse_ramp(height_to_width_ratio, 1.35, 1.95)
        )

        scattering_score = ramp(geometry.scattering, 0.18, 0.45)
        sphere_size_consistency = inverse_ramp(
            abs(sphere.diameter_estimate_m - max(major_extent, middle_extent)),
            0.02,
            0.10,
        )
        # curvature_score also supports sphere evidence: a uniformly curved
        # visible shell is consistent with both sphere and cylinder, but combined
        # with round_xy_balance and low elongation it points toward sphere.
        sphere_like_raw = clamp01(
            0.18 * sphere_fit_score
            + 0.18 * normalized_sphere_fit_score
            + 0.10 * sphere_extent_balance
            + 0.08 * scattering_score
            + 0.08 * sphere_size_consistency
            + 0.16 * compact_round_shell_score
            + 0.14 * sphere_cross_section_consistency
            + 0.08 * compact_visible_ratio_score
            + 0.10 * curvature_score             # NEW: curved surface supports sphere
            - 0.18 * elongated_non_sphere_penalty
            - 0.18 * sphere_major_axis_penalty
            - 0.16 * flat_round_penalty
        )
        sphere_like = clamp01(
            sphere_like_raw * (1.0 - 0.70 * round_penalty_from_box)
            + 0.08 * compact_round_shell_score
            + 0.10 * small_sphere_candidate
            + 0.12 * large_sphere_candidate
            + 0.08 * sphere_height_balance_score
            - 0.12 * elongated_non_sphere_penalty
            - 0.22 * tall_round_body_cylinder_support
        )

        capped_cylinder_span_m = min(
            finite_or(cylinder.diameter_estimate_m, 0.0),
            max(dimensions.width_like_m, minor_extent) * 1.10,
        )
        capped_sphere_span_m = min(
            finite_or(sphere.diameter_estimate_m, 0.0),
            max(dimensions.height_like_m, dimensions.width_like_m) * 1.05,
        )
        power_span_reference_m = max(
            dimensions.thickness_like_m,
            capped_cylinder_span_m,
            capped_sphere_span_m,
        )
        usable_body_span_m = max(
            dimensions.width_like_m,
            dimensions.thickness_like_m,
            capped_cylinder_span_m,
        )
        effective_precision_span_m = dimensions.small_span_m
        if compact_round_shell_score >= 0.58 and sphere.diameter_estimate_m > 0.0:
            # For partial spheres, the PCA minor axis is shell depth, not a real pinch span.
            effective_precision_span_m = max(
                dimensions.small_span_m,
                min(
                    max(major_extent, middle_extent) * 0.75,
                    sphere.diameter_estimate_m * 0.75,
                ),
            )

        power_size_score = ramp(
            power_span_reference_m,
            thresholds["power_grasp_min_span_m"],
            thresholds["power_grasp_min_span_m"] + 0.03,
        )
        small_span_score = inverse_ramp(
            effective_precision_span_m,
            thresholds["small_object_max_span_m"],
            thresholds["power_grasp_min_span_m"],
        )
        # Extend the high boundary slightly (to 1.3× box_power_grasp_min_major_m)
        # so elongated thin objects longer than 10 cm (pen, ruler) still carry
        # a nonzero precision_size_score contribution to the pinch score.
        precision_size_score = inverse_ramp(
            major_extent,
            thresholds["tripod_object_max_span_m"],
            thresholds["box_power_grasp_min_major_m"] * 1.30,
        )
        narrow_body_score = inverse_ramp(
            usable_body_span_m,
            thresholds["small_object_max_span_m"],
            thresholds["tripod_object_max_span_m"] + 0.010,
        )
        wide_body_score = ramp(
            usable_body_span_m,
            thresholds["small_object_max_span_m"],
            thresholds["tripod_object_max_span_m"] + 0.020,
        )
        elongated_score = ramp(geometry.elongation_ratio, 1.25, 2.50)
        round_compact_score = clamp01(
            0.45 * sphere_like
            + 0.20 * cylinder_like
            + 0.20 * (1.0 - elongated_score)
            + 0.15 * compact_round_shell_score
        )
        small_round_size_score = inverse_ramp(
            max(power_span_reference_m, sphere.diameter_estimate_m),
            thresholds["tripod_object_max_span_m"],
            thresholds["tripod_object_max_span_m"] + 0.025,
        )
        large_round_size_score = ramp(
            max(power_span_reference_m, sphere.diameter_estimate_m),
            thresholds["tripod_object_max_span_m"],
            thresholds["tripod_object_max_span_m"] + 0.03,
        )
        spherical_shape_core = clamp01(
            0.45 * sphere_like
            + 0.20 * compact_round_shell_score
            + 0.20 * sphere_height_balance_score
            + 0.15 * curvature_score
        )
        small_spherical_body = clamp01(
            inverse_ramp(
                max(dimensions.height_like_m, dimensions.width_like_m),
                small_body_dimension_soft_m,
                small_body_dimension_max_m,
            )
            * spherical_shape_core
            * inverse_ramp(box_like, 0.50, 0.70)
        )
        large_spherical_body = clamp01(
            ramp(
                min(dimensions.height_like_m, dimensions.width_like_m),
                small_body_dimension_soft_m,
                small_body_dimension_max_m,
            )
            * spherical_shape_core
            * inverse_ramp(box_like, 0.50, 0.70)
        )
        non_spherical_shape_core = clamp01(
            0.40 * inverse_ramp(sphere_like, 0.50, 0.72)
            + 0.30 * inverse_ramp(height_width_similarity_score, 0.70, 0.90)
            + 0.20 * tall_body_score
            + 0.10 * inverse_ramp(compact_round_shell_score, 0.55, 0.75)
        )
        small_non_spherical_body = clamp01(
            inverse_ramp(
                max(dimensions.width_like_m, dimensions.thickness_like_m),
                small_body_dimension_soft_m,
                small_body_dimension_max_m,
            )
            * non_spherical_shape_core
            * (1.0 - 0.85 * small_spherical_body)
        )

        box_power_object = clamp01(
            0.34 * box_like
            + 0.20 * ramp(
                major_extent,
                thresholds["box_power_grasp_min_major_m"],
                thresholds["box_power_grasp_min_major_m"] + 0.08,
            )
            + 0.18 * ramp(
                middle_extent,
                thresholds["box_power_grasp_min_middle_m"],
                thresholds["box_power_grasp_min_middle_m"] + 0.04,
            )
            + 0.10 * power_size_score
            + 0.08 * thin_box_score
            + 0.10 * large_face_score
            - 0.14 * compact_round_shell_score
        )
        small_precision_object = clamp01(
            0.32 * precision_size_score
            + 0.30 * small_span_score
            + 0.38 * narrow_body_score
            - 0.32 * compact_round_shell_score
            - 0.20 * round_compact_score
            - 0.30 * small_sphere_candidate
            - 0.20 * wide_body_score
            - 0.25 * large_round_size_score * sphere_like
        )
        compact_height_score = inverse_ramp(
            dimensions.height_like_m,
            thresholds["tripod_object_max_span_m"] + 0.020,
            thresholds["box_power_grasp_min_major_m"] + 0.005,
        )
        compact_width_score = inverse_ramp(
            dimensions.width_like_m,
            thresholds["tripod_object_max_span_m"],
            thresholds["tripod_object_max_span_m"] + 0.020,
        )
        compact_non_spherical_object = clamp01(
            compact_height_score
            * compact_width_score
            * clamp01(
                0.55 * flat_round_penalty
                + 0.25 * box_like
                + 0.20 * inverse_ramp(sphere_like, 0.35, 0.60)
            )
            * (1.0 - 0.70 * small_sphere_candidate)
        )
        handle_like = clamp01(
            0.45 * ramp(cylinder.radial_outlier_fraction, 0.05, 0.25)
            + 0.35 * inverse_ramp(minor_extent, 0.020, 0.040)
            + 0.20 * elongated_score
            - 0.20 * compact_round_shell_score
        )

        family_evidence = {
            "box_like": box_like,
            "cylinder_like": cylinder_like,
            "sphere_like": sphere_like,
            "box_power_object": box_power_object,
            "small_precision_object": small_precision_object,
            "handle_like": handle_like,
            "round_compact_object": round_compact_score,
            "round_partial_shell": compact_round_shell_score,
            "small_round_object": small_sphere_candidate,
            "large_round_object": large_sphere_candidate,
            "small_spherical_body": small_spherical_body,
            "large_spherical_body": large_spherical_body,
            "small_non_spherical_body": small_non_spherical_body,
            "sphere_height_balance": sphere_height_balance_score,
            "tall_round_body_cylinder_support": tall_round_body_cylinder_support,
            "compact_non_spherical_object": compact_non_spherical_object,
            "large_box_face": large_face_score,
        }

        score_terms: dict[str, dict[str, float]] = {}
        score_terms["cylindrical"] = {
            "usable_power_span": 0.24 * power_size_score,
            "cylinder_body_evidence": 0.24 * cylinder_like,
            "large_box_power_evidence": 0.34 * box_power_object,
            "elongation_or_tallness": 0.10
            * max(
                elongated_score,
                ramp(
                    major_extent,
                    thresholds["box_power_grasp_min_major_m"],
                    thresholds["box_power_grasp_min_major_m"] + 0.07,
                ),
            ),
            "tall_curved_body": 0.12
            * tall_body_score
            * max(cylinder_cv_score, cylinder_fit_score),
            "tall_round_body_support": 0.18 * tall_round_body_cylinder_support,
            "wide_body_wrap": 0.10
            * wide_body_score
            * max(cylinder_like, box_power_object, cylinder_fit_score),
            "round_shell_support": 0.10
            * compact_round_shell_score
            * inverse_ramp(major_extent, 0.12, 0.18),
            "small_non_spherical_body_penalty": -0.20 * small_non_spherical_body,
            "compact_precision_penalty": -0.22 * compact_non_spherical_object,
            "precision_penalty": -0.16 * small_precision_object,
            "small_round_penalty": -0.12
            * (round_compact_score * small_round_size_score),
            "sphere_shape_penalty": -0.12
            * max(small_sphere_candidate, large_sphere_candidate)
            * inverse_ramp(height_to_width_ratio, 1.35, 1.95),
        }
        score_terms["spherical"] = {
            "sphere_fit_and_extent": 0.40 * sphere_like,
            "compact_roundness": 0.24 * round_compact_score,
            "partial_round_shell_support": 0.14 * compact_round_shell_score,
            "large_round_size": 0.18 * large_round_size_score,
            "large_round_shape": 0.20 * large_sphere_candidate,
            "explicit_large_sphere_rule": 0.18 * large_spherical_body,
            "height_width_balance": 0.16 * sphere_height_balance_score,
            "not_precision_scale": 0.08 * (1.0 - small_precision_object),
            "elongation_penalty": -0.15 * elongated_score,
            "tall_round_body_penalty": -0.32 * tall_round_body_cylinder_support,
            "box_penalty": -0.18 * box_like,
            "flat_round_penalty": -0.18 * flat_round_penalty,
            "compact_precision_penalty": -0.30 * compact_non_spherical_object,
            "small_sphere_rule_penalty": -0.12 * small_spherical_body,
            "small_round_penalty": -0.10 * small_sphere_candidate,
        }
        score_terms["tripod"] = {
            "small_round_size": 0.24 * small_round_size_score,
            "compact_roundness": 0.18 * round_compact_score,
            "sphere_evidence": 0.12 * sphere_like,
            "small_spherical_object": 0.30 * small_sphere_candidate,
            "explicit_small_sphere_rule": 0.30 * small_spherical_body,
            "precision_bias": 0.02 * small_precision_object,
            "elongation_penalty": -0.10 * elongated_score,
            "power_penalty": -0.18 * power_size_score,
            "box_penalty": -0.10 * box_like,
        }
        score_terms["pinch"] = {
            "small_opposing_span": 0.24 * small_span_score,
            "overall_smallness": 0.18 * precision_size_score,
            "narrow_body_support": 0.18 * narrow_body_score,
            "compact_non_spherical_support": 0.90 * compact_non_spherical_object,
            "small_non_spherical_rule": 0.42 * small_non_spherical_body,
            "precision_shape": 0.20
            * max(
                elongated_score,
                ramp(geometry.mid_to_minor_ratio, 1.50, 3.00),
            )
            * narrow_body_score,
            "not_power_object": 0.10 * (1.0 - box_power_object),
            "small_sphere_rule_penalty": -0.42 * small_spherical_body,
            "sphere_penalty": -0.18 * (sphere_like * large_round_size_score),
            "round_shell_penalty": -0.28 * compact_round_shell_score * large_round_size_score,
            "small_round_penalty": -0.24 * small_sphere_candidate,
            "wide_body_penalty": -0.16 * wide_body_score,
            "power_penalty": -0.24 * power_size_score,
        }
        score_terms["hook"] = {
            "handle_like_evidence": 0.70 * handle_like,
            "thin_handle_span": 0.35 * inverse_ramp(minor_extent, 0.018, 0.035),
            "small_cross_section_support": 0.10
            * inverse_ramp(
                power_span_reference_m,
                thresholds["power_grasp_min_span_m"],
                thresholds["power_grasp_min_span_m"] + 0.03,
            ),
            "suppression_prior": -thresholds["hook_score_penalty"],
            "other_grasp_penalty": -0.20
            * max(box_power_object, cylinder_like, sphere_like, small_precision_object),
        }

        scores = {
            grip_name: clamp01(sum(terms.values()))
            for grip_name, terms in score_terms.items()
        }

        priority_lookup = {
            grip_name: -index for index, grip_name in enumerate(GRIP_PRIORITY)
        }
        selected_grip = max(
            scores,
            key=lambda grip_name: (scores[grip_name], priority_lookup[grip_name]),
        )

        sorted_scores = sorted(scores.values(), reverse=True)
        best_score = sorted_scores[0]
        second_score = sorted_scores[1] if len(sorted_scores) > 1 else 0.0
        confidence = clamp01(0.70 * best_score + 0.30 * max(best_score - second_score, 0.0))

        grasp_span_basis_m, recommended_opening_m, basis_reason = self._select_span_basis(
            selected_grip=selected_grip,
            geometry=geometry,
            family_evidence=family_evidence,
            thresholds=thresholds,
        )
        object_family = self._select_object_family(
            family_evidence=family_evidence,
            selected_grip=selected_grip,
            geometry=geometry,
        )
        decision_path = self._build_decision_path(
            selected_grip=selected_grip,
            family_evidence=family_evidence,
            geometry=geometry,
            thresholds=thresholds,
            basis_reason=basis_reason,
        )

        return ClassificationResult(
            selected_grip=selected_grip,
            confidence=confidence,
            scores={name: float(score) for name, score in scores.items()},
            score_terms={
                name: {term: float(value) for term, value in terms.items()}
                for name, terms in score_terms.items()
            },
            family_evidence={
                name: float(score) for name, score in family_evidence.items()
            },
            object_family=object_family,
            decision_path=tuple(decision_path),
            grasp_span_basis_m=float(grasp_span_basis_m),
            recommended_opening_m=float(recommended_opening_m),
            basis_reason=basis_reason,
            power_span_reference_m=float(power_span_reference_m),
        )

    def _select_object_family(
        self,
        family_evidence: dict[str, float],
        selected_grip: str,
        geometry: GeometryBundle,
    ) -> str:
        dimensions = geometry.dimensions
        if family_evidence.get("compact_non_spherical_object", 0.0) >= 0.55:
            return "compact_precision_object"
        if family_evidence.get("small_round_object", 0.0) >= max(
            0.55,
            family_evidence["small_precision_object"] - 0.05,
        ):
            return "sphere_like_small"
        if family_evidence.get("large_round_object", 0.0) >= 0.45 and family_evidence[
            "sphere_like"
        ] >= max(0.35, family_evidence["cylinder_like"] - 0.10):
            return "sphere_like_large"
        large_box_candidate = (
            family_evidence["box_power_object"] >= 0.60
            and family_evidence.get("large_box_face", 0.0) >= 0.70
            and family_evidence["box_like"] >= 0.45
            and dimensions.middle_extent_m >= 0.07
            and geometry.mid_to_minor_ratio >= 2.2
        )
        if family_evidence["round_partial_shell"] >= max(
            0.62,
            family_evidence["box_like"] + 0.05,
        ):
            if family_evidence["sphere_like"] >= max(0.42, family_evidence["cylinder_like"] - 0.05):
                return "sphere_like_large" if selected_grip in ("spherical", "cylindrical") else "sphere_like_small"
        if large_box_candidate:
            return "box_like_large"
        if family_evidence["box_like"] >= max(
            BOX_DOMINANCE_FOR_ROUND_SUPPRESSION,
            family_evidence["cylinder_like"] + 0.15,
            family_evidence["sphere_like"] + 0.15,
        ):
            return "box_like_large" if family_evidence["box_power_object"] >= 0.55 else "box_like"
        if family_evidence["handle_like"] >= max(
            0.60,
            family_evidence["cylinder_like"] + 0.10,
        ):
            return "handle_like"
        if family_evidence["cylinder_like"] >= max(
            0.50,
            family_evidence["sphere_like"] + 0.10,
            family_evidence["small_precision_object"] + 0.05,
        ):
            return "cylinder_like"
        if family_evidence["sphere_like"] >= max(
            0.50,
            family_evidence["cylinder_like"] + 0.05,
        ):
            return "sphere_like_large" if selected_grip in ("spherical", "cylindrical") else "sphere_like_small"
        if family_evidence["small_precision_object"] >= 0.55:
            return (
                "slender_precision"
                if selected_grip == "pinch"
                else "small_precision_object"
            )
        return f"{selected_grip}_dominant"

    def _build_decision_path(
        self,
        selected_grip: str,
        family_evidence: dict[str, float],
        geometry: GeometryBundle,
        thresholds: dict[str, float],
        basis_reason: str,
    ) -> list[str]:
        decision_path: list[str] = []
        dimensions = geometry.dimensions

        if family_evidence.get("compact_non_spherical_object", 0.0) >= 0.55:
            decision_path.append("compact non-spherical body is small enough for pinch")
        elif family_evidence.get("small_round_object", 0.0) >= 0.55:
            decision_path.append("compact sphere fit dominates the small-object evidence")
        elif family_evidence.get("large_round_object", 0.0) >= 0.45:
            decision_path.append("round body is too large for a precision grasp")
        elif family_evidence.get("tall_round_body_cylinder_support", 0.0) >= 0.45:
            decision_path.append("curved body is too tall relative to its width to be spherical")
        elif family_evidence["box_power_object"] >= 0.55:
            decision_path.append("large object with thin grasp dimension")
        elif family_evidence["small_precision_object"] >= 0.55:
            decision_path.append("small object with precision-dominated span")
        elif (
            geometry.elongation_ratio >= 1.8
            and family_evidence["cylinder_like"] >= max(0.45, family_evidence["sphere_like"])
        ):
            decision_path.append("elongated single-view cylindrical body dominates visible cloud")
        elif family_evidence["round_partial_shell"] >= 0.58:
            decision_path.append("partial round shell dominates visible cloud")
        elif family_evidence["sphere_like"] >= 0.55:
            decision_path.append("round compact geometry dominates visible cloud")
        elif family_evidence["cylinder_like"] >= 0.50:
            decision_path.append("partial cylindrical body evidence is strong")

        if (
            geometry.elongation_ratio >= 1.8
            and family_evidence["cylinder_like"] >= max(0.45, family_evidence["sphere_like"])
        ):
            decision_path.append("trimmed radial profile supports a cylindrical body")
        elif family_evidence.get("compact_non_spherical_object", 0.0) >= 0.55:
            decision_path.append("compact size overrides wrap-grasp preference")
        elif family_evidence.get("small_round_object", 0.0) >= 0.55:
            decision_path.append("round fit is strong enough to override the tiny shell depth")
        elif family_evidence.get("large_round_object", 0.0) >= 0.45:
            decision_path.append("round geometry dominates over cylindrical wrap cues")
        elif family_evidence.get("tall_round_body_cylinder_support", 0.0) >= 0.45:
            decision_path.append("height is noticeably larger than width, so the round shell is treated as cylindrical")
        elif family_evidence["round_partial_shell"] >= max(family_evidence["box_like"], 0.58):
            decision_path.append("similar visible extents and sphere fit override shell thickness")
        elif family_evidence["box_like"] >= max(family_evidence["sphere_like"], 0.45):
            decision_path.append("dominant planar / box-like evidence")
        elif family_evidence["sphere_like"] >= max(family_evidence["box_like"], 0.45):
            decision_path.append("low sphere-fit error with similar extents")
        elif family_evidence["cylinder_like"] >= 0.45:
            decision_path.append("trimmed radial profile supports a cylindrical body")

        if selected_grip in ("cylindrical", "spherical"):
            decision_path.append("power grasp preferred over precision grasp")
        elif selected_grip == "tripod":
            decision_path.append("small round object favors tripod over power grasp")
        elif selected_grip == "pinch":
            decision_path.append("small opposing-finger span favors pinch")
        else:
            decision_path.append("handle-like evidence remained after strong hook suppression")

        if "thickness-like span" in basis_reason:
            decision_path.append("box thickness chosen as the wrap span")
        elif "body diameter" in basis_reason:
            decision_path.append("body diameter chosen as the wrap span")
        elif "sphere diameter" in basis_reason:
            decision_path.append("sphere diameter used for pre-contact aperture")
        elif selected_grip == "tripod":
            decision_path.append("compact span kept below spherical power threshold")
        elif selected_grip == "pinch":
            decision_path.append("precision span remains below power-grasp threshold")

        if thresholds["hook_score_penalty"] > 0.0:
            decision_path.append("hook suppressed by prior penalty")
        if geometry.surface.is_partial_view:
            decision_path.append("single-view partial cloud assumption applied")
        if dimensions.minor_extent_m < thresholds["power_grasp_min_span_m"]:
            decision_path.append("minor extent is below the nominal power-grasp span")

        return decision_path

    def _select_span_basis(
        self,
        selected_grip: str,
        geometry: GeometryBundle,
        family_evidence: dict[str, float],
        thresholds: dict[str, float],
    ) -> tuple[float, float, str]:
        dimensions = geometry.dimensions
        cylinder = geometry.cylinder
        sphere = geometry.sphere

        basis_reason = "fallback span"
        if selected_grip == "cylindrical":
            strong_box_wrap = (
                family_evidence["box_power_object"] >= 0.60
                and (
                    family_evidence["box_like"] >= 0.45
                    or family_evidence.get("large_box_face", 0.0) >= 0.70
                )
                and geometry.mid_to_minor_ratio >= 2.2
            )
            if strong_box_wrap:
                grasp_span_basis_m = dimensions.thickness_like_m
                basis_reason = "box-like power grasp uses thickness-like span"
            elif family_evidence.get("tall_round_body_cylinder_support", 0.0) >= 0.45:
                grasp_span_basis_m = max(
                    cylinder.diameter_estimate_m,
                    dimensions.middle_extent_m,
                )
                basis_reason = "tall curved body uses visible cylindrical diameter span"
            elif (
                family_evidence.get("round_partial_shell", 0.0) >= 0.58
                and sphere.diameter_estimate_m > 0.0
            ):
                grasp_span_basis_m = sphere.diameter_estimate_m
                basis_reason = "partial round shell uses sphere diameter as wrap span"
            elif cylinder.diameter_estimate_m > 0.0:
                # For a single-view half-cylinder the algebraic circle fit tends to
                # underestimate the true diameter because it only sees one arc.
                # dimensions.middle_extent_m spans the full visible width (≈ true
                # diameter for a front-facing cylinder), so we take the larger of
                # the two estimates to guarantee a safe pre-contact aperture.
                grasp_span_basis_m = max(
                    cylinder.diameter_estimate_m,
                    dimensions.middle_extent_m,
                )
                basis_reason = "cylindrical body diameter estimated from trimmed fit (floor-clamped to visible width)"
            else:
                grasp_span_basis_m = max(
                    dimensions.thickness_like_m,
                    0.5 * dimensions.width_like_m,
                )
                basis_reason = "fallback to stable power-grasp span"
        elif selected_grip == "spherical":
            if sphere.diameter_estimate_m > 0.0:
                grasp_span_basis_m = sphere.diameter_estimate_m
                basis_reason = "sphere diameter estimated from least-squares fit"
            else:
                grasp_span_basis_m = float(np.median(dimensions.pca_extents))
                basis_reason = "fallback to median compact extent"
        elif selected_grip == "tripod":
            if family_evidence["sphere_like"] >= 0.45 and sphere.diameter_estimate_m > 0.0:
                grasp_span_basis_m = sphere.diameter_estimate_m
                basis_reason = "small round object uses fitted diameter for tripod span"
            else:
                grasp_span_basis_m = max(
                    dimensions.small_span_m,
                    min(dimensions.width_like_m, thresholds["tripod_object_max_span_m"]),
                )
                basis_reason = "tripod uses a compact stable opposing-finger span"
        elif selected_grip == "pinch":
            if (
                family_evidence.get("round_partial_shell", 0.0) >= 0.58
                and sphere.diameter_estimate_m > 0.0
            ):
                grasp_span_basis_m = max(
                    dimensions.small_span_m,
                    min(
                        sphere.diameter_estimate_m * 0.70,
                        max(dimensions.major_extent_m, dimensions.middle_extent_m) * 0.75,
                    ),
                )
                basis_reason = "partial round shell prevents shell-depth pinch span"
            else:
                grasp_span_basis_m = dimensions.small_span_m
                basis_reason = "pinch uses the smallest stable opposing-finger span"
        else:
            grasp_span_basis_m = min(dimensions.small_span_m, dimensions.width_like_m)
            basis_reason = "hook fallback uses the thinnest visible cross-section"

        grasp_span_basis_m = max(float(grasp_span_basis_m), 0.001)
        recommended_opening_m = grasp_span_basis_m + thresholds["opening_margin_m"]
        max_hand_opening_m = thresholds["max_hand_opening_m"]
        if max_hand_opening_m > 0.0:
            recommended_opening_m = min(recommended_opening_m, max_hand_opening_m)

        return grasp_span_basis_m, recommended_opening_m, basis_reason

    def _build_output_json(
        self,
        cloud_path: Path,
        input_point_count: int,
        cleanup: CleanupSummary,
        geometry: GeometryBundle,
        classification: ClassificationResult,
        threshold_params: dict[str, float],
        output_suffix: str,
        ground_removed_suffix: str,
        recursive_search: bool,
    ) -> dict[str, object]:
        dimensions = geometry.dimensions
        cylinder = geometry.cylinder
        sphere = geometry.sphere
        surface = geometry.surface

        return {
            "schema_version": 1,
            "source_ply": str(cloud_path.resolve()),
            "generated_at_utc": datetime.now(timezone.utc)
            .replace(microsecond=0)
            .isoformat()
            .replace("+00:00", "Z"),
            "selected_grip": classification.selected_grip,
            "confidence": round(classification.confidence, 6),
            "recommended_opening_m": round(
                classification.recommended_opening_m, 6
            ),
            "grasp_span_basis_m": round(classification.grasp_span_basis_m, 6),
            "opening_margin_m": round(threshold_params["opening_margin_m"], 6),
            "scores": {
                grip_name: round(score, 6)
                for grip_name, score in classification.scores.items()
            },
            "family_evidence": {
                family_name: round(score, 6)
                for family_name, score in classification.family_evidence.items()
            },
            "decision": {
                "object_family": classification.object_family,
                "decision_path": list(classification.decision_path),
                "basis_reason": classification.basis_reason,
                "power_span_reference_m": round(
                    classification.power_span_reference_m, 6
                ),
                "score_terms": {
                    grip_name: {
                        term_name: round(term_value, 6)
                        for term_name, term_value in term_values.items()
                    }
                    for grip_name, term_values in classification.score_terms.items()
                },
            },
            "dimensions_m": {
                "pca_major": round(dimensions.major_extent_m, 6),
                "pca_middle": round(dimensions.middle_extent_m, 6),
                "pca_minor": round(dimensions.minor_extent_m, 6),
                "obb_major": round(float(dimensions.obb_extents[0]), 6),
                "obb_middle": round(float(dimensions.obb_extents[1]), 6),
                "obb_minor": round(float(dimensions.obb_extents[2]), 6),
                "height_like": round(dimensions.height_like_m, 6),
                "width_like": round(dimensions.width_like_m, 6),
                "thickness_like": round(dimensions.thickness_like_m, 6),
                "small_span": round(dimensions.small_span_m, 6),
                "cylinder_diameter_estimate": round(
                    cylinder.diameter_estimate_m, 6
                ),
                "sphere_diameter_estimate": round(sphere.diameter_estimate_m, 6),
            },
            "shape_descriptors": {
                "elongation_ratio": round(geometry.elongation_ratio, 6),
                "flatness_ratio": round(geometry.flatness_ratio, 6),
                "mid_to_minor_ratio": round(geometry.mid_to_minor_ratio, 6),
                "linearity": round(geometry.linearity, 6),
                "planarity": round(geometry.planarity, 6),
                "scattering": round(geometry.scattering, 6),
                "surface_variation_median": round(
                    surface.surface_variation_median, 6
                ),
                "curvature_score": round(
                    ramp(surface.surface_variation_median, 0.07, 0.20), 6
                ),
                "flat_surface_score": round(
                    inverse_ramp(surface.surface_variation_median, 0.06, 0.18), 6
                ),
                "cylinder_radius_cv": round(cylinder.radius_cv, 6),
                "cylinder_fit_error_m": round(cylinder.fit_error_m, 6),
                "cylinder_radial_outlier_fraction": round(
                    cylinder.radial_outlier_fraction, 6
                ),
                "sphere_fit_error_m": round(sphere.fit_error_m, 6),
                "sphere_fit_error_normalized": round(
                    sphere.normalized_fit_error, 6
                ),
                "normal_dispersion": round(surface.normal_dispersion, 6),
                "normal_axis_alignment": round(
                    surface.normal_axis_alignment, 6
                ),
                "view_bias": round(surface.view_bias, 6),
                "is_partial_view": surface.is_partial_view,
            },
            "point_cloud_stats": {
                "point_count": input_point_count,
                "point_count_after_cleanup": geometry.point_count,
                "used_normals_from_file": geometry.normals.used_normals_from_file,
                "recomputed_normals": geometry.normals.recomputed_normals,
                "removed_outliers_before_classification": cleanup.removed_outliers,
                "statistical_removed": cleanup.statistical_removed,
                "radius_removed": cleanup.radius_removed,
            },
            "assumptions": {
                "single_view_partial_cloud": True,
                "hook_deprioritized": True,
                "opening_is_initial_pre_contact_aperture": True,
            },
            "parameters": {
                "input_dir": str(cloud_path.parent.resolve()),
                "recursive_search": recursive_search,
                "ground_removed_suffix": ground_removed_suffix,
                "output_suffix": output_suffix,
                **{name: round(value, 6) for name, value in threshold_params.items()},
                "extent_percentile_low": round(
                    float(self.get_parameter("extent_percentile_low").value), 6
                ),
                "extent_percentile_high": round(
                    float(self.get_parameter("extent_percentile_high").value), 6
                ),
                "surface_variation_k_neighbors": int(
                    self.get_parameter("surface_variation_k_neighbors").value
                ),
                "normal_estimation_radius_m": round(
                    float(self.get_parameter("normal_estimation_radius_m").value), 6
                ),
                "normal_estimation_max_nn": int(
                    self.get_parameter("normal_estimation_max_nn").value
                ),
                "cylinder_axis_trim_percent": round(
                    float(self.get_parameter("cylinder_axis_trim_percent").value), 6
                ),
                "cylinder_radial_outlier_percent": round(
                    float(self.get_parameter("cylinder_radial_outlier_percent").value),
                    6,
                ),
                "sphere_fit_inlier_percentile": round(
                    float(self.get_parameter("sphere_fit_inlier_percentile").value), 6
                ),
                "statistical_nb_neighbors": int(
                    self.get_parameter("statistical_nb_neighbors").value
                ),
                "statistical_std_ratio": round(
                    float(self.get_parameter("statistical_std_ratio").value), 6
                ),
                "radius_outlier_nb_points": int(
                    self.get_parameter("radius_outlier_nb_points").value
                ),
                "radius_outlier_radius_m": round(
                    float(self.get_parameter("radius_outlier_radius_m").value), 6
                ),
                "use_radius_outlier_filter": bool(
                    self.get_parameter("use_radius_outlier_filter").value
                ),
            },
            "cleanup_warnings": list(cleanup.warnings),
        }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GraspRuleClassifierNode()
    exit_code = 1
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)
