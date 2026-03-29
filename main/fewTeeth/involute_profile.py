from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import math
import numpy as np
from numpy.typing import NDArray

__all__ = [
    "ExternalGearParams",
    "ExternalGearGeometry",
    "generate_external_gear_flanks",
    "generate_external_gear_polylines",
    "as_3d_polyline",
]


@dataclass
class ExternalGearParams:
    z: int
    module: float
    alpha_deg: float = 20.0
    x: float = 0.0
    ha_star: float = 1.0
    c_star: float = 0.25
    flank_points: int = 80
    tip_probe_length: float = 0.0

    @property
    def alpha(self) -> float:
        return math.radians(self.alpha_deg)


@dataclass
class ExternalGearGeometry:
    r: float
    rb: float
    ra: float
    rf: float
    s: float
    phi_pitch: float


def _remove_duplicate_points(
    x: NDArray[np.float64],
    y: NDArray[np.float64],
    tol: float = 1e-12,
) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
    if x.size == 0:
        return x, y
    dx = np.diff(x)
    dy = np.diff(y)
    mask = np.concatenate([[True], np.hypot(dx, dy) > tol])
    return x[mask], y[mask]


def _rotate_points(
    x: NDArray[np.float64],
    y: NDArray[np.float64],
    angle: float,
) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
    c = math.cos(angle)
    s = math.sin(angle)
    xr = c * x - s * y
    yr = s * x + c * y
    return xr, yr


def _append_tip_probe_segment(
    x: NDArray[np.float64],
    y: NDArray[np.float64],
    length: float,
    flip_direction: bool,
) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
    if length <= 0.0 or x.size < 2:
        return x, y

    tip = np.array([x[-1], y[-1]], dtype=float)
    prev = np.array([x[-2], y[-2]], dtype=float)
    tangent = tip - prev
    tnorm = float(np.linalg.norm(tangent))
    if tnorm <= 1e-12:
        return x, y

    curvature_normal = np.array([-tangent[1], tangent[0]], dtype=float)
    solid_normal = curvature_normal
    if flip_direction:
        solid_normal = -solid_normal

    nnorm = float(np.linalg.norm(solid_normal))
    if nnorm <= 1e-12:
        return x, y

    probe = tip + solid_normal / nnorm * length
    return np.concatenate((x, np.array([probe[0]]))), np.concatenate((y, np.array([probe[1]])))


def _compute_geometry(params: ExternalGearParams) -> ExternalGearGeometry:
    if params.z <= 0 or params.module <= 0:
        raise ValueError("ExternalGearParams requires z>0 and module>0")

    alpha = params.alpha
    m = params.module
    z = params.z
    x = params.x

    r = 0.5 * m * z
    rb = r * math.cos(alpha)
    rf = r - m * (params.ha_star + params.c_star - x)
    ra = r + m * (params.ha_star + x)
    s = math.pi * m / 2.0 + 2.0 * x * m * math.tan(alpha)
    phi_pitch = s / (2.0 * r) if r != 0.0 else 0.0
    return ExternalGearGeometry(r=r, rb=rb, ra=ra, rf=rf, s=s, phi_pitch=phi_pitch)


def _base_tooth_flanks(params: ExternalGearParams, geometry: ExternalGearGeometry) -> Dict[str, NDArray[np.float64]]:
    alpha = params.alpha
    rb = geometry.rb
    rf = geometry.rf
    ra = geometry.ra
    phi = geometry.phi_pitch

    n = max(int(params.flank_points), 2)
    r_min = max(rb, rf)
    r_max = ra
    rC = np.linspace(r_min, r_max, n, dtype=float)

    ratio = np.clip(rb / rC, -1.0, 1.0)
    alpha_c = np.arccos(ratio)
    theta = math.tan(alpha) - alpha
    theta_c = np.tan(alpha_c) - alpha_c
    phi_c = theta + phi - theta_c

    x_right = rC * np.sin(phi_c)
    y_right = rC * np.cos(phi_c)
    x_right, y_right = _remove_duplicate_points(x_right, y_right)
    x_right_base = x_right.copy()
    y_right_base = y_right.copy()
    x_right, y_right = _append_tip_probe_segment(x_right, y_right, params.tip_probe_length, flip_direction=False)

    x_left = -x_right_base
    y_left = y_right_base
    x_left, y_left = _remove_duplicate_points(x_left, y_left)
    x_left, y_left = _append_tip_probe_segment(x_left, y_left, params.tip_probe_length, flip_direction=True)

    return {
        "left": np.column_stack((x_left, y_left)),
        "right": np.column_stack((x_right, y_right)),
    }


def generate_external_gear_flanks(
    params: ExternalGearParams,
    *,
    replicate_all_teeth: bool = True,
    tooth_index: int = 0,
) -> Tuple[Dict[Tuple[int, str], NDArray[np.float64]], ExternalGearGeometry]:
    geometry = _compute_geometry(params)
    base = _base_tooth_flanks(params, geometry)
    flanks: Dict[Tuple[int, str], NDArray[np.float64]] = {}

    if replicate_all_teeth:
        for ti in range(params.z):
            angle = 2.0 * math.pi * ti / params.z
            for flank_name, pts in base.items():
                xr, yr = _rotate_points(pts[:, 0], pts[:, 1], angle)
                flanks[(ti, flank_name)] = np.column_stack((xr, yr))
    else:
        ti = int(tooth_index) % params.z
        angle = 2.0 * math.pi * ti / params.z
        for flank_name, pts in base.items():
            xr, yr = _rotate_points(pts[:, 0], pts[:, 1], angle)
            flanks[(ti, flank_name)] = np.column_stack((xr, yr))

    return flanks, geometry


def generate_external_gear_polylines(
    params: ExternalGearParams,
    *,
    z_level: float = 0.0,
) -> Tuple[list[list[list[float]]], ExternalGearGeometry]:
    flanks, geometry = generate_external_gear_flanks(params, replicate_all_teeth=True)
    polylines: list[list[list[float]]] = []
    for tooth_index in range(params.z):
        for flank_name in ("left", "right"):
            pts = flanks[(tooth_index, flank_name)]
            polylines.append(as_3d_polyline(pts, z_level=z_level))
    return polylines, geometry


def as_3d_polyline(points2d: NDArray[np.float64], *, z_level: float = 0.0) -> list[list[float]]:
    return [[float(x), float(y), float(z_level)] for x, y in points2d]
