"""Small-tooth-difference involute gear profile generator (per tooth flank).

This module ports the MATLAB prototype ``small_tooth_diff.m`` into pure Python
and exposes a flank-based API. Instead of returning closed polylines for the
entire gear, each tooth flank (left/right) is exported separately so that the
EXUDYN model can instantiate individual contact objects per tooth pair.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import math
import numpy as np
from numpy.typing import NDArray

__all__ = [
    "FewTeethGearPair",
    "FewTeethGeometry",
    "generate_few_teeth_flanks",
    "CycloidProfileParams",
    "generate_cycloid_profile",
]


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------


@dataclass
class FewTeethGearPair:
    """Parameter set describing a few-tooth-difference gear pair (SI units)."""

    z_outer: int
    """Teeth count of the outer (driver) gear."""

    z_inner: int
    """Teeth count of the inner gear (z_inner > z_outer)."""

    module: float
    """Normal module ``m`` (meters)."""

    x_outer: float = 0.0
    """Addendum modification coefficient of the outer gear (dimensionless)."""

    x_inner: float = 0.5
    """Addendum modification coefficient of the inner gear (dimensionless)."""

    alpha_deg: float = 20.0
    """Standard pressure angle in degrees (converted to radians via ``alpha`` property)."""

    ha_star: float = 0.75
    """Normalized addendum height coefficient (per module)."""

    c_star: float = 0.5
    """Normalized clearance coefficient (per module)."""

    flank_points_outer: int = 80
    """Sampling points per outer tooth flank used to discretize the involute."""

    flank_points_inner: int = 120
    """Sampling points per inner tooth flank used to discretize the involute."""

    tip_probe_length: float = 0.0
    """Optional visualization length for the outer tooth tip probe segment (meters)."""
    """Sampling points per inner tooth flank used to discretize the involute."""

    @property
    def alpha(self) -> float:
        return math.radians(self.alpha_deg)


@dataclass
class FewTeethGeometry:
    """Derived geometry useful for downstream processing."""

    alpha0: float
    center_distance: float
    y_shift: float
    delta_y: float
    r_outer: float
    r_inner: float
    rb_outer: float
    rb_inner: float
    ra_outer: float
    ra_inner: float
    rf_outer: float
    rf_inner: float


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _solve_alpha0(target: float, initial: float = 0.5, tol: float = 1e-12, max_iter: int = 50) -> float:
    """Solve ``tan(alpha0) - alpha0 = target`` by Newton iteration."""

    alpha0 = initial
    for _ in range(max_iter):
        f = math.tan(alpha0) - alpha0 - target
        df = 1.0 / (math.cos(alpha0) ** 2) - 1.0
        step = f / df
        alpha0 -= step
        if abs(step) < tol:
            return alpha0
    raise RuntimeError("Failed to converge while solving alpha0")


def _remove_duplicate_points(x: NDArray[np.float64], y: NDArray[np.float64], tol: float = 1e-12) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
    if x.size == 0:
        return x, y
    dx = np.diff(x)
    dy = np.diff(y)
    mask = np.concatenate([[True], np.hypot(dx, dy) > tol])
    return x[mask], y[mask]


def _rotate_points(x: NDArray[np.float64], y: NDArray[np.float64], angle: float) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    xr = cos_a * x - sin_a * y
    yr = sin_a * x + cos_a * y
    return xr, yr


def _append_tip_probe_segment(
    x: NDArray[np.float64],
    y: NDArray[np.float64],
    length: float,
    is_inner: bool = False,
    flip_direction: bool = False,
) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    Extend a flank polyline with an extra point along the surface normal at the tip.
    
    Uses the same probe direction logic as C++:
    - Computes curvature normal (tangent rotated 90 degrees)
    - Determines direction based on gear type (inner/outer)
    - Does not depend on runtime geometric position judgments
    
    Args:
        x, y: Flank point coordinates
        length: Probe extension length
        is_inner: True for inner gear, False for outer gear
        flip_direction: If True, flip the probe direction (for mirrored flanks)
    
    Returns:
        Extended x, y arrays with probe point appended
    """
    if length <= 0.0 or x.size < 2:
        return x, y

    tip = np.array([x[-1], y[-1]], dtype=float)
    prev = np.array([x[-2], y[-2]], dtype=float)
    
    # 1. Compute tangent vector
    tangent = tip - prev
    tangent_norm = np.linalg.norm(tangent)
    if tangent_norm <= 1e-12:
        return x, y

    # 2. Compute curvature normal (rotate tangent 90 degrees counterclockwise)
    # tangent = [dx, dy] → curvature_normal = [-dy, dx]
    curvature_normal = np.array([-tangent[1], tangent[0]], dtype=float)
    
    # 3. Determine solid normal based on gear type
    # Matches C++ ComputeSolidNormalFromCurvature logic
    if is_inner:
        # Inner gear: curvature normal points to cavity, flip to point to solid
        solid_normal = -curvature_normal
    else:
        # Outer gear: curvature normal already points to solid exterior
        solid_normal = curvature_normal
    
    # 4. Apply flip for mirrored flanks (left flank)
    if flip_direction:
        solid_normal = -solid_normal
    
    probe_direction = solid_normal
    
    # 5. Normalize and extend
    normal_norm = np.linalg.norm(probe_direction)
    if normal_norm <= 1e-12:
        return x, y

    probe_point = tip + probe_direction / normal_norm * length
    x_ext = np.concatenate((x, np.array([probe_point[0]])))
    y_ext = np.concatenate((y, np.array([probe_point[1]])))
    return x_ext, y_ext


# ---------------------------------------------------------------------------
# Base tooth flank generation (local coordinates)
# ---------------------------------------------------------------------------


def _outer_tooth_flanks(params: FewTeethGearPair, n_points: Optional[int]) -> Dict[str, Tuple[NDArray[np.float64], NDArray[np.float64]]]:
    alpha = params.alpha
    m = params.module
    z1 = params.z_outer
    x1 = params.x_outer

    r1 = 0.5 * m * z1
    rb1 = r1 * math.cos(alpha)
    rf1 = r1 - m * (params.ha_star + params.c_star - x1)
    ra1 = r1 + m * (params.ha_star + x1)
    s1 = math.pi * m / 2.0 + 2.0 * x1 * m * math.tan(alpha)

    sample = n_points or params.flank_points_outer
    r_min = max(rb1, rf1)
    r_max = ra1
    rC = np.linspace(r_min, r_max, max(sample, 2), dtype=float)

    ratio = np.clip(rb1 / rC, -1.0, 1.0)
    alpha_c = np.arccos(ratio)
    theta = math.tan(alpha) - alpha
    theta_c = np.tan(alpha_c) - alpha_c
    phi = s1 / (2.0 * r1)
    phi_c = theta + phi - theta_c

    x_right = rC * np.sin(phi_c)
    y_right = rC * np.cos(phi_c)
    x_right, y_right = _remove_duplicate_points(x_right, y_right)
    x_right_base = x_right.copy()
    y_right_base = y_right.copy()
    x_right, y_right = _append_tip_probe_segment(x_right, y_right, params.tip_probe_length, is_inner=False, flip_direction=False)

    x_left = -x_right_base
    y_left = y_right_base
    x_left, y_left = _remove_duplicate_points(x_left, y_left)
    x_left, y_left = _append_tip_probe_segment(x_left, y_left, params.tip_probe_length, is_inner=False, flip_direction=True)

    return {
        "left": (x_left, y_left),
        "right": (x_right, y_right),
    }


def _inner_tooth_flanks(params: FewTeethGearPair, delta_y: float, n_points: Optional[int]) -> Dict[str, Tuple[NDArray[np.float64], NDArray[np.float64]]]:
    alpha = params.alpha
    m = params.module
    z2 = params.z_inner
    x2 = params.x_inner

    r2 = 0.5 * m * z2
    rb2 = r2 * math.cos(alpha)
    ra2 = r2 - m * (params.ha_star - x2 + delta_y)
    rf2 = r2 + m * (params.ha_star + params.c_star + x2)
    s2 = math.pi * m / 2.0 - 2.0 * x2 * m * math.tan(alpha)

    sample = n_points or params.flank_points_inner
    r_min = max(ra2, rb2)
    r_max = rf2
    rC = np.linspace(r_min, r_max, max(sample, 2), dtype=float)

    ratio = np.clip(rb2 / rC, -1.0, 1.0)
    alpha_c = np.arccos(ratio)
    theta = math.tan(alpha) - alpha
    theta_c = np.tan(alpha_c) - alpha_c
    phi = s2 / (2.0 * r2)
    phi_c = -theta + phi + theta_c

    x_right = rC * np.sin(phi_c)
    y_right = rC * np.cos(phi_c)
    x_right, y_right = _remove_duplicate_points(x_right, y_right)

    x_left = -x_right
    y_left = y_right
    x_left, y_left = _remove_duplicate_points(x_left, y_left)

    return {
        "left": (x_left, y_left),
        "right": (x_right, y_right),
    }


# ---------------------------------------------------------------------------
# Geometry aggregation
# ---------------------------------------------------------------------------


def _compute_geometry(params: FewTeethGearPair) -> FewTeethGeometry:
    alpha = params.alpha
    inv_alpha = math.tan(alpha) - alpha
    target = inv_alpha + 2.0 * (params.x_inner - params.x_outer) / (params.z_inner - params.z_outer) * math.tan(alpha)
    alpha0 = _solve_alpha0(target, initial=alpha)

    m = params.module
    a0 = 0.5 * m * (params.z_inner - params.z_outer) * math.cos(alpha) / math.cos(alpha0)
    y = (params.z_inner - params.z_outer) / 2.0 * (math.cos(alpha) / math.cos(alpha0) - 1.0)
    delta_y = params.x_inner - params.x_outer - y

    r_outer = 0.5 * m * params.z_outer
    r_inner = 0.5 * m * params.z_inner
    rb_outer = r_outer * math.cos(alpha)
    rb_inner = r_inner * math.cos(alpha)
    ra_outer = r_outer + m * (params.ha_star + params.x_outer)
    ra_inner = r_inner - m * (params.ha_star - params.x_inner + delta_y)
    rf_outer = r_outer - m * (params.ha_star + params.c_star - params.x_outer)
    rf_inner = r_inner + m * (params.ha_star + params.c_star + params.x_inner)

    return FewTeethGeometry(
        alpha0=alpha0,
        center_distance=a0,
        y_shift=y,
        delta_y=delta_y,
        r_outer=r_outer,
        r_inner=r_inner,
        rb_outer=rb_outer,
        rb_inner=rb_inner,
        ra_outer=ra_outer,
        ra_inner=ra_inner,
        rf_outer=rf_outer,
        rf_inner=rf_inner,
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def generate_few_teeth_flanks(
    params: FewTeethGearPair,
    *,
    outer_points: Optional[int] = None,
    inner_points: Optional[int] = None,
) -> Tuple[Dict[Tuple[str, int, str], NDArray[np.float64]], FewTeethGeometry]:
    """Generate per-tooth flank segments for outer/inner gears.

    Returns:
        flanks: dict where key = ("outer"|"inner", tooth_index, "left"|"right")
            and value is ``numpy.ndarray`` with shape ``(n_points, 2)``
            containing [x, y] rows in local coordinates (outer flanks optionally
            include an extra tip-probe segment when ``params.tip_probe_length`` > 0).
        geometry: :class:`FewTeethGeometry` with derived scalar values.
    """

    geometry = _compute_geometry(params)
    flanks: Dict[Tuple[str, int, str], NDArray[np.float64]] = {}

    base_outer = _outer_tooth_flanks(params, outer_points)
    for tooth_index in range(params.z_outer):
        angle = 2.0 * math.pi * tooth_index / params.z_outer
        for flank_name, (x_base, y_base) in base_outer.items():
            xr, yr = _rotate_points(x_base, y_base, angle)
            flanks[("outer", tooth_index, flank_name)] = np.column_stack((xr, yr))

    base_inner = _inner_tooth_flanks(params, geometry.delta_y, inner_points)
    theta_offset_inner = math.pi / params.z_inner
    for tooth_index in range(params.z_inner):
        angle = 2.0 * math.pi * tooth_index / params.z_inner + theta_offset_inner
        for flank_name, (x_base, y_base) in base_inner.items():
            xr, yr = _rotate_points(x_base, y_base, angle)
            flanks[("inner", tooth_index, flank_name)] = np.column_stack((xr, yr))

    return flanks, geometry


# ---------------------------------------------------------------------------
# Legacy compatibility (notify users to migrate)
# ---------------------------------------------------------------------------

CycloidProfileParams = FewTeethGearPair


def generate_cycloid_profile(*args, **kwargs):
    raise RuntimeError(
        "generate_cycloid_profile() 已废弃，请改用 generate_few_teeth_flanks() 并在主程序中"
        "按齿面创建接触对象。"
    )
