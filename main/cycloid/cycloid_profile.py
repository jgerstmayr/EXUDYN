"""Cycloid profile generator supporting multi-segment modifications.

This module ports the three/four/five-segment shaping approach from the
`cycloidPin.m` MATLAB tool into Python so that the EXUDYN workflow can rely on
the same family of tooth profiles.

Note: All length parameters (e, R_z, r_z, gamma values) should be provided
in consistent units. When used with EXUDYN, SI units (meters) are expected.
"""

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray


@dataclass
class CycloidProfileParams:
    """Cycloid wheel tooth profile parameters with shaping controls.
    
    All length parameters (e, R_z, r_z, gamma*) should use consistent units.
    For EXUDYN simulations, use SI units (meters).
    Angular parameters (theta*, delta_theta) are in degrees.
    """

    z_b: float  # Number of pin teeth
    e: float  # Eccentricity (m when using SI)
    R_z: float  # Pin circle radius (m when using SI)
    r_z: float  # Pin radius (m when using SI)

    profile_mode: str = "3seg"  # {'3seg', '4seg', '5seg'}
    delta_theta: float = 0.5  # angular step in degrees for segment sampling

    # Three-segment shaping parameters (theta* in degrees, gamma* in length units)
    theta1_3: float = 0.0
    theta2_3: float = 20.0
    theta3_3: float = 100.0
    theta4_3: float = 180.0
    gamma1_3: float = 0.03e-3  # Default in meters (was 0.03 mm)
    gamma2_3: float = 0.02e-3  # Default in meters (was 0.02 mm)

    # Four-segment shaping parameters (theta* in degrees, gamma* in length units)
    theta1_4: float = 0.0
    theta2_4: float = 2.0
    theta3_4: float = 5.0
    theta4_4: float = 120.0
    theta5_4: float = 170.0
    theta6_4: float = 180.0
    gamma1_4: float = 0.025e-3  # Default in meters (was 0.025 mm)
    gamma2_4: float = 0.1e-3  # Default in meters (was 0.1 mm)

    # Five-segment shaping parameters (theta* in degrees, gamma* in length units)
    theta1_5: float = 0.0
    theta2_5: float = 2.0
    theta3_5: float = 8.0
    theta4_5: float = 45.0
    theta5_5: float = 90.0
    theta6_5: float = 120.0
    theta7_5: float = 180.0
    gamma1_5: float = 0.04e-3  # Default in meters (was 0.04 mm)
    gamma2_5: float = 0.02e-3  # Default in meters (was 0.02 mm)
    gamma3_5: float = 0.04e-3  # Default in meters (was 0.04 mm)

    # Manufacturing deviation placeholders (in length units, meters when using SI)
    cross_rod_error: float = 0.0
    cross_rod_errors: Optional[Sequence[float]] = None


def _inclusive_arange(start: float, stop: float, step: float) -> NDArray[np.float64]:
    """Return a 1D array including both start and stop with the given step."""

    if step <= 0:
        raise ValueError("delta_theta must be positive")
    if stop < start:
        # np.arange handles this when step is positive, but we clamp to ensure inclusion
        count = int(np.floor((stop - start) / step)) + 1
    else:
        count = int(np.floor((stop - start) / step)) + 1
    values = start + step * np.arange(max(count, 1))
    if values.size == 0 or values[-1] < stop:
        values = np.append(values, stop)
    else:
        values[-1] = stop
    return values


def _rotate_deg(x: NDArray[np.float64], y: NDArray[np.float64], angle_deg: float) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
    angle_rad = np.deg2rad(angle_deg)
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)
    xr = x * cos_a - y * sin_a
    yr = x * sin_a + y * cos_a
    return xr, yr


def _apply_radial_offset(x: NDArray[np.float64], y: NDArray[np.float64], delta: float) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
    if abs(delta) < 1e-12:
        return x, y
    theta = np.arctan2(y, x)
    radius = np.hypot(x, y)
    adjusted_radius = radius + 0.5 * delta
    return adjusted_radius * np.cos(theta), adjusted_radius * np.sin(theta)


def _cycloid_core_formula(
    theta_deg,
    A: float,
    Za: int,
    Zb: int,
    Rz: float,
    Rc,
    backend: str = 'numpy'
):
    """
    摆线轮齿廓坐标核心公式。
    
    Parameters
    ----------
    theta_deg : 角度参数（度），numpy数组或CasADi符号变量
    A : 偏心距 e
    Za : 摆线轮齿数 (z_b - 1)
    Zb : 针销数 z_b
    Rz : 针销中心圆半径 R_z
    Rc : 修形后的滚子半径（与theta_deg同类型）
    backend : 'numpy' 或 'casadi'
    
    Returns
    -------
    x, y : 齿廓坐标
    cos_gamma, sin_gamma : 法向分量
    """
    if backend == 'casadi':
        import casadi as ca
        sin_fn, cos_fn, sqrt_fn = ca.sin, ca.cos, ca.sqrt
        fmax_fn = ca.fmax
        deg2rad = lambda d: d * np.pi / 180
    else:
        sin_fn, cos_fn, sqrt_fn = np.sin, np.cos, np.sqrt
        fmax_fn = lambda a, b: np.maximum(a, b)
        deg2rad = np.deg2rad
    
    phi_rad = deg2rad(theta_deg / Za)
    za_phi_rad = Za * phi_rad
    zb_phi_rad = Zb * phi_rad
    
    K1 = A * Zb / Rz
    denom_sq = 1 + K1**2 - 2 * K1 * cos_fn(za_phi_rad)
    denom = sqrt_fn(fmax_fn(denom_sq, 1e-12))
    
    # 基础摆线坐标
    x0 = Rz * (sin_fn(phi_rad) - (K1 / Zb) * sin_fn(zb_phi_rad))
    y0 = Rz * (cos_fn(phi_rad) - (K1 / Zb) * cos_fn(zb_phi_rad))
    
    # 法向偏移方向
    cos_gamma = (K1 * sin_fn(zb_phi_rad) - sin_fn(phi_rad)) / denom
    sin_gamma = (-K1 * cos_fn(zb_phi_rad) + cos_fn(phi_rad)) / denom
    
    # 完整坐标（包含修形等距曲线偏移）
    x = x0 + Rc * cos_gamma
    y = y0 - Rc * sin_gamma
    
    return x, y, cos_gamma, sin_gamma

def _calculate_segment_coordinates(
    A: float,
    Za: int,
    Zb: int,
    Rz: float,
    Rc_vals: NDArray[np.float64],
    theta_deg: NDArray[np.float64],
    segment_idx: int,
    cross_rod_error: float,
) -> Tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.int_]]:
    theta_deg = np.asarray(theta_deg, dtype=float)
    phi_deg = theta_deg / Za
    phi_rad = np.deg2rad(phi_deg)
    zb_phi_rad = np.deg2rad(Zb * phi_deg)
    za_phi_rad = np.deg2rad(Za * phi_deg)

    K1 = A * Zb / Rz
    denom = np.sqrt(np.clip(1 + K1**2 - 2 * K1 * np.cos(za_phi_rad), 1e-12, None))

    x0 = Rz * (np.sin(phi_rad) - (K1 / Zb) * np.sin(zb_phi_rad))
    y0 = Rz * (np.cos(phi_rad) - (K1 / Zb) * np.cos(zb_phi_rad))

    cos_gamma = (K1 * np.sin(zb_phi_rad) - np.sin(phi_rad)) / denom
    sin_gamma = (-K1 * np.cos(zb_phi_rad) + np.cos(phi_rad)) / denom

    Rc_vals = np.asarray(Rc_vals, dtype=float)
    X = x0 + Rc_vals * cos_gamma
    Y = y0 - Rc_vals * sin_gamma

    X, Y = _apply_radial_offset(X, Y, cross_rod_error)
    S = np.full(theta_deg.shape, segment_idx, dtype=int)
    return X, Y, S


def _compute_profile_3seg(params: CycloidProfileParams) -> Tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.int_]]:
    A = float(params.e)
    Zb = int(round(params.z_b))
    Za = Zb - 1
    if Za <= 0:
        raise ValueError("z_b must be greater than 1")
    Rz = float(params.R_z)
    Rc = float(params.r_z)
    dt = float(params.delta_theta)
    t1, t2, t3, t4 = map(float, (params.theta1_3, params.theta2_3, params.theta3_3, params.theta4_3))
    g1 = float(params.gamma1_3)
    g2 = float(params.gamma2_3)
    cross_err = float(params.cross_rod_error)

    theta1_vals = _inclusive_arange(t1, t2, dt)
    denom12 = max(t2 - t1, 1e-9)
    Rc1 = Rc + g1 * (t2 - np.abs(theta1_vals)) / denom12
    X1, Y1, S1 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc1, theta1_vals, 1, cross_err)

    theta2_vals = _inclusive_arange(t2, t3, dt)
    Rc2 = np.full_like(theta2_vals, Rc)
    X2, Y2, S2 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc2, theta2_vals, 2, cross_err)

    theta3_vals = _inclusive_arange(t3, t4, dt)
    denom34 = max(t4 - t3, 1e-9)
    Rc3 = Rc + g2 * (np.abs(theta3_vals) - t3) / denom34
    X3, Y3, S3 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc3, theta3_vals, 3, cross_err)

    theta1_neg = _inclusive_arange(-t2, -t1, dt)
    Rc1_neg = Rc + g1 * (t2 - np.abs(theta1_neg)) / denom12
    X1n, Y1n, S1n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc1_neg, theta1_neg, 4, cross_err)

    theta2_neg = _inclusive_arange(-t3, -t2, dt)
    Rc2_neg = np.full_like(theta2_neg, Rc)
    X2n, Y2n, S2n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc2_neg, theta2_neg, 5, cross_err)

    theta3_neg = _inclusive_arange(-t4, -t3, dt)
    Rc3_neg = Rc + g2 * (np.abs(theta3_neg) - t3) / denom34
    X3n, Y3n, S3n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc3_neg, theta3_neg, 6, cross_err)

    X_profile = np.concatenate([X3n, X2n, X1n, X1, X2, X3])
    Y_profile = np.concatenate([Y3n, Y2n, Y1n, Y1, Y2, Y3])
    S_profile = np.concatenate([S3n, S2n, S1n, S1, S2, S3])
    return _revolve_and_sort(params, X_profile, Y_profile, S_profile)


def _compute_profile_4seg(params: CycloidProfileParams) -> Tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.int_]]:
    A = float(params.e)
    Zb = int(round(params.z_b))
    Za = Zb - 1
    if Za <= 0:
        raise ValueError("z_b must be greater than 1")
    Rz = float(params.R_z)
    Rc = float(params.r_z)
    dt = float(params.delta_theta)
    t1, t2, t3, t4, t5, t6 = map(float, (
        params.theta1_4,
        params.theta2_4,
        params.theta3_4,
        params.theta4_4,
        params.theta5_4,
        params.theta6_4,
    ))
    g1 = float(params.gamma1_4)
    g2 = float(params.gamma2_4)
    cross_err = float(params.cross_rod_error)

    theta1_vals = _inclusive_arange(t1, t2, dt)
    Rc1 = np.full_like(theta1_vals, Rc + g1)
    X1, Y1, S1 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc1, theta1_vals, 1, cross_err)

    theta2_vals = _inclusive_arange(t2, t3, dt)
    denom23 = max(t3 - t2, 1e-9)
    Rc2 = Rc + g1 * (t3 - np.abs(theta2_vals)) / denom23
    X2, Y2, S2 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc2, theta2_vals, 2, cross_err)

    theta3_vals = _inclusive_arange(t3, t4, dt)
    Rc3 = np.full_like(theta3_vals, Rc)
    X3, Y3, S3 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc3, theta3_vals, 3, cross_err)

    theta4_vals = _inclusive_arange(t4, t5, dt)
    denom45 = max(t5 - t4, 1e-9)
    Rc4 = Rc + g2 * (np.abs(theta4_vals) - t4) / denom45
    X4, Y4, S4 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc4, theta4_vals, 4, cross_err)

    theta5_vals = _inclusive_arange(t5, t6, dt)
    Rc5 = np.full_like(theta5_vals, Rc + g2)
    X5, Y5, S5 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc5, theta5_vals, 5, cross_err)

    theta1_neg = _inclusive_arange(-t6, -t5, dt)
    Rc5n = np.full_like(theta1_neg, Rc + g2)
    X5n, Y5n, S5n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc5n, theta1_neg, 10, cross_err)

    theta2_neg = _inclusive_arange(-t5, -t4, dt)
    denom54 = denom45
    Rc4n = Rc + g2 * (np.abs(theta2_neg) - t4) / denom54
    X4n, Y4n, S4n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc4n, theta2_neg, 9, cross_err)

    theta3_neg = _inclusive_arange(-t4, -t3, dt)
    Rc3n = np.full_like(theta3_neg, Rc)
    X3n, Y3n, S3n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc3n, theta3_neg, 8, cross_err)

    theta4_neg = _inclusive_arange(-t3, -t2, dt)
    Rc2n = Rc + g1 * (t3 - np.abs(theta4_neg)) / denom23
    X2n, Y2n, S2n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc2n, theta4_neg, 7, cross_err)

    theta5_neg = _inclusive_arange(-t2, -t1, dt)
    Rc1n = np.full_like(theta5_neg, Rc + g1)
    X1n, Y1n, S1n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc1n, theta5_neg, 6, cross_err)

    X_profile = np.concatenate([X5n, X4n, X3n, X2n, X1n, X1, X2, X3, X4, X5])
    Y_profile = np.concatenate([Y5n, Y4n, Y3n, Y2n, Y1n, Y1, Y2, Y3, Y4, Y5])
    S_profile = np.concatenate([S5n, S4n, S3n, S2n, S1n, S1, S2, S3, S4, S5])
    return _revolve_and_sort(params, X_profile, Y_profile, S_profile)


def _compute_profile_5seg(params: CycloidProfileParams) -> Tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.int_]]:
    A = float(params.e)
    Zb = int(round(params.z_b))
    Za = Zb - 1
    if Za <= 0:
        raise ValueError("z_b must be greater than 1")
    Rz = float(params.R_z)
    Rc = float(params.r_z)
    dt = float(params.delta_theta)
    t1, t2, t3, t4, t5, t6, t7 = map(float, (
        params.theta1_5,
        params.theta2_5,
        params.theta3_5,
        params.theta4_5,
        params.theta5_5,
        params.theta6_5,
        params.theta7_5,
    ))
    g1 = float(params.gamma1_5)
    g2 = float(params.gamma2_5)
    g3 = float(params.gamma3_5)
    cross_err = float(params.cross_rod_error)

    theta1_vals = _inclusive_arange(t1, t2, dt)
    Rc1 = np.full_like(theta1_vals, Rc + g1)
    X1, Y1, S1 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc1, theta1_vals, 1, cross_err)

    theta2_vals = _inclusive_arange(t2, t3, dt)
    denom23 = max(t3 - t2, 1e-9)
    Rc2 = Rc + g1 * (t3 - np.abs(theta2_vals)) / denom23
    X2, Y2, S2 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc2, theta2_vals, 2, cross_err)

    theta3_vals = _inclusive_arange(t3, t4, dt)
    Rc3 = np.full_like(theta3_vals, Rc)
    X3, Y3, S3 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc3, theta3_vals, 3, cross_err)

    theta4_vals = _inclusive_arange(t4, t5, dt)
    denom45 = max(t5 - t4, 1e-9)
    Rc4 = Rc + g2 * (np.abs(theta4_vals) - t4) / denom45
    X4, Y4, S4 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc4, theta4_vals, 4, cross_err)

    theta5_vals = _inclusive_arange(t5, t6, dt)
    denom56 = max(t6 - t5, 1e-9)
    Rc5 = Rc + g2 + (g3 - g2) * (theta5_vals - t5) / denom56
    X5, Y5, S5 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc5, theta5_vals, 5, cross_err)

    theta6_vals = _inclusive_arange(t6, t7, dt)
    Rc6 = np.full_like(theta6_vals, Rc + g3)
    X6, Y6, S6 = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc6, theta6_vals, 6, cross_err)

    theta1_neg = _inclusive_arange(-t7, -t6, dt)
    Rc6n = np.full_like(theta1_neg, Rc + g3)
    X6n, Y6n, S6n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc6n, theta1_neg, 12, cross_err)

    theta2_neg = _inclusive_arange(-t6, -t5, dt)
    Rc5n = Rc + g2 + (g3 - g2) * (np.abs(theta2_neg) - t5) / denom56
    X5n, Y5n, S5n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc5n, theta2_neg, 11, cross_err)

    theta3_neg = _inclusive_arange(-t5, -t4, dt)
    Rc4n = Rc + g2 * (np.abs(theta3_neg) - t4) / denom45
    X4n, Y4n, S4n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc4n, theta3_neg, 10, cross_err)

    theta4_neg = _inclusive_arange(-t4, -t3, dt)
    Rc3n = np.full_like(theta4_neg, Rc)
    X3n, Y3n, S3n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc3n, theta4_neg, 9, cross_err)

    theta5_neg = _inclusive_arange(-t3, -t2, dt)
    Rc2n = Rc + g1 * (t3 - np.abs(theta5_neg)) / denom23
    X2n, Y2n, S2n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc2n, theta5_neg, 8, cross_err)

    theta6_neg = _inclusive_arange(-t2, -t1, dt)
    Rc1n = np.full_like(theta6_neg, Rc + g1)
    X1n, Y1n, S1n = _calculate_segment_coordinates(A, Za, Zb, Rz, Rc1n, theta6_neg, 7, cross_err)

    X_profile = np.concatenate([X6n, X5n, X4n, X3n, X2n, X1n, X1, X2, X3, X4, X5, X6])
    Y_profile = np.concatenate([Y6n, Y5n, Y4n, Y3n, Y2n, Y1n, Y1, Y2, Y3, Y4, Y5, Y6])
    S_profile = np.concatenate([S6n, S5n, S4n, S3n, S2n, S1n, S1, S2, S3, S4, S5, S6])
    return _revolve_and_sort(params, X_profile, Y_profile, S_profile)


def _revolve_and_sort(
    params: CycloidProfileParams,
    X_profile: NDArray[np.float64],
    Y_profile: NDArray[np.float64],
    S_profile: NDArray[np.int_],
) -> Tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.int_]]:
    Zb = int(round(params.z_b))
    num_profiles = Zb - 1
    if num_profiles <= 0:
        raise ValueError("z_b must be greater than 1")

    angle_inc = 360.0 / num_profiles
    base_x = np.asarray(X_profile, dtype=float)
    base_y = np.asarray(Y_profile, dtype=float)
    base_s = np.asarray(S_profile, dtype=int)

    cross_errors = None
    if params.cross_rod_errors is not None:
        cross_errors = np.asarray(params.cross_rod_errors, dtype=float)
        if cross_errors.size < num_profiles:
            raise ValueError("cross_rod_errors length must be at least Zb-1")

    complete_x = None
    complete_y = None
    complete_s = None

    for i in range(num_profiles):
        if cross_errors is not None:
            x_curr, y_curr = _apply_radial_offset(base_x, base_y, cross_errors[i])
        else:
            x_curr, y_curr = base_x, base_y

        x_rot, y_rot = _rotate_deg(x_curr, y_curr, i * angle_inc)
        s_rot = base_s

        if i == 0:
            complete_x = x_rot.copy()
            complete_y = y_rot.copy()
            complete_s = s_rot.copy()
        else:
            complete_x = np.concatenate([complete_x, x_rot[1:]])
            complete_y = np.concatenate([complete_y, y_rot[1:]])
            complete_s = np.concatenate([complete_s, s_rot[1:]])

    theta_all = np.arctan2(complete_y, complete_x)
    order = np.argsort(theta_all)
    return complete_x[order], complete_y[order], complete_s[order]


def _resample_equal_arc_length(
    x: NDArray[np.float64], y: NDArray[np.float64], n_points: int
) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
    if n_points <= 0:
        raise ValueError("n_points must be positive")

    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)

    if not np.allclose([x[0], y[0]], [x[-1], y[-1]]):
        x = np.append(x, x[0])
        y = np.append(y, y[0])

    dx = np.diff(x)
    dy = np.diff(y)
    seg_len = np.hypot(dx, dy)
    arc = np.concatenate(([0.0], np.cumsum(seg_len)))
    total = arc[-1]
    if total <= 0:
        raise ValueError("profile length is zero")

    targets = np.linspace(0.0, total, n_points + 1)[:-1]
    x_new = np.interp(targets, arc, x)
    y_new = np.interp(targets, arc, y)
    return x_new, y_new


PROFILE_BUILDERS = {
    "3seg": _compute_profile_3seg,
    "4seg": _compute_profile_4seg,
    "5seg": _compute_profile_5seg,
}


def generate_cycloid_profile(
    params: CycloidProfileParams,
    *,
    phi_h: float = 0.0,
    n_points: Optional[int] = None,
) -> Tuple[list, list]:
    """Generate cycloid profile coordinates using multi-segment shaping.

    The shaping mode is selected through ``params.profile_mode``.
    
    Args:
        params: Profile parameters (all lengths in consistent units, e.g., meters for SI)
        phi_h: Phase rotation angle in radians (default: 0.0)
        n_points: Number of points for resampling (default: use all generated points)
    
    Returns:
        Tuple of (x_coords, y_coords) in the same units as input parameters.
        For SI units, coordinates are in meters.
    """

    mode = params.profile_mode.lower()
    if mode not in PROFILE_BUILDERS:
        raise ValueError(f"Unsupported profile_mode '{params.profile_mode}'")

    x_profile, y_profile, _ = PROFILE_BUILDERS[mode](params)
    x_profile = np.asarray(x_profile, dtype=float)
    y_profile = np.asarray(y_profile, dtype=float)

    if x_profile.size == 0:
        raise ValueError("profile generation produced no points")

    if phi_h != 0.0:
        cos_h = np.cos(phi_h)
        sin_h = np.sin(phi_h)
        x_profile, y_profile = (
            x_profile * cos_h - y_profile * sin_h,
            x_profile * sin_h + y_profile * cos_h,
        )

    if n_points is None:
        n_points = x_profile.size
    x_final, y_final = _resample_equal_arc_length(x_profile, y_profile, int(n_points))

    return x_final.tolist(), y_final.tolist()


# =================== 曲率计算功能（CasADi 自动微分）===================

try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False


def _build_rc_casadi_3seg(params: CycloidProfileParams, theta):
    """Build CasADi Rc expression for 3seg shaping."""
    import casadi as ca
    Rc_base = float(params.r_z)
    t1 = float(params.theta1_3)
    t2 = float(params.theta2_3)
    t3 = float(params.theta3_3)
    t4 = float(params.theta4_3)
    g1 = float(params.gamma1_3)
    g2 = float(params.gamma2_3)
    
    theta_abs = ca.fabs(theta)
    denom12 = ca.fmax(t2 - t1, 1e-9)
    denom34 = ca.fmax(t4 - t3, 1e-9)
    
    # 段1: |theta| in [t1, t2] -> Rc_base + g1 * (t2 - |theta|) / (t2 - t1)
    # 段2: |theta| in [t2, t3] -> Rc_base
    # 段3: |theta| in [t3, t4] -> Rc_base + g2 * (|theta| - t3) / (t4 - t3)
    Rc_seg1 = Rc_base + g1 * (t2 - theta_abs) / denom12
    Rc_seg2 = Rc_base
    Rc_seg3 = Rc_base + g2 * (theta_abs - t3) / denom34
    
    return ca.if_else(theta_abs <= t2, Rc_seg1,
                      ca.if_else(theta_abs <= t3, Rc_seg2, Rc_seg3))


def _build_rc_casadi_4seg(params: CycloidProfileParams, theta):
    """Build CasADi Rc expression for 4seg shaping."""
    import casadi as ca
    Rc_base = float(params.r_z)
    t1 = float(params.theta1_4)
    t2 = float(params.theta2_4)
    t3 = float(params.theta3_4)
    t4 = float(params.theta4_4)
    t5 = float(params.theta5_4)
    t6 = float(params.theta6_4)
    g1 = float(params.gamma1_4)
    g2 = float(params.gamma2_4)
    
    theta_abs = ca.fabs(theta)
    denom23 = ca.fmax(t3 - t2, 1e-9)
    denom45 = ca.fmax(t5 - t4, 1e-9)
    
    # 段1: [t1, t2] -> Rc_base + g1
    # 段2: [t2, t3] -> Rc_base + g1 * (t3 - |theta|) / (t3 - t2)
    # 段3: [t3, t4] -> Rc_base
    # 段4: [t4, t5] -> Rc_base + g2 * (|theta| - t4) / (t5 - t4)
    # 段5: [t5, t6] -> Rc_base + g2
    Rc_seg1 = Rc_base + g1
    Rc_seg2 = Rc_base + g1 * (t3 - theta_abs) / denom23
    Rc_seg3 = Rc_base
    Rc_seg4 = Rc_base + g2 * (theta_abs - t4) / denom45
    Rc_seg5 = Rc_base + g2
    
    return ca.if_else(theta_abs <= t2, Rc_seg1,
                      ca.if_else(theta_abs <= t3, Rc_seg2,
                                 ca.if_else(theta_abs <= t4, Rc_seg3,
                                            ca.if_else(theta_abs <= t5, Rc_seg4, Rc_seg5))))


def _build_rc_casadi_5seg(params: CycloidProfileParams, theta):
    """Build CasADi Rc expression for 5seg shaping."""
    import casadi as ca
    Rc_base = float(params.r_z)
    t1 = float(params.theta1_5)
    t2 = float(params.theta2_5)
    t3 = float(params.theta3_5)
    t4 = float(params.theta4_5)
    t5 = float(params.theta5_5)
    t6 = float(params.theta6_5)
    t7 = float(params.theta7_5)
    g1 = float(params.gamma1_5)
    g2 = float(params.gamma2_5)
    g3 = float(params.gamma3_5)
    
    theta_abs = ca.fabs(theta)
    denom23 = ca.fmax(t3 - t2, 1e-9)
    denom45 = ca.fmax(t5 - t4, 1e-9)
    denom56 = ca.fmax(t6 - t5, 1e-9)
    
    # 段1: [t1, t2] -> Rc_base + g1
    # 段2: [t2, t3] -> Rc_base + g1 * (t3 - |theta|) / (t3 - t2)
    # 段3: [t3, t4] -> Rc_base
    # 段4: [t4, t5] -> Rc_base + g2 * (|theta| - t4) / (t5 - t4)
    # 段5: [t5, t6] -> Rc_base + g2 + (g3 - g2) * (|theta| - t5) / (t6 - t5)
    # 段6: [t6, t7] -> Rc_base + g3
    Rc_seg1 = Rc_base + g1
    Rc_seg2 = Rc_base + g1 * (t3 - theta_abs) / denom23
    Rc_seg3 = Rc_base
    Rc_seg4 = Rc_base + g2 * (theta_abs - t4) / denom45
    Rc_seg5 = Rc_base + g2 + (g3 - g2) * (theta_abs - t5) / denom56
    Rc_seg6 = Rc_base + g3
    
    return ca.if_else(theta_abs <= t2, Rc_seg1,
                      ca.if_else(theta_abs <= t3, Rc_seg2,
                                 ca.if_else(theta_abs <= t4, Rc_seg3,
                                            ca.if_else(theta_abs <= t5, Rc_seg4,
                                                       ca.if_else(theta_abs <= t6, Rc_seg5, Rc_seg6)))))


def compute_curvature_casadi(
    params: CycloidProfileParams,
    theta_deg_values: NDArray[np.float64],
) -> NDArray[np.float64]:
    """
    使用 CasADi 自动微分精确计算摆线轮曲率（包含完整修形）。
    
    支持 3seg/4seg/5seg 修形模式，根据 params.profile_mode 自动选择。
    
    Parameters
    ----------
    params : CycloidProfileParams
        齿廓参数（包含修形参数）
    theta_deg_values : array
        角度参数值 (度)，范围 [-180, 180]
    
    Returns
    -------
    curvature : 曲率数组 κ = 1/ρ (1/m)
    """
    if not CASADI_AVAILABLE:
        raise ImportError("CasADi is required. Install with: pip install casadi")
    
    # 基础参数
    A = float(params.e)
    Zb = int(round(params.z_b))
    Za = Zb - 1
    Rz = float(params.R_z)
    
    # 符号变量
    theta = ca.SX.sym('theta')
    
    # 根据修形模式构建 Rc 表达式
    mode = params.profile_mode
    if mode == "4seg":
        Rc = _build_rc_casadi_4seg(params, theta)
    elif mode == "5seg":
        Rc = _build_rc_casadi_5seg(params, theta)
    else:  # 默认 3seg
        Rc = _build_rc_casadi_3seg(params, theta)
    
    # 使用核心公式计算坐标（CasADi后端）
    x, y, _, _ = _cycloid_core_formula(theta, A, Za, Zb, Rz, Rc, backend='casadi')
    
    # 自动微分求导数（对 theta 求导）
    dx_dtheta = ca.jacobian(x, theta)
    dy_dtheta = ca.jacobian(y, theta)
    d2x_dtheta2 = ca.jacobian(dx_dtheta, theta)
    d2y_dtheta2 = ca.jacobian(dy_dtheta, theta)
    
    # 曲率公式 κ = (x'y'' - y'x'') / (x'² + y'²)^1.5
    # 正曲率=凸面（向外弯曲），负曲率=凹面（向内弯曲）
    numerator = dx_dtheta * d2y_dtheta2 - dy_dtheta * d2x_dtheta2
    denominator = (dx_dtheta**2 + dy_dtheta**2)**1.5
    curvature_expr = ca.if_else(denominator > 1e-20, numerator / denominator, 0)
    
    # 编译为函数
    curvature_func = ca.Function('curvature', [theta], [curvature_expr])
    
    # 批量计算
    theta_deg_values = np.asarray(theta_deg_values, dtype=float)
    result = np.zeros(len(theta_deg_values))
    for i, t in enumerate(theta_deg_values):
        result[i] = float(curvature_func(t))
    
    return result


def compute_curvature_with_shaping(
    params: CycloidProfileParams,
    use_casadi: bool = True
) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    计算完整齿廓的曲率，包含修形效果。
    
    Parameters
    ----------
    params : CycloidProfileParams
        齿廓参数
    use_casadi : bool
        已忽略，始终使用 CasADi 自动微分
    
    Returns
    -------
    profile_coords : (x, y) 齿廓坐标
    curvature : 曲率数组
    """
    if not CASADI_AVAILABLE:
        raise ImportError("CasADi is required. Install with: pip install casadi")
    
    # 生成齿廓
    x, y = generate_cycloid_profile(params)
    x = np.asarray(x)
    y = np.asarray(y)
    
    # 从坐标反推角度
    angles = np.degrees(np.arctan2(x, y)) % 360
    # 映射到 [0, 180] 范围
    angles = np.where(angles > 180, 360 - angles, angles)
    
    # 使用 CasADi 计算曲率
    curvature = compute_curvature_casadi(params, angles)
    
    return (x, y), curvature
