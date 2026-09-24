"""Photogrammetric space resection using collinearity equations.

The solver estimates six exterior-orientation parameters:
    X0, Y0, Z0, omega, phi, kappa

Camera intrinsics are treated as known.
"""

from dataclasses import dataclass
from typing import Literal, Sequence
import numpy as np
from scipy.optimize import least_squares


DistortionConvention = Literal["metashape", "opencv"]


@dataclass(frozen=True)
class CameraIntrinsics:
    """Known camera interior-orientation and lens-distortion parameters.

    Parameters
    ----------
    f:
        Focal length in pixels.
    cx, cy:
        Principal-point coordinates in pixels in the image-coordinate system
        used by the supplied observations.
    k1, k2, k3:
        Radial distortion coefficients.
    p1, p2:
        Tangential distortion coefficients.
    distortion_convention:
        "metashape" or "opencv". The two systems place/name the tangential
        coefficients differently.
    """

    f: float
    cx: float
    cy: float
    k1: float = 0.0
    k2: float = 0.0
    k3: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    distortion_convention: DistortionConvention = "metashape"


def rotation_matrix(omega: float, phi: float, kappa: float) -> np.ndarray:
    """Return the photogrammetric omega-phi-kappa rotation matrix.

    Angles are in radians.
    """
    r_omega = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(omega), np.sin(omega)],
            [0.0, -np.sin(omega), np.cos(omega)],
        ]
    )
    r_phi = np.array(
        [
            [np.cos(phi), 0.0, -np.sin(phi)],
            [0.0, 1.0, 0.0],
            [np.sin(phi), 0.0, np.cos(phi)],
        ]
    )
    r_kappa = np.array(
        [
            [np.cos(kappa), np.sin(kappa), 0.0],
            [-np.sin(kappa), np.cos(kappa), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return r_kappa @ r_phi @ r_omega


def distort_normalized(
    x: np.ndarray,
    y: np.ndarray,
    camera: CameraIntrinsics,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply radial and tangential distortion to normalized coordinates."""
    r2 = x * x + y * y
    radial = 1.0 + camera.k1 * r2 + camera.k2 * r2**2 + camera.k3 * r2**3

    if camera.distortion_convention == "metashape":
        # Metashape frame-camera convention.
        x_tan = camera.p1 * (r2 + 2.0 * x * x) + 2.0 * camera.p2 * x * y
        y_tan = camera.p2 * (r2 + 2.0 * y * y) + 2.0 * camera.p1 * x * y
    elif camera.distortion_convention == "opencv":
        # OpenCV Brown-Conrady convention.
        x_tan = 2.0 * camera.p1 * x * y + camera.p2 * (r2 + 2.0 * x * x)
        y_tan = camera.p1 * (r2 + 2.0 * y * y) + 2.0 * camera.p2 * x * y
    else:
        raise ValueError(
            "distortion_convention must be either 'metashape' or 'opencv'"
        )

    return x * radial + x_tan, y * radial + y_tan


def project_points(
    ground_points: np.ndarray,
    exterior: Sequence[float],
    camera: CameraIntrinsics,
) -> np.ndarray:
    """Project Nx3 object points to Nx2 image coordinates.

    `exterior` is [X0, Y0, Z0, omega, phi, kappa], with angles in radians.
    """
    ground = np.asarray(ground_points, dtype=float)
    if ground.ndim != 2 or ground.shape[1] != 3:
        raise ValueError("ground_points must have shape (N, 3)")

    X0, Y0, Z0, omega, phi, kappa = map(float, exterior)
    R = rotation_matrix(omega, phi, kappa)

    relative = ground - np.array([X0, Y0, Z0], dtype=float)
    camera_xyz = (R @ relative.T).T

    z = camera_xyz[:, 2]
    if np.any(np.isclose(z, 0.0)):
        raise ValueError("At least one point projects at or near infinity (Zc ≈ 0).")

    # Photogrammetric collinearity sign convention.
    x_norm = -camera_xyz[:, 0] / z
    y_norm = -camera_xyz[:, 1] / z

    x_dist, y_dist = distort_normalized(x_norm, y_norm, camera)

    u = camera.cx + camera.f * x_dist
    v = camera.cy + camera.f * y_dist
    return np.column_stack([u, v])


def reprojection_residuals(
    exterior: Sequence[float],
    ground_points: np.ndarray,
    image_points: np.ndarray,
    camera: CameraIntrinsics,
) -> np.ndarray:
    """Return flattened observed-minus-predicted image residuals."""
    predicted = project_points(ground_points, exterior, camera)
    observed = np.asarray(image_points, dtype=float)
    return (observed - predicted).ravel()


def solve_space_resection(
    ground_points: np.ndarray,
    image_points: np.ndarray,
    camera: CameraIntrinsics,
    initial_guess: Sequence[float] | None = None,
) -> dict:
    """Estimate camera exterior orientation by nonlinear least squares.

    Parameters
    ----------
    ground_points:
        Nx3 object/control coordinates.
    image_points:
        Nx2 observed image coordinates in pixels.
    camera:
        Known camera intrinsics.
    initial_guess:
        [X0, Y0, Z0, omega_deg, phi_deg, kappa_deg].
        A reasonable approximation is recommended.

    Returns
    -------
    dict
        Estimated pose, optimization status, and reprojection RMSE.
    """
    ground = np.asarray(ground_points, dtype=float)
    image = np.asarray(image_points, dtype=float)

    if ground.ndim != 2 or ground.shape[1] != 3:
        raise ValueError("ground_points must have shape (N, 3)")
    if image.ndim != 2 or image.shape[1] != 2:
        raise ValueError("image_points must have shape (N, 2)")
    if len(ground) != len(image):
        raise ValueError("ground_points and image_points must contain the same N")
    if len(ground) < 6:
        raise ValueError("Use at least 6 control points for this implementation.")
    if camera.f <= 0:
        raise ValueError("Focal length must be positive.")

    if initial_guess is None:
        center = np.mean(ground, axis=0)
        span = float(np.max(np.ptp(ground, axis=0)))
        if span <= 0:
            raise ValueError("Control points must span a non-zero spatial extent.")
        initial = np.array(
            [center[0], center[1], center[2] + 2.0 * span, 0.0, 0.0, 0.0],
            dtype=float,
        )
    else:
        if len(initial_guess) != 6:
            raise ValueError("initial_guess must contain exactly 6 values.")
        initial = np.asarray(initial_guess, dtype=float).copy()
        initial[3:6] = np.deg2rad(initial[3:6])

    result = least_squares(
        reprojection_residuals,
        initial,
        args=(ground, image, camera),
        method="lm",
    )

    X0, Y0, Z0, omega, phi, kappa = result.x
    rmse = float(np.sqrt(np.mean(result.fun**2)))

    return {
        "X0": float(X0),
        "Y0": float(Y0),
        "Z0": float(Z0),
        "omega_deg": float(np.rad2deg(omega)),
        "phi_deg": float(np.rad2deg(phi)),
        "kappa_deg": float(np.rad2deg(kappa)),
        "rmse_px": rmse,
        "success": bool(result.success),
        "message": str(result.message),
        "nfev": int(result.nfev),
    }
