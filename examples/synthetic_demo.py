"""Reproducible synthetic example for the space-resection solver."""

import numpy as np
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.space_resection import CameraIntrinsics, project_points, solve_space_resection


def main():
    rng = np.random.default_rng(42)

    camera = CameraIntrinsics(
        f=3200.0,
        cx=2000.0,
        cy=1500.0,
        k1=-0.03,
        k2=0.004,
        k3=0.0,
        p1=0.0005,
        p2=-0.0003,
        distortion_convention="metashape",
    )

    ground_points = np.array(
        [
            [-2.0, -1.5, 0.0],
            [ 2.0, -1.5, 0.2],
            [ 2.0,  1.5, 0.0],
            [-2.0,  1.5, 0.3],
            [-1.0, -0.5, 1.0],
            [ 1.0, -0.5, 0.8],
            [ 1.2,  0.8, 1.2],
            [-0.8,  0.9, 0.7],
            [ 0.0,  0.0, 1.5],
            [ 0.5,  1.1, 0.4],
        ],
        dtype=float,
    )

    true_exterior_deg = np.array([0.6, -0.4, 8.0, 2.0, -3.0, 1.2])
    true_exterior_rad = true_exterior_deg.copy()
    true_exterior_rad[3:6] = np.deg2rad(true_exterior_rad[3:6])

    image_points = project_points(ground_points, true_exterior_rad, camera)
    image_points += rng.normal(0.0, 0.15, image_points.shape)

    result = solve_space_resection(
        ground_points,
        image_points,
        camera,
        initial_guess=[0.4, -0.2, 7.5, 1.5, -2.5, 0.8],
    )

    print("True vs estimated exterior orientation")
    print("-" * 50)
    print(f"X0       true={true_exterior_deg[0]: .4f}  estimated={result['X0']: .4f}")
    print(f"Y0       true={true_exterior_deg[1]: .4f}  estimated={result['Y0']: .4f}")
    print(f"Z0       true={true_exterior_deg[2]: .4f}  estimated={result['Z0']: .4f}")
    print(f"omega    true={true_exterior_deg[3]: .4f}  estimated={result['omega_deg']: .4f} deg")
    print(f"phi      true={true_exterior_deg[4]: .4f}  estimated={result['phi_deg']: .4f} deg")
    print(f"kappa    true={true_exterior_deg[5]: .4f}  estimated={result['kappa_deg']: .4f} deg")
    print(f"\nReprojection RMSE: {result['rmse_px']:.4f} px")
    print(f"Converged: {result['success']} ({result['message']})")


if __name__ == "__main__":
    main()
