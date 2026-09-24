import unittest
import numpy as np

from src.space_resection import CameraIntrinsics, project_points, solve_space_resection


class TestSpaceResection(unittest.TestCase):
    def test_recovers_noise_free_pose(self):
        camera = CameraIntrinsics(
            f=2500.0,
            cx=1600.0,
            cy=1200.0,
            k1=-0.02,
            k2=0.003,
            p1=0.0004,
            p2=-0.0002,
            distortion_convention="metashape",
        )

        ground = np.array(
            [
                [-2.0, -1.0, 0.0],
                [ 2.0, -1.0, 0.2],
                [ 2.0,  1.0, 0.0],
                [-2.0,  1.0, 0.4],
                [-1.0, -0.4, 1.0],
                [ 1.0, -0.5, 0.8],
                [ 0.8,  0.7, 1.1],
                [-0.7,  0.8, 0.6],
            ],
            dtype=float,
        )

        true_deg = np.array([0.5, -0.3, 7.0, 1.5, -2.0, 0.8])
        true_rad = true_deg.copy()
        true_rad[3:6] = np.deg2rad(true_rad[3:6])
        image = project_points(ground, true_rad, camera)

        result = solve_space_resection(
            ground,
            image,
            camera,
            initial_guess=[0.4, -0.2, 6.7, 1.2, -1.7, 0.5],
        )

        self.assertTrue(result["success"])
        self.assertLess(result["rmse_px"], 1e-6)
        self.assertAlmostEqual(result["X0"], true_deg[0], places=6)
        self.assertAlmostEqual(result["Y0"], true_deg[1], places=6)
        self.assertAlmostEqual(result["Z0"], true_deg[2], places=6)
        self.assertAlmostEqual(result["omega_deg"], true_deg[3], places=6)
        self.assertAlmostEqual(result["phi_deg"], true_deg[4], places=6)
        self.assertAlmostEqual(result["kappa_deg"], true_deg[5], places=6)


if __name__ == "__main__":
    unittest.main()
