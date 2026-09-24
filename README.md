Photogrammetric Space Resection

Estimate a camera's exterior orientation — position (X0, Y0, Z0) and rotation (omega, phi, kappa) — from 3D control points and their corresponding 2D image coordinates.

This project originated from a graduate Close-Range Photogrammetry assignment and was refactored into a small, reproducible Python project.

Method Overview

The implementation estimates the six exterior-orientation parameters of a camera:

Position: X0, Y0, Z0
Rotation: omega, phi, kappa
Processing Pipeline
flowchart LR
    A["3D Control Points<br/>(X, Y, Z)"] --> B["Camera Coordinate<br/>Transformation"]
    B --> C["Collinearity<br/>Equations"]
    C --> D["Lens<br/>Distortion"]
    D --> E["Predicted Image<br/>Coordinates"]
    E --> F["Reprojection<br/>Residuals"]
    F --> G["Nonlinear<br/>Least Squares"]
    G --> H["Estimated<br/>Camera Pose"]

The processing workflow is:

Transform object-space 3D points into the camera coordinate system using the omega–phi–kappa rotation model.
Compute normalized image coordinates using the photogrammetric collinearity equations.
Apply radial and tangential lens distortion.
Convert the coordinates into image-space pixel coordinates.
Estimate the exterior-orientation parameters by minimizing reprojection error using nonlinear least squares.
Inputs
3D ground/control points
Corresponding 2D image coordinates
Known camera intrinsics
Outputs
Camera position: X0, Y0, Z0
Camera rotation: omega, phi, kappa
Reprojection RMSE
Why This Project?

Space resection is a fundamental problem in photogrammetry and 3D computer vision.

Given known 3D–2D correspondences and camera intrinsics, the goal is to recover the camera pose that best explains the observed image measurements.

The implementation includes:

Photogrammetric collinearity equations
Omega–Phi–Kappa rotation convention
Radial lens distortion (k1, k2, k3)
Tangential lens distortion (p1, p2)
Nonlinear least-squares adjustment
Reprojection RMSE for accuracy assessment
Metashape and OpenCV tangential-distortion conventions
Repository Structure
photogrammetric-space-resection/
├── README.md
├── requirements.txt
├── .gitignore
├── pose_recovery.png
├── src/
│   └── space_resection.py
├── examples/
│   └── synthetic_demo.py
└── tests/
    └── test_synthetic.py
Installation
git clone https://github.com/Reza-pourali/photogrammetric-space-resection.git
cd photogrammetric-space-resection
pip install -r requirements.txt
Quick Start
import numpy as np
from src.space_resection import CameraIntrinsics, solve_space_resection

ground_points = np.array([
    [0.0, 0.0, 0.0],
    [2.0, 0.0, 0.0],
    [2.0, 2.0, 0.0],
    [0.0, 2.0, 0.0],
    [0.5, 0.5, 0.5],
    [1.5, 0.5, 0.8],
])

image_points = np.array([
    # [u, v] image coordinates in pixels
])

camera = CameraIntrinsics(
    f=3000.0,
    cx=2000.0,
    cy=1500.0,
    k1=0.0,
    k2=0.0,
    k3=0.0,
    p1=0.0,
    p2=0.0,
    distortion_convention="metashape",
)

result = solve_space_resection(
    ground_points,
    image_points,
    camera,
    initial_guess=[1.0, 1.0, 5.0, 0.0, 0.0, 0.0],
)

print(result)

Angles in initial_guess and the returned result are expressed in degrees.

Synthetic Validation

A reproducible validation experiment is included in:

examples/synthetic_demo.py

Run it with:

python examples/synthetic_demo.py

The experiment:

defines a camera with known pose and intrinsics
generates 3D control points
projects them into image space
adds small image-coordinate noise
estimates the camera pose again
Pose Recovery Results
Parameter	True Value	Estimated Value	Absolute Error
X0	0.6000	0.5986	0.0014
Y0	-0.4000	-0.4010	0.0010
Z0	8.0000	8.0004	0.0004
omega (deg)	2.0000	2.0071	0.0071
phi (deg)	-3.0000	-3.0098	0.0098
kappa (deg)	1.2000	1.1935	0.0065

Reprojection RMSE: 0.0966 px

The estimated camera pose closely matches the known synthetic ground truth.

3D Visualization

The figure below shows the synthetic 3D control points together with the true and estimated camera positions.




Testing

Run the automated test with:

python -m unittest tests/test_synthetic.py

The noise-free synthetic pose-recovery test verifies that the implementation can recover the known camera pose to numerical precision.

Camera Model

For each 3D point, coordinates are transformed from the object coordinate system into the camera coordinate system.

Normalized image coordinates are then computed using the collinearity equations.

Lens distortion is applied in normalized coordinates before conversion to image-space pixels using focal length and principal point.

Note: Metashape and OpenCV use different conventions for the two tangential-distortion coefficients. The implementation allows the convention to be selected explicitly.

Current Scope

This public version contains the verified collinearity-equation / nonlinear least-squares implementation.

The original coursework also compared space resection against Direct Linear Transformation (DLT).

A cleaned and independently validated DLT implementation may be added as a future extension.

Research Relevance

This project represents part of my transition from photogrammetry toward 3D computer vision and geometric perception.

It demonstrates practical experience with:

Camera geometry
Photogrammetric modeling
3D coordinate transformations
Lens-distortion modeling
Nonlinear optimization
Scientific Python
Reproducible computational experiments

My broader research interests include:

3D Computer Vision
Point Cloud Processing
Deep Learning
Human/Object Tracking
LiDAR Perception
Photogrammetry
Academic Context

Graduate coursework in Close-Range Photogrammetry
K. N. Toosi University of Technology

Author

Reza Pourali
M.Sc. Student in Photogrammetry
K. N. Toosi University of Technology

Research interests: 3D Computer Vision • Point Clouds • Deep Learning • Photogrammetry
