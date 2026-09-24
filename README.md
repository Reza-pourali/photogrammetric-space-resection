# Photogrammetric Space Resection

Estimate a camera's **exterior orientation** — position \((X0, Y0, Z0)\) and rotation \((omega, phi, kappa)\) — from 3D control points and their corresponding 2D image coordinates.

This project originated from a graduate **Close-Range Photogrammetry** assignment and was refactored into a small, reproducible Python project.

## Method Overview

This implementation estimates the six exterior-orientation parameters of a camera:

- **Position:** `X0`, `Y0`, `Z0`
- **Rotation:** `omega`, `phi`, `kappa`

### Processing pipeline

1. Transform object-space 3D points into the camera coordinate system using the **omega–phi–kappa** rotation model.
2. Compute normalized image coordinates using the **photogrammetric collinearity equations**.
3. Apply **radial** and **tangential** lens distortion.
4. Convert distorted normalized coordinates into image-space pixel coordinates using focal length and principal point.
5. Estimate the unknown exterior-orientation parameters by minimizing reprojection error with **nonlinear least squares (Levenberg–Marquardt)**.

### Input and output

**Inputs**
- 3D ground/control points
- 2D image coordinates
- Known camera intrinsics

**Outputs**
- Estimated camera position: `X0`, `Y0`, `Z0`
- Estimated camera rotation: `omega`, `phi`, `kappa`
- Reprojection RMSE in pixels

## Why this project?

Space resection is a core problem in photogrammetry and camera geometry: given known 3D–2D correspondences and camera intrinsics, recover the camera pose that best explains the observed image measurements.

The implementation uses:

- Photogrammetric collinearity equations
- Omega–Phi–Kappa rotation convention
- Radial lens distortion (`k1`, `k2`, `k3`)
- Tangential lens distortion (`p1`, `p2`)
- Nonlinear least-squares adjustment (Levenberg–Marquardt)
- Reprojection RMSE for accuracy assessment

The distortion implementation supports both **Metashape** and **OpenCV** tangential-coefficient conventions.

## Repository structure

```text
photogrammetric-space-resection/
├── README.md
├── requirements.txt
├── .gitignore
├── src/
│   └── space_resection.py
├── examples/
│   └── synthetic_demo.py
└── tests/
    └── test_synthetic.py
```

## Installation

```bash
git clone https://github.com/Reza-pourali/photogrammetric-space-resection.git
cd photogrammetric-space-resection
pip install -r requirements.txt
```

## Quick start

```python
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
```

Angles in `initial_guess` and the returned result are in **degrees**.

## Reproducible synthetic validation

Run:

```bash
python examples/synthetic_demo.py
```

The demo creates synthetic 3D control points, projects them using a known camera pose, adds small image noise, and then estimates the pose again.

Run the test:

```bash
python -m unittest tests/test_synthetic.py
```
## Synthetic validation summary

A reproducible synthetic experiment is included in `examples/synthetic_demo.py`.

In this demo:

- A synthetic camera with known intrinsics and distortion is defined
- A set of 3D control points is generated
- The points are projected into image space using a known camera pose
- Small image noise is added
- The solver estimates the pose back from the noisy observations

### Result

The solver is able to recover the camera pose with **sub-pixel reprojection accuracy** on the synthetic example.

- **Reprojection RMSE:** approximately **0.10 px**
- **Validation type:** known-pose synthetic recovery
- **Unit test status:** passed

This gives a controlled verification that the implementation is numerically consistent before applying it to real measurements.

## Camera model

For a 3D point, coordinates are first transformed from the object coordinate system into the camera coordinate system. Ideal normalized image coordinates are then obtained from the collinearity model. Lens distortion is applied in normalized coordinates, followed by conversion to pixel coordinates using focal length and principal point.

> **Note:** Metashape and OpenCV use different placements/naming conventions for the two tangential distortion coefficients. Select the convention that matches the source of your calibration parameters.

## Current scope

This first public version contains the verified **collinearity / nonlinear least-squares** solution.

The original coursework also compared the result against **Direct Linear Transformation (DLT)**. A cleaned and independently validated DLT implementation can be added as a later extension rather than publishing an unverified draft.

## Why this project matters for my research

This project reflects my academic transition from **photogrammetry** toward **3D computer vision** and **camera geometry**.

It demonstrates hands-on experience with:

- camera modeling
- photogrammetric collinearity equations
- nonlinear optimization
- lens-distortion modeling
- reproducible scientific Python workflows

This repository is also aligned with my broader research interests in:

- 3D Computer Vision
- Point Cloud Processing
- Deep Learning
- Human/Object Tracking
- Photogrammetry

## Academic context

Graduate coursework in Close-Range Photogrammetry  
K. N. Toosi University of Technology

## Author

**Reza Pourali**  
M.Sc. Photogrammetry  
Research interests: 3D Computer Vision, Point Clouds, Deep Learning, Photogrammetry
