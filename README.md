# Measurement of Concrete Spall Depth and Area Using a Single RGB-D Image Captured from Arbitrary Perspectives
The paper is currently under review. This repository provides Spall to Full for semantics-guided depth completion and MATLAB tools for single-view RGB-D quantification of concrete spalling area and depth on planar and curved surfaces.

# RGB-D Spalling Measurement

This folder contains the MATLAB implementation used to measure concrete spalling geometry from a single registered RGB-D frame. Separate tools are provided for planar and curved reference surfaces.

## Files

| File | Purpose |
| --- | --- |
| `measure_spalling_area_planar.m` | Spalling-area measurement using an intact planar reference surface. |
| `measure_spalling_area_curved.m` | Spalling-area measurement using a locally fitted quadratic reference surface. |
| `measure_spalling_depth_planar.m` | Point-wise spalling-depth measurement relative to a RANSAC-fitted reference plane. |
| `measure_spalling_depth_curved.m` | Point-wise spalling-depth measurement relative to a locally fitted quadratic reference surface. |

## Requirements

- MATLAB R2025a (original implementation environment).
- Image Processing Toolbox is required for interactive ROIs and image processing.
- Computer Vision Toolbox is additionally required by `measure_spalling_depth_planar.m` (`pointCloud` and `pcfitplane`).

## Inputs

Each function takes three paths:

1. `rgb_path`: RGB image.
2. `depth_path`: depth image registered to the RGB image. Depth values are expected in millimetres.
3. `intrinsics_path`: Azure Kinect calibration text file containing the color-camera intrinsics and Rational 6KT distortion coefficients (`fx`, `fy`, `cx`, `cy`, `k1`-`k6`, `p1`, and `p2`).

The RGB and depth images should describe the same frame. If their image sizes differ, the depth image is resized to the RGB resolution using nearest-neighbour interpolation.

## Quick start

```matlab
rgb_path = "path/to/rgb.png";
depth_path = "path/to/depth.png";
intrinsics_path = "path/to/calibration.txt";

% Choose one measurement tool:
measure_spalling_area_planar(rgb_path, depth_path, intrinsics_path);
measure_spalling_area_curved(rgb_path, depth_path, intrinsics_path);
measure_spalling_depth_planar(rgb_path, depth_path, intrinsics_path);
measure_spalling_depth_curved(rgb_path, depth_path, intrinsics_path);
```

Run only the function corresponding to the surface geometry and quantity being measured.

## Interaction

### Area measurement

1. Draw one or more intact reference regions. Double-click to close each polygon and finish the current ROI.
2. Finish reference selection when prompted.
3. Click **Measure spalling area**, draw the spalling ROI, and repeat if multiple spalled regions are present.
4. The total surface area is reported in `cm^2`.

### Depth measurement

1. Draw intact regions used to reconstruct the reference plane or curved surface.
2. In the point-measurement window, left-click the spalling locations to measure.
3. Press `z` for zoom mode, `p` for pan mode, and `Esc` or `Enter` to finish.
4. Point-wise depths are reported in millimetres. The curved-surface tool also displays 2D and 3D result views.

## Notes

- The code internally applies the Rational 6KT distortion correction before geometric reconstruction.
- Invalid or missing depth values should be encoded as zero (or values below 10 mm); these pixels are excluded or filled according to the measurement workflow.
- The supplied depth should already be aligned to the RGB camera coordinate system.
