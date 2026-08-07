# Measurement of Concrete Spall Depth and Area Using a Single RGB-D Image Captured from Arbitrary Perspectives
The paper is currently under review. This repository provides Spall to Full for semantics-guided depth completion and MATLAB tools for single-view RGB-D quantification of concrete spalling area and depth on planar and curved surfaces.

Regarding the measurement of concrete spalling in complex scenarios such as corresponding fires, please refer to the website “https://github.com/MikeyDong/Spall-to-Full”。

# RGB-D Spalling Measurement

MATLAB tools for measuring concrete spalling area and depth from a registered RGB-D frame using planar or curved reference surfaces.

## Files

| File | Purpose |
| --- | --- |
| `run_measurement.m` | Entry script for setting input paths and selecting a measurement tool. |
| `visualize_valid_depth.m` | Preview valid-depth coverage before selecting reference regions. |
| `measure_spalling_area_planar.m` | Spalling-area measurement with a planar reference surface. |
| `measure_spalling_area_curved.m` | Spalling-area measurement with a quadratic reference surface. |
| `measure_spalling_depth_planar.m` | Point-wise spalling-depth measurement with a planar reference surface. |
| `measure_spalling_depth_curved.m` | Point-wise spalling-depth measurement with a quadratic reference surface. |

## Requirements

- MATLAB R2025a.
- Image Processing Toolbox.
- Computer Vision Toolbox for `measure_spalling_depth_planar.m` (`pointCloud` and `pcfitplane`).

## Usage

Set the RGB, aligned depth, and calibration paths in `run_measurement.m`:

```matlab
rgb_path = "path/to/rgb_image.png";
depth_path = "path/to/depth_image.png";
intrinsics_path = "path/to/intrinsics.txt";
```

Keep one measurement call active and run `run_measurement.m`. The script first calls `visualize_valid_depth.m` automatically. Blue pixels indicate valid depth after distortion correction (`alpha = 0.65`), while unshaded pixels indicate missing or invalid depth. Inspect the coverage, click **OK** to continue, and use the preview to help select intact reference regions with dense valid-depth coverage.

## Interaction

### Planar and curved area measurement

1. Click **1. Define intact reference**, draw an intact polygon, and double-click to finish it. Use **Add** to select another reference region or **Done** to fit the reference surface.
2. Click **2. Measure spalling area**, draw the spalling region, and double-click to finish the polygon. Use **Continue** for another spalling region or **Finish** to complete the measurement.
3. Individual and total spalling areas are reported in cm². 

### Planar depth measurement

1. Draw an intact reference polygon and double-click to finish it. Use **Continue** to add another reference region or **Finish** to reconstruct the reference plane.
2. Left-click a spalling location to measure its depth.
3. Press `z` for zoom mode (left-click to zoom in, right-click to zoom out), then press any key to return to point measurement. Press `p` for pan mode, drag to pan, then press any key to return.
4. Press `Esc` or `Enter` to finish. Depth is reported in mm.

### Curved depth measurement

1. Draw an intact reference polygon and double-click to finish it. Press `Space` to add another reference region, or `Esc`/`Enter` to finish reference selection and reconstruct the quadratic surface.
2. Left-click a spalling location to measure its depth.
3. Press `z` for zoom mode (left-click to zoom in, right-click to zoom out), then press any key to return to point measurement. Press `p` for pan mode, drag to pan, then press any key to return.
4. Press `Esc` or `Enter` to finish. Depth is reported in mm.

## Notes

- RGB and depth images must represent the same frame; depth must be registered to the RGB image and expressed in millimetres.
- The calibration file must contain the color-camera intrinsics and Rational 6KT distortion parameters (`fx`, `fy`, `cx`, `cy`, `k1`-`k6`, `p1`, `p2`). Distortion correction is applied internally.
- Keep `run_measurement.m`, `visualize_valid_depth.m`, and the four measurement functions in the same folder or on the MATLAB path.
- When possible, use at least two intact reference regions. Prefer regions close to the measurement target and with dense valid-depth coverage; avoid areas dominated by missing depth.
- The valid-depth preview is only a selection aid and does not modify the RGB or depth inputs used by the measurement functions.

## Disclaimer

Measurement accuracy can be affected by depth noise or missing depth, RGB-depth registration and camera calibration errors, viewing distance and angle, surface condition, reference-region placement, and manual ROI or point selection. Results should therefore be checked against the available depth coverage and the requirements of the intended application.
