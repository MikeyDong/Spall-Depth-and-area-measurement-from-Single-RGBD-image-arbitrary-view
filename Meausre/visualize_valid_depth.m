function valid_mask = visualize_valid_depth(rgb_path, depth_path, intrinsics_path)
%VISUALIZE_VALID_DEPTH Preview valid depth coverage on the RGB image.
%   Blue overlay = valid depth after Rational 6KT distortion correction.
%   Unshaded pixels = missing/invalid depth. Overlay alpha is fixed at 0.65.

narginchk(3, 3)

rgb_path = string(rgb_path);
depth_path = string(depth_path);
intrinsics_path = string(intrinsics_path);

OVERLAY_ALPHA = 0.65;
MIN_VALID_DEPTH_MM = 10;

if ~isfile(rgb_path), error('RGB image not found: %s', rgb_path); end
if ~isfile(depth_path), error('Depth image not found: %s', depth_path); end
if ~isfile(intrinsics_path), error('Calibration file not found: %s', intrinsics_path); end

% Read the first calibration group (color camera; depth is RGB-aligned).
raw_txt = fileread(intrinsics_path);

cam_fx = read_required_value(raw_txt, 'fx:');
cam_fy = read_required_value(raw_txt, 'fy:');
cam_cx = read_required_value(raw_txt, 'cx:');
cam_cy = read_required_value(raw_txt, 'cy:');

dist_k1 = read_required_value(raw_txt, 'k1:');
dist_k2 = read_required_value(raw_txt, 'k2:');
dist_k3 = read_required_value(raw_txt, 'k3:');
dist_p1 = read_required_value(raw_txt, 'p1:');
dist_p2 = read_required_value(raw_txt, 'p2:');

dist_k4 = read_optional_value(raw_txt, 'k4:', 0);
dist_k5 = read_optional_value(raw_txt, 'k5:', 0);
dist_k6 = read_optional_value(raw_txt, 'k6:', 0);

cam_params.fx = cam_fx;
cam_params.fy = cam_fy;
cam_params.cx = cam_cx;
cam_params.cy = cam_cy;
cam_params.k = [dist_k1, dist_k2, dist_k3, dist_k4, dist_k5, dist_k6];
cam_params.p = [dist_p1, dist_p2];

% Load the registered RGB-D pair.
img_rgb_raw = imread(rgb_path);
img_depth_raw = double(imread(depth_path));

[h_rgb, w_rgb, ~] = size(img_rgb_raw);
[h_d, w_d] = size(img_depth_raw);

if h_rgb ~= h_d || w_rgb ~= w_d
    img_depth_raw = imresize(img_depth_raw, [h_rgb, w_rgb], 'nearest');
end

% Apply the same Rational 6KT mapping used by the measurement functions.
[map_x, map_y] = init_undistort_map(w_rgb, h_rgb, cam_params);

img_rgb = zeros(size(img_rgb_raw), class(img_rgb_raw));
for c = 1:size(img_rgb_raw, 3)
    img_rgb(:,:,c) = interp2(double(img_rgb_raw(:,:,c)), map_x, map_y, 'linear', 0);
end
img_rgb = uint8(img_rgb);

img_depth = interp2(img_depth_raw, map_x, map_y, 'nearest', 0);

raw_mask = map_x >= 1 & map_x <= w_rgb & ...
           map_y >= 1 & map_y <= h_rgb;
valid_mask = raw_mask & isfinite(img_depth) & img_depth >= MIN_VALID_DEPTH_MM;

% RGB base image with a blue overlay on valid-depth pixels.
figure('Name', 'Valid-depth coverage', 'NumberTitle', 'off', ...
       'Toolbar', 'figure', 'Position', [100, 100, 1200, 800]);
imshow(img_rgb);
hold on;

blue_layer = zeros(h_rgb, w_rgb, 3);
blue_layer(:,:,3) = 1;
h_overlay = imshow(blue_layer);
set(h_overlay, 'AlphaData', OVERLAY_ALPHA * double(valid_mask));

valid_fraction = nnz(valid_mask) / max(nnz(raw_mask), 1);
title(sprintf('Blue = valid depth | coverage within image bounds: %.1f%%', ...
    100 * valid_fraction));
hold off;
end


function value = read_required_value(raw_txt, key)
idx = strfind(raw_txt, key);
if isempty(idx)
    error('Calibration key not found: %s', key);
end
value = sscanf(raw_txt(idx(1)+length(key):end), '%f', 1);
end


function value = read_optional_value(raw_txt, key, default_value)
idx = strfind(raw_txt, key);
if isempty(idx)
    value = default_value;
else
    value = sscanf(raw_txt(idx(1)+length(key):end), '%f', 1);
end
end


function [map_u, map_v] = init_undistort_map(W, H, params)
[u_grid, v_grid] = meshgrid(1:W, 1:H);

x = (u_grid - params.cx) / params.fx;
y = (v_grid - params.cy) / params.fy;

r2 = x.^2 + y.^2;
r4 = r2.^2;
r6 = r2.^3;

numerator = 1 + params.k(1)*r2 + params.k(2)*r4 + params.k(3)*r6;
denominator = 1 + params.k(4)*r2 + params.k(5)*r4 + params.k(6)*r6;
radial_scale = numerator ./ denominator;

x_tan = 2 * params.p(1) * x .* y + params.p(2) * (r2 + 2 * x.^2);
y_tan = params.p(1) * (r2 + 2 * y.^2) + 2 * params.p(2) * x .* y;

x_distorted = x .* radial_scale + x_tan;
y_distorted = y .* radial_scale + y_tan;

map_u = x_distorted * params.fx + params.cx;
map_v = y_distorted * params.fy + params.cy;
end
