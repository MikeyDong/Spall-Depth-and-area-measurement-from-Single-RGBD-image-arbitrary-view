function measure_spalling_depth_planar(rgb_path, depth_path, intrinsics_path)
%MEASURE_SPALLING_DEPTH_PLANAR Measure spalling depth on a planar surface.
%   MEASURE_SPALLING_DEPTH_PLANAR(RGB_PATH, DEPTH_PATH, INTRINSICS_PATH)
%   reconstructs a planar intact reference from interactively annotated
%   regions and reports depth at interactively selected spalling points.
%
%   RGB_PATH         Registered RGB image.
%   DEPTH_PATH       Metric depth image aligned with RGB (depth in mm).
%   INTRINSICS_PATH  Azure Kinect calibration text file containing
%                    fx, fy, cx, cy, k1-k6, p1, and p2.

narginchk(3, 3)
rgb_path = string(rgb_path);
depth_path = string(depth_path);
intrinsics_path = string(intrinsics_path);
rng(42)

DEFAULT_EDGE_CROP = 10;

if ~isfile(rgb_path), error('RGB image not found: %s', rgb_path); end
if ~isfile(depth_path), error('Depth image not found: %s', depth_path); end
if ~isfile(intrinsics_path), error('Calibration file not found: %s', intrinsics_path); end

% Read camera intrinsics and distortion coefficients.
fid = fopen(intrinsics_path, 'r');
raw_txt = fscanf(fid, '%c');
fclose(fid);

try
    k_fx = strfind(raw_txt, 'fx:'); k_fy = strfind(raw_txt, 'fy:');
    k_cx = strfind(raw_txt, 'cx:'); k_cy = strfind(raw_txt, 'cy:');
    cam_fx = sscanf(raw_txt(k_fx(1)+3:end), '%f', 1);
    cam_fy = sscanf(raw_txt(k_fy(1)+3:end), '%f', 1);
    cam_cx = sscanf(raw_txt(k_cx(1)+3:end), '%f', 1);
    cam_cy = sscanf(raw_txt(k_cy(1)+3:end), '%f', 1);

    k_k1 = strfind(raw_txt, 'k1:'); k_k2 = strfind(raw_txt, 'k2:'); k_k3 = strfind(raw_txt, 'k3:');
    k_p1 = strfind(raw_txt, 'p1:'); k_p2 = strfind(raw_txt, 'p2:');

    dist_k1 = sscanf(raw_txt(k_k1(1)+3:end), '%f', 1);
    dist_k2 = sscanf(raw_txt(k_k2(1)+3:end), '%f', 1);
    dist_k3 = sscanf(raw_txt(k_k3(1)+3:end), '%f', 1);
    dist_p1 = sscanf(raw_txt(k_p1(1)+3:end), '%f', 1);
    dist_p2 = sscanf(raw_txt(k_p2(1)+3:end), '%f', 1);

    fprintf('Intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f\n', cam_fx, cam_fy, cam_cx, cam_cy);
catch
    error('Failed to parse the calibration file.');
end

% Load the registered RGB-D pair.
img_rgb_raw = imread(rgb_path);
img_depth_raw = double(imread(depth_path));

[h_rgb, w_rgb, ~] = size(img_rgb_raw);
[h_d, w_d] = size(img_depth_raw);

if h_rgb ~= h_d || w_rgb ~= w_d
    img_depth_raw = imresize(img_depth_raw, [h_rgb, w_rgb], 'nearest');
end

if ~exist('dist_k4', 'var')
    try
        fid = fopen(intrinsics_path, 'r'); raw_txt = fscanf(fid, '%c'); fclose(fid);
        k_k4 = strfind(raw_txt, 'k4:'); k_k5 = strfind(raw_txt, 'k5:'); k_k6 = strfind(raw_txt, 'k6:');
        dist_k4 = sscanf(raw_txt(k_k4(1)+3:end), '%f', 1);
        dist_k5 = sscanf(raw_txt(k_k5(1)+3:end), '%f', 1);
        dist_k6 = sscanf(raw_txt(k_k6(1)+3:end), '%f', 1);
    catch
        error('Failed to parse k4-k6 from the calibration file.');
    end
end

cam_params.fx = cam_fx; cam_params.fy = cam_fy;
cam_params.cx = cam_cx; cam_params.cy = cam_cy;
cam_params.k = [dist_k1, dist_k2, dist_k3, dist_k4, dist_k5, dist_k6];
cam_params.p = [dist_p1, dist_p2];

fprintf('Applying Rational 6KT distortion correction...\n');
[map_x, map_y] = init_undistort_map(w_rgb, h_rgb, cam_params);

img_rgb = zeros(size(img_rgb_raw), class(img_rgb_raw));
for c = 1:size(img_rgb_raw, 3)
    img_rgb(:,:,c) = interp2(double(img_rgb_raw(:,:,c)), map_x, map_y, 'linear', 0);
end
img_rgb = uint8(img_rgb);
img_depth = interp2(img_depth_raw, map_x, map_y, 'nearest', 0);

raw_mask = map_x >= 1 & map_x <= w_rgb & ...
           map_y >= 1 & map_y <= h_rgb;

processed_depth = img_depth;

inv_pix = (img_depth < 10) & raw_mask;
if sum(inv_pix(:)) > 0
    processed_depth = regionfill(processed_depth, inv_pix);
end
fprintf('Fitting the intact reference plane with RANSAC...\n');

h_temp = figure('Name', 'Select intact reference regions', ...
    'NumberTitle', 'off', ...
    'Position', [100, 100, 1200, 800]);

imshow(img_rgb);
hold on;

roi_mask_total = false(h_rgb, w_rgb);

flat_roi_vertices = {};
flat_roi_colors = {};
flat_roi_face_alphas = [];

roi_count = 0;

while true
    title(sprintf(['Draw intact reference region %d\n' ...
        'Click polygon vertices; double-click to close and confirm'], roi_count + 1));

    try
        h_r = drawpolygon( ...
            'Color', 'g', ...
            'LineWidth', 2);

        wait_for_roi(h_r);

        current_vertices = h_r.Position;

        flat_roi_vertices{end + 1} = current_vertices;
        flat_roi_colors{end + 1} = h_r.Color;
        flat_roi_face_alphas(end + 1) = h_r.FaceAlpha;

        roi_mask_total = roi_mask_total | createMask(h_r);

        roi_count = roi_count + 1;

    catch
        if roi_count == 0
            if ishandle(h_temp)
                close(h_temp);
            end
            error('No intact reference region was selected; RANSAC cannot proceed.');
        else
            fprintf('Reference-region selection stopped after %d region(s).\n', roi_count);
            break;
        end
    end

    continue_choice = questdlg( ...
        sprintf('%d reference region(s) selected. Add another?', roi_count), ...
        'Reference regions', ...
        'Continue', ...
        'Finish', ...
        'Continue');

    if isempty(continue_choice) || strcmp(continue_choice, 'Finish')
        break;
    end
end

fprintf('Reference-region selection completed: %d region(s).\n', roi_count);

if ishandle(h_temp)
    close(h_temp);
end

[XX_grid, YY_grid] = meshgrid(1:w_rgb, 1:h_rgb);
Z_raw = processed_depth;

fit_indices = find(roi_mask_total & (Z_raw > 100));

if isempty(fit_indices)
    error('The selected reference regions contain no valid depth for RANSAC fitting.');
end

u_fit = XX_grid(fit_indices);
v_fit = YY_grid(fit_indices);
z_fit = Z_raw(fit_indices);

x_fit = (u_fit - cam_cx) .* z_fit ./ cam_fx;
y_fit = (v_fit - cam_cy) .* z_fit ./ cam_fy;

ptCloud = pointCloud([x_fit, y_fit, z_fit]);
[model, ~, ~] = pcfitplane(ptCloud, 2.0);
param = model.Parameters;

fprintf('RANSAC reference plane: %.6fX + %.6fY + %.6fZ + %.6f = 0\n', ...
    param(1), param(2), param(3), param(4));

X_full = (XX_grid - cam_cx) .* Z_raw ./ cam_fx;
Y_full = (YY_grid - cam_cy) .* Z_raw ./ cam_fy;
Z_full = Z_raw;

denom = norm(param(1:3));

dist_map = (param(1) * X_full + ...
            param(2) * Y_full + ...
            param(3) * Z_full + ...
            param(4)) / denom;

final_depth_map = abs(dist_map);

final_depth_map(~raw_mask) = NaN;
final_depth_map(Z_raw < 10) = NaN;

final_crop_val = DEFAULT_EDGE_CROP;

if final_crop_val > 0
    se = strel('disk', round(final_crop_val));
    final_mask_concrete = imerode(raw_mask, se);
else
    final_mask_concrete = raw_mask;
end

spall_map_final = final_depth_map;
spall_map_final(~final_mask_concrete) = NaN;

h_fig4 = figure('Name', 'Spalling-depth point measurement', 'NumberTitle', 'off', ...
    'Position', [960, 100, 1000, 700], 'Toolbar', 'figure');

img_show = img_rgb;
for c = 1:3
    t = img_show(:,:,c);
    t(~final_mask_concrete) = 0;
    img_show(:,:,c) = t;
end

imshow(img_show);
hold on;

for roi_i = 1:numel(flat_roi_vertices)
    vertices = flat_roi_vertices{roi_i};

    if ~isempty(vertices)
        patch( ...
            vertices(:,1), ...
            vertices(:,2), ...
            flat_roi_colors{roi_i}, ...
            'FaceColor', flat_roi_colors{roi_i}, ...
            'FaceAlpha', flat_roi_face_alphas(roi_i), ...
            'EdgeColor', flat_roi_colors{roi_i}, ...
            'LineWidth', 2, ...
            'HitTest', 'off');
    end
end
title({'Spalling-depth point measurement', ...
       'Green: reference regions | Left click: point | z: zoom | p: pan | Esc/Enter: finish'});

clicked_x = [];
clicked_y = [];
clicked_depth = [];
pt_id = 0;

zoom(h_fig4, 'off');
pan(h_fig4, 'off');

fprintf('\n>>> Spalling-depth point measurement <<<\n');
fprintf('Left click: point; z: zoom; p: pan; Esc/Enter: finish.\n');

while true
    [x, y, button] = ginput(1);

    if isempty(button)
        break;
    end

    % -------------------------------------------------
    % -------------------------------------------------
    if button == 27 || button == 13   % Esc or Enter
        break;
    end

    % -------------------------------------------------
    % -------------------------------------------------
    if button == double('z') || button == double('Z')
        title({'Zoom mode: left click to zoom in, right click to zoom out', ...
               'Press any key to return to point measurement'});
        zoom(h_fig4, 'on');
        waitforbuttonpress;
        zoom(h_fig4, 'off');
        title({'Spalling-depth point measurement', ...
               'Left click: point | z: zoom | p: pan | Esc/Enter: finish'});
        continue;
    end

    % -------------------------------------------------
    % -------------------------------------------------
    if button == double('p') || button == double('P')
        title({'Pan mode: drag to pan', ...
               'Press any key to return to point measurement'});
        pan(h_fig4, 'on');
        waitforbuttonpress;
        pan(h_fig4, 'off');
        title({'Spalling-depth point measurement', ...
               'Left click: point | z: zoom | p: pan | Esc/Enter: finish'});
        continue;
    end

    % -------------------------------------------------
    % -------------------------------------------------
    if button ~= 1
        continue;
    end

    xi = round(x);
    yi = round(y);

    if xi < 1 || xi > w_rgb || yi < 1 || yi > h_rgb
        continue;
    end

    pt_id = pt_id + 1;

    curr_depth = spall_map_final(yi, xi);

    clicked_x(pt_id,1) = xi;
    clicked_y(pt_id,1) = yi;
    clicked_depth(pt_id,1) = curr_depth;

    if ~isnan(curr_depth)
        plot(xi, yi, 'ro', 'MarkerSize', 8, 'LineWidth', 1.8);
        plot(xi, yi, 'r+', 'MarkerSize', 8, 'LineWidth', 1.8);
        text(xi + 12, yi, sprintf('%d: %.2f mm', pt_id, curr_depth), ...
            'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold', ...
            'BackgroundColor', 'k', 'Margin', 1);
    else
        plot(xi, yi, 'co', 'MarkerSize', 8, 'LineWidth', 1.8);
        plot(xi, yi, 'cx', 'MarkerSize', 8, 'LineWidth', 1.8);
        text(xi + 12, yi, sprintf('%d: invalid', pt_id), ...
            'Color', 'c', 'FontSize', 11, 'FontWeight', 'bold', ...
            'BackgroundColor', 'k', 'Margin', 1);
    end

    title({'Spalling-depth point measurement', ...
           sprintf('%d point(s) selected | Left click: point | z: zoom | p: pan | Esc/Enter: finish', pt_id)});
end

hold off;

fprintf('\n\n========================================\n');
fprintf('        Spalling-depth results         \n');
fprintf('========================================\n');

if isempty(clicked_depth)
    fprintf('No points were selected.\n');
else
    fprintf('ID\tX(pixel)\tY(pixel)\tDepth(mm)\n');
    fprintf('----------------------------------------\n');

    valid_idx = ~isnan(clicked_depth);

    for i = 1:numel(clicked_depth)
        if valid_idx(i)
            fprintf('%d\t%d\t\t%d\t\t%.4f\n', i, clicked_x(i), clicked_y(i), clicked_depth(i));
        else
            fprintf('%d\t%d\t\t%d\t\tNaN(invalid)\n', i, clicked_x(i), clicked_y(i));
        end
    end

    valid_depths = clicked_depth(valid_idx);

    if ~isempty(valid_depths)
        fprintf('----------------------------------------\n');
        fprintf('Valid points: %d / %d\n', numel(valid_depths), numel(clicked_depth));
        fprintf('Maximum depth: %.4f mm\n', max(valid_depths));
        fprintf('Minimum depth: %.4f mm\n', min(valid_depths));
        fprintf('Mean depth: %.4f mm\n', mean(valid_depths));
    end
end

fprintf('Measurement completed.\n');
end

function wait_for_roi(h)
    l = addlistener(h,'ROIClicked',@(s,e) roi_double_click_callback(s,e)); uiwait; delete(l);
end
function roi_double_click_callback(~,e)
    if strcmp(e.SelectionType,'double'), uiresume; end
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
