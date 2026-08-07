function measure_spalling_depth_curved(rgb_path, depth_path, intrinsics_path)
%MEASURE_SPALLING_DEPTH_CURVED Measure spalling depth on a curved surface.
%   MEASURE_SPALLING_DEPTH_CURVED(RGB_PATH, DEPTH_PATH, INTRINSICS_PATH)
%   reconstructs a local quadratic intact reference surface from
%   interactively annotated regions and reports depth at selected points.
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
VIS_DOWNSAMPLE_FACTOR = 10;

if ~isfile(rgb_path), error('RGB image not found: %s', rgb_path); end
if ~isfile(depth_path), error('Depth image not found: %s', depth_path); end
if ~isfile(intrinsics_path), error('Calibration file not found: %s', intrinsics_path); end

% Read camera intrinsics and distortion coefficients.
fid = fopen(intrinsics_path, 'r');
raw_txt = fscanf(fid, '%c');
fclose(fid);

try
    % The first group is the color-camera calibration; depth is RGB-aligned.
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
        k_k1 = strfind(raw_txt, 'k1:'); k_k2 = strfind(raw_txt, 'k2:'); k_k3 = strfind(raw_txt, 'k3:');
        k_k4 = strfind(raw_txt, 'k4:'); k_k5 = strfind(raw_txt, 'k5:'); k_k6 = strfind(raw_txt, 'k6:');
        k_p1 = strfind(raw_txt, 'p1:'); k_p2 = strfind(raw_txt, 'p2:');

        dist_k1 = sscanf(raw_txt(k_k1(1)+3:end), '%f', 1);
        dist_k2 = sscanf(raw_txt(k_k2(1)+3:end), '%f', 1);
        dist_k3 = sscanf(raw_txt(k_k3(1)+3:end), '%f', 1);
        dist_k4 = sscanf(raw_txt(k_k4(1)+3:end), '%f', 1);
        dist_k5 = sscanf(raw_txt(k_k5(1)+3:end), '%f', 1);
        dist_k6 = sscanf(raw_txt(k_k6(1)+3:end), '%f', 1);
        dist_p1 = sscanf(raw_txt(k_p1(1)+3:end), '%f', 1);
        dist_p2 = sscanf(raw_txt(k_p2(1)+3:end), '%f', 1);
        fprintf('Loaded Rational 6KT distortion coefficients.\n');
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

fprintf('Distortion correction completed.\n');
raw_mask = map_x >= 1 & map_x <= w_rgb & ...
           map_y >= 1 & map_y <= h_rgb;

processed_depth = img_depth;

inv_pix = (img_depth < 10) & raw_mask;
if sum(inv_pix(:)) > 0
    processed_depth = regionfill(processed_depth, inv_pix);
end
fprintf('Reconstructing the intact quadratic reference surface...\n');

    h_temp = figure('Name', 'Select intact reference regions', 'NumberTitle', 'off', ...
        'Position', [120, 80, 1200, 800], 'Toolbar', 'figure');
    imshow(img_rgb);
    title({'Draw an intact region for reference-surface fitting', ...
           'Double-click to close; Space: add region; Esc/Enter: finish'});

    roi_mask_total = false(h_rgb, w_rgb);
    flat_roi_vertices = {};
    flat_roi_face_alpha = [];
    while true
        try
    h_r = drawpolygon('Color', 'm', 'LineWidth', 2);
    wait_for_roi(h_r);

    flat_roi_vertices{end + 1} = h_r.Position;

    flat_roi_face_alpha(end + 1) = h_r.FaceAlpha;

    roi_mask_total = roi_mask_total | createMask(h_r);

            waitforbuttonpress;
            key = double(get(gcf, 'CurrentCharacter'));

            if key == 27 || key == 13
                break;
            end
        catch
            break;
        end
    end

    close(h_temp);

    [XX_grid, YY_grid] = meshgrid(1:w_rgb, 1:h_rgb);
    Z_raw = processed_depth;

    fit_indices = find(roi_mask_total & (Z_raw > 100));
    if isempty(fit_indices), error('The selected reference regions contain no valid depth.'); end

    u_fit = XX_grid(fit_indices);
    v_fit = YY_grid(fit_indices);
    z_fit = Z_raw(fit_indices);

    x_fit = (u_fit - cam_cx) .* z_fit ./ cam_fx;
    y_fit = (v_fit - cam_cy) .* z_fit ./ cam_fy;

    MODEL_THRESH_MM = 2.0;
    RANSAC_ITERS    = 800;
    MIN_INLIER_RATE = 0.50;

    P_raw = [x_fit, y_fit, z_fit];
    c0_raw = mean(P_raw, 1);

    [~, ~, V_raw] = svd(P_raw - c0_raw, 'econ');

    P_local = (P_raw - c0_raw) * V_raw;

    s_fit = P_local(:,1);
    t_fit = P_local(:,2);
    w_fit = P_local(:,3); % Local surface-normal coordinate.

    [~, inlier_mask] = ransac_quad_surface(s_fit, t_fit, w_fit, MODEL_THRESH_MM, RANSAC_ITERS, MIN_INLIER_RATE);

    fit_pts = P_raw(inlier_mask, :);

    if size(fit_pts, 1) < 50
        warning('Too few RANSAC inliers; fitting the quadratic surface using all reference points.');
        fit_pts = P_raw;
    end

    ref_quad = fit_quadric_cap_from_points_B(fit_pts);

    fprintf('Quadratic reference-surface fitting completed.\n');

    X_meas = (XX_grid - cam_cx) .* Z_raw ./ cam_fx;
    Y_meas = (YY_grid - cam_cy) .* Z_raw ./ cam_fy;
    Z_meas = Z_raw;

    valid_mask_depth = raw_mask & (Z_raw > 10) & isfinite(Z_raw);

    [final_depth_map, ~, ~, ~] = point_to_quadric_distance_map_B( ...
        X_meas, Y_meas, Z_meas, ref_quad, valid_mask_depth);

    % Mask & invalid
    final_depth_map(~raw_mask) = NaN;
    final_depth_map(Z_raw < 10) = NaN;
valid_p = final_depth_map(~isnan(final_depth_map));
if ~isempty(valid_p)
    FIXED_CLIM = [-5, max(10, prctile(valid_p, 99.5))];
else
    FIXED_CLIM = [-5, 20];
end
fprintf('3D display color range: %.1f to %.1f mm\n', FIXED_CLIM(1), FIXED_CLIM(2));

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
    current_vertices = flat_roi_vertices{roi_i};

    patch( ...
        'XData', current_vertices(:,1), ...
        'YData', current_vertices(:,2), ...
        'FaceColor', 'g', ...
        'FaceAlpha', flat_roi_face_alpha(roi_i), ...
        'EdgeColor', 'g', ...
        'LineWidth', 2, ...
        'HitTest', 'off');
end

title({'Spalling-depth point measurement', ...
       'Green: reference regions | Left click: point | z: zoom | p: pan | Esc/Enter: finish'});

clicked_x = [];
clicked_y = [];
clicked_depth = [];
pt_id = 0;

zoom(h_fig4, 'off');
pan(h_fig4, 'off');

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

if ~isempty(clicked_depth)
    fprintf('\n=== Spalling-depth results ===\n');
    fprintf('ID\tX(pixel)\tY(pixel)\tDepth(mm)\n');

    valid_idx = ~isnan(clicked_depth);
    for i = 1:numel(clicked_depth)
        if valid_idx(i)
            fprintf('%d\t%d\t\t%d\t\t%.3f\n', i, clicked_x(i), clicked_y(i), clicked_depth(i));
        else
            fprintf('%d\t%d\t\t%d\t\tNaN(invalid)\n', i, clicked_x(i), clicked_y(i));
        end
    end

    valid_depths = clicked_depth(valid_idx);
    if ~isempty(valid_depths)
        fprintf('----------------------------------------\n');
        fprintf('Valid points: %d / %d\n', numel(valid_depths), numel(clicked_depth));
        fprintf('Maximum depth: %.3f mm\n', max(valid_depths));
        fprintf('Minimum depth: %.3f mm\n', min(valid_depths));
        fprintf('Mean depth: %.3f mm\n', mean(valid_depths));
    end

    figure('Name', '3D spalling-depth results', 'NumberTitle', 'off', 'Position', [120, 120, 1200, 700]);

    step = VIS_DOWNSAMPLE_FACTOR;
    map_vis = spall_map_final(1:step:end, 1:step:end);
    mask_vis = imresize(final_mask_concrete, 1/step, 'nearest');
    map_vis(~mask_vis) = NaN;

    [XX, YY] = meshgrid(1:step:w_rgb, 1:step:h_rgb);

    surf(XX, YY, -map_vis, map_vis, ...
        'EdgeColor', 'none', 'FaceColor', 'interp', 'FaceLighting', 'gouraud');
    hold on;
    colormap(jet(256));
    clim(FIXED_CLIM);
    colorbar;

    valid_idx = ~isnan(clicked_depth);
    if any(valid_idx)
        plot3(clicked_x(valid_idx), clicked_y(valid_idx), -clicked_depth(valid_idx) + 2, ...
            'wp', 'MarkerSize', 14, 'MarkerFaceColor', 'k', 'LineWidth', 1.5);

        for i = find(valid_idx').'
            text(clicked_x(i) + 15, clicked_y(i), -clicked_depth(i) + 2, ...
                sprintf('%d: %.2f mm', i, clicked_depth(i)), ...
                'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold', ...
                'BackgroundColor', 'k', 'Margin', 1);
        end
    end

    axis tight; axis equal; axis vis3d;
    set(gca, 'YDir', 'reverse');
    view(-45, 50);
    light('Position',[0 0 1000], 'Style', 'local');
    material([0.6 0.3 0.2]);
    grid on;
    title('3D spalling-depth results');

    figure('Name', '2D spalling-depth overlay', 'NumberTitle', 'off', 'Position', [150, 150, 1000, 700]);
    imshow(img_rgb); hold on;

    if any(valid_idx)
        plot(clicked_x(valid_idx), clicked_y(valid_idx), 'ro', 'MarkerSize', 9, 'LineWidth', 2);
        plot(clicked_x(valid_idx), clicked_y(valid_idx), 'rx', 'MarkerSize', 9, 'LineWidth', 2);
        for i = find(valid_idx').'
            text(clicked_x(i) + 12, clicked_y(i), sprintf('%d: %.2f mm', i, clicked_depth(i)), ...
                'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold', ...
                'BackgroundColor', 'k', 'Margin', 1);
        end
    end

    invalid_idx = isnan(clicked_depth);
    if any(invalid_idx)
        plot(clicked_x(invalid_idx), clicked_y(invalid_idx), 'co', 'MarkerSize', 9, 'LineWidth', 2);
        plot(clicked_x(invalid_idx), clicked_y(invalid_idx), 'cx', 'MarkerSize', 9, 'LineWidth', 2);
        for i = find(invalid_idx').'
            text(clicked_x(i) + 12, clicked_y(i), sprintf('%d: invalid', i), ...
                'Color', 'c', 'FontSize', 11, 'FontWeight', 'bold', ...
                'BackgroundColor', 'k', 'Margin', 1);
        end
    end

    title('2D spalling-depth overlay');
    hold off;
end
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
function [best_coeff, best_inlier_mask] = ransac_quad_surface(x, y, z, thresh, iters, min_inlier_rate)

    x = x(:); y = y(:); z = z(:);
    N = numel(z);
    if N < 20
        error('Too few points for quadratic-surface fitting: %d', N);
    end

    best_inliers = -inf;
    best_coeff = nan(1,6);
    best_inlier_mask = false(N,1);

    % [x^2, y^2, x*y, x, y, 1] * coeff = z
    Phi = [x.^2, y.^2, x.*y, x, y, ones(N,1)];

    sample_size = 6;
    min_inliers = max(round(min_inlier_rate * N), 20);

    rng(0)

    for k = 1:iters
        idx = randperm(N, sample_size);
        Phi_s = Phi(idx,:);
        z_s = z(idx);

        if rank(Phi_s) < size(Phi,2)
            continue;
        end
        coeff = Phi_s \ z_s;

        z_pred = Phi * coeff;
        residual = abs(z - z_pred);

        inlier_mask = residual < thresh;
        nin = sum(inlier_mask);

        if nin < min_inliers
            continue;
        end

        Phi_in = Phi(inlier_mask,:);
        z_in = z(inlier_mask);

        if rank(Phi_in) < sample_size
            continue;
        end

        coeff_refined = Phi_in \ z_in;

        z_pred2 = Phi * coeff_refined;
        residual2 = abs(z - z_pred2);
        inlier_mask2 = residual2 < thresh;
        nin2 = sum(inlier_mask2);

        if nin2 > best_inliers
            best_inliers = nin2;
            best_coeff = coeff_refined(:).';
            best_inlier_mask = inlier_mask2;
        end
    end

    if any(isnan(best_coeff))
        warning('RANSAC quadratic fitting failed; falling back to least squares using all reference points.');
        best_coeff = (Phi \ z).';
        best_inlier_mask = true(N,1);
    end
end
function quad = fit_quadric_cap_from_points_B(P)

    if size(P,1) < 20
        error('Too few reference points for stable quadratic-surface fitting.');
    end

    c0 = mean(P, 1);
    Q  = P - c0;
    [~,~,V] = svd(Q, 'econ');

    e1 = V(:,1);
    e2 = V(:,2);
    e3 = V(:,3);

    R = [e1, e2, e3];
    local = (P - c0) * R;

    s = local(:,1);
    t = local(:,2);
    w = local(:,3);

    A = [s.^2, t.^2, s.*t, s, t, ones(size(s))];
    coef = A \ w;

    quad.origin = c0(:);
    quad.R = R;
    quad.coef = coef(:).';
end
function [dist_map, Xq, Yq, Zq] = point_to_quadric_distance_map_B(Xm, Ym, Zm, quad, valid_mask)
%   w = a s^2 + b t^2 + c s t + d s + e t + f

    sz = size(Xm);

    dist_map = nan(sz);
    Xq = nan(sz); Yq = nan(sz); Zq = nan(sz);

    idx = find(valid_mask);
    if isempty(idx)
        return;
    end

    P = [Xm(idx), Ym(idx), Zm(idx)];

    Ploc = (P - quad.origin(:).') * quad.R;
    sm = Ploc(:,1);
    tm = Ploc(:,2);
    wm = Ploc(:,3);

    coef = quad.coef(:);
    a = coef(1); b = coef(2); c = coef(3);
    d = coef(4); e = coef(5); f0 = coef(6);

    s = sm;
    t = tm;

    w_init = a.*s.^2 + b.*t.^2 + c.*s.*t + d.*s + e.*t + f0;
    best_obj = (s-sm).^2 + (t-tm).^2 + (w_init-wm).^2;
    best_s = s;
    best_t = t;

    for it = 1:100
        w  = a.*s.^2 + b.*t.^2 + c.*s.*t + d.*s + e.*t + f0;

        fs = 2*a.*s + c.*t + d;
        ft = 2*b.*t + c.*s + e;

        rss = 2*a;
        rtt = 2*b;
        rst = c;

        rw = w - wm;

        g1 = (s - sm) + rw .* fs;
        g2 = (t - tm) + rw .* ft;

        H11 = 1 + fs.^2 + rw .* rss;
        H22 = 1 + ft.^2 + rw .* rtt;
        H12 = fs .* ft + rw .* rst;

        detH = H11 .* H22 - H12 .* H12;
        bad = abs(detH) < 1e-12;
        detH(bad) = 1e-12;

        ds = ( H22 .* g1 - H12 .* g2) ./ detH;
        dt = (-H12 .* g1 + H11 .* g2) ./ detH;

        max_step = 10;   % mm
        step_norm = sqrt(ds.^2 + dt.^2);
        scale = min(1, max_step ./ max(step_norm, 1e-12));
        ds = ds .* scale;
        dt = dt .* scale;

        s_try = s - ds;
        t_try = t - dt;

        w_try = a.*s_try.^2 + b.*t_try.^2 + c.*s_try.*t_try + d.*s_try + e.*t_try + f0;
        obj_try = (s_try-sm).^2 + (t_try-tm).^2 + (w_try-wm).^2;

        improved = obj_try < best_obj;
        best_obj(improved) = obj_try(improved);
        best_s(improved) = s_try(improved);
        best_t(improved) = t_try(improved);

        s(improved) = s_try(improved);
        t(improved) = t_try(improved);
    end

    s = best_s;
    t = best_t;
    wq = a.*s.^2 + b.*t.^2 + c.*s.*t + d.*s + e.*t + f0;

    Qloc = [s, t, wq];
    Q = Qloc * quad.R' + quad.origin(:).';

    dvec = P - Q;
    dist = sqrt(sum(dvec.^2, 2));

    dist_map(idx) = dist;
    Xq(idx) = Q(:,1);
    Yq(idx) = Q(:,2);
    Zq(idx) = Q(:,3);
end
