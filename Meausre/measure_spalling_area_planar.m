function measure_spalling_area_planar(rgb_path, depth_path, intrinsics_path)
%MEASURE_SPALLING_AREA_PLANAR Measure spalling area on a planar surface.
%   MEASURE_SPALLING_AREA_PLANAR(RGB_PATH, DEPTH_PATH, INTRINSICS_PATH)
%   opens an interactive interface. First annotate one or more intact
%   reference regions, then annotate the spalled regions to measure.
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
    REF_PLANE = [];

    if ~isfile(rgb_path), error('RGB image not found: %s', rgb_path); end
    if ~isfile(depth_path), error('Depth image not found: %s', depth_path); end
    if ~isfile(intrinsics_path), error('Calibration file not found: %s', intrinsics_path); end

    fprintf('Initializing planar spalling-area measurement...\n');

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

    try
        fid = fopen(intrinsics_path, 'r'); raw_txt = fscanf(fid, '%c'); fclose(fid);
        k_k4 = strfind(raw_txt, 'k4:'); k_k5 = strfind(raw_txt, 'k5:'); k_k6 = strfind(raw_txt, 'k6:');

        if isempty(k_k4) || isempty(k_k5) || isempty(k_k6)
             dist_k4=0; dist_k5=0; dist_k6=0;
             fprintf('k4-k6 not found; using 0.\n');
        else
             dist_k4 = sscanf(raw_txt(k_k4(1)+3:end), '%f', 1);
             dist_k5 = sscanf(raw_txt(k_k5(1)+3:end), '%f', 1);
             dist_k6 = sscanf(raw_txt(k_k6(1)+3:end), '%f', 1);
        end
    catch
        dist_k4=0; dist_k5=0; dist_k6=0;
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

    rgb_img = img_rgb;
    depth_img = img_depth;
    params = cam_params;
    hFig = figure('Name', 'Planar spalling-area measurement', ...
                  'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'figure', ...
                  'Position', [50, 50, 1400, 900]);

    hPanelLeft = uipanel('Parent', hFig, 'Position', [0 0 0.75 1]);
    hPanelRight = uipanel('Parent', hFig, 'Position', [0.75 0 0.25 1], 'Title', 'Controls');

    hAx = axes('Parent', hPanelLeft, 'Position', [0.05 0.05 0.9 0.9]);
    imshow(rgb_img, 'Parent', hAx); hold on;
    title('Step 1: Define the intact reference plane', 'FontSize', 14, 'Color', 'b');

    uicontrol('Parent', hPanelRight, 'Style', 'text', 'String', 'Status:', ...
              'Position', [20 800 200 20], 'HorizontalAlignment', 'left', 'FontSize', 12);
    hStatus = uicontrol('Parent', hPanelRight, 'Style', 'text', 'String', 'Reference not defined', ...
              'Position', [20 770 200 30], 'HorizontalAlignment', 'left', 'FontSize', 11, 'ForegroundColor', 'r');

    btnRef = uicontrol('Parent', hPanelRight, 'Style', 'pushbutton', 'String', '1. Define intact reference', ...
              'Position', [20 700 250 50], 'FontSize', 11, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.8 0.9 0.8], ...
              'Callback', @(s,e) mode_define_reference());

    btnMeasure = uicontrol('Parent', hPanelRight, 'Style', 'pushbutton', 'String', '2. Measure spalling area', ...
              'Position', [20 630 250 50], 'FontSize', 11, 'FontWeight', 'bold', ...
              'BackgroundColor', [1 0.8 0.8], 'Enable', 'off', ...
              'Callback', @(s,e) mode_explosion_measure());

    uicontrol('Parent', hPanelRight, 'Style', 'pushbutton', 'String', 'Clear annotations', ...
              'Position', [20 560 150 40], 'FontSize', 10, ...
              'Callback', @(s,e) clear_plots());

    hResultText = uicontrol('Parent', hPanelRight, 'Style', 'edit', 'Max', 100, 'HorizontalAlignment', 'left', ...
                            'Position', [20 50 250 400], 'String', 'Waiting for input...', 'FontSize', 10);

    function mode_define_reference()
        title(hAx, 'Draw one or more intact reference regions', 'Color', 'b');

        [h_im, w_im, ~] = size(rgb_img);
        total_mask = false(h_im, w_im);

        is_adding = true;
        roi_count = 1;

        while is_adding
            title(hAx, sprintf('Draw intact reference region %d (double-click to finish)', roi_count), 'Color', 'b');

            roi = drawpolygon(hAx, 'Color', 'g', 'LineWidth', 2, 'Label', sprintf('Ref #%d', roi_count));

            wait(roi);

            if ~isvalid(roi) || isempty(roi.Position)
                break;
            end

            total_mask = total_mask | createMask(roi);

            choice = questdlg('Add another reference region?', 'Reference regions', 'Add', 'Done', 'Add');
            if strcmp(choice, 'Done')
                is_adding = false;
            else
                roi_count = roi_count + 1;
            end
        end

        if sum(total_mask(:)) == 0
            errordlg('No valid reference region was selected.'); return;
        end

        mask = total_mask;
        [pts_3d, ~] = get_3d_points_from_mask(mask, depth_img, params);
        if isempty(pts_3d)
            errordlg('The selected reference region contains no valid depth.'); return;
        end

        fprintf('Fitting the planar reference surface...\n');
        [model, inliers_idx] = custom_ransac_plane(pts_3d, 3.0, 2000); % 3 mm threshold

        if isempty(model)
            errordlg('Reference-plane fitting failed.'); return;
        end

        n_vec = model(1:3);
        D_val = model(4);

        if n_vec(3) > 0
            n_vec = -n_vec;
            D_val = -D_val;
        end

        REF_PLANE.n = n_vec;
        REF_PLANE.D = D_val;
        REF_PLANE.equation = [n_vec, D_val];

        set(hStatus, 'String', 'Reference plane defined', 'ForegroundColor', [0 0.6 0]);
        set(btnMeasure, 'Enable', 'on');

        update_log(sprintf('Reference plane fitted:\nNormal: [%.3f, %.3f, %.3f]\nD: %.3f', ...
            n_vec(1), n_vec(2), n_vec(3), D_val));

        title(hAx, 'Reference plane locked. Spalling regions can now be measured.', 'Color', 'k');

        center_poly = mean(roi.Position, 1);
        quiver(hAx, center_poly(1), center_poly(2), n_vec(1)*50, n_vec(2)*50, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    end

    function mode_explosion_measure()
        if isempty(REF_PLANE)
            errordlg('Define the intact reference plane first.'); return;
        end

        total_area = 0;
        count = 1;
        is_continuing = true;

        while is_continuing
            title(hAx, sprintf('Draw spalling region %d', count), 'Color', 'r');

            roi = drawpolygon(hAx, 'Color', 'r', 'LineWidth', 2, 'Label', sprintf('Exp #%d', count));
            if isempty(roi.Position), break; end

            mask = createMask(roi);

            area_val = calculate_area_projected(mask, params, REF_PLANE, sprintf('Spalling region %d', count));

            total_area = total_area + area_val;

            pos = roi.Position;
            center = mean(pos, 1);
            text(hAx, center(1), center(2), sprintf('%.2f cm^2', area_val), ...
                 'Color', 'y', 'FontSize', 12, 'BackgroundColor', 'k');

            update_log(sprintf('Region %d: %.2f cm^2', count, area_val));

            choice = questdlg('Annotate another spalling region?', 'Measurement', 'Continue', 'Finish', 'Continue');
            if strcmp(choice, 'Finish')
                is_continuing = false;
            else
                count = count + 1;
            end
        end

        if total_area > 0
            msgbox(sprintf('Measurement completed.\nTotal spalling area: %.2f cm^2', total_area));
            update_log(sprintf('=== Total area: %.2f cm^2 ===', total_area));
        end
        title(hAx, 'Ready');
    end

    function real_area_cm2 = calculate_area_projected(mask, p, ref_plane, label_str)
        fprintf('\n========== Spalling-area measurement: %s ==========\n', label_str);

        [v_idx, u_idx] = find(mask);
        num_pts = length(v_idx);
        if num_pts == 0, real_area_cm2 = 0; return; end

        % -----------------------------------------------------------------
        % -----------------------------------------------------------------
        x_norm = (u_idx - p.cx) / p.fx;
        y_norm = (v_idx - p.cy) / p.fy;

        n = ref_plane.n;
        D = ref_plane.D;

        dot_NV = n(1) * x_norm + n(2) * y_norm + n(3);

        dot_NV(abs(dot_NV) < 1e-8) = 1e-8;

        Z_virtual = -D ./ dot_NV;

        valid_mask = (Z_virtual > 0) & (Z_virtual < 10000);
        if sum(valid_mask) == 0
            warning('Reference-plane reconstruction failed. Check the selected intact regions.');
            real_area_cm2 = 0; return;
        end

        Z_final = Z_virtual(valid_mask);

        % -----------------------------------------------------------------
        % -----------------------------------------------------------------
        pixel_areas = (Z_final.^2) ./ (p.fx * p.fy);
        area_proj_mm2 = sum(pixel_areas);

        fprintf('  Projected area: %.2f mm^2\n', area_proj_mm2);

        % -----------------------------------------------------------------
        % -----------------------------------------------------------------
        %

        v_cam = [0; 0; 1]; % Camera optical axis.

        cos_theta = abs(dot(n, v_cam));

        fprintf('  Plane normal n: [%.3f, %.3f, %.3f]\n', n(1), n(2), n(3));
        fprintf('  cos(theta): %.4f (inclination %.1f deg)\n', cos_theta, rad2deg(acos(cos_theta)));

        if cos_theta < 0.087 % cos(85)
            warning('The reference plane is nearly parallel to the viewing rays; area may be unstable.');
            cos_theta = 0.087;
        end

        % -----------------------------------------------------------------
        % -----------------------------------------------------------------
        real_area_mm2 = area_proj_mm2 / cos_theta;

        real_area_cm2 = real_area_mm2 / 100.0;

        fprintf('  Surface area: %.2f cm^2\n', real_area_cm2);

    end
    function [points, valid_idx] = get_3d_points_from_mask(mask, depth_img, p)
        [v, u] = find(mask);
        ind = sub2ind(size(depth_img), v, u);
        z = double(depth_img(ind));

        valid = z > 0;
        z = z(valid); u = u(valid); v = v(valid);

        x = (u - p.cx) .* z / p.fx;
        y = (v - p.cy) .* z / p.fy;
        points = [x, y, z];
        valid_idx = valid;
    end

    function [best_model, best_inliers] = custom_ransac_plane(points, dist_threshold, max_iters)
        N = size(points, 1);
        best_model = []; best_inliers = []; max_cnt = 0;
        if N < 3, return; end

        for i = 1:max_iters
            idx = randperm(N, 3);
            p = points(idx, :);
            n = cross(p(2,:)-p(1,:), p(3,:)-p(1,:));
            nn = norm(n);
            if nn < 1e-6, continue; end
            n = n / nn;
            D = -dot(n, p(1,:));

            dists = abs(points * n' + D);
            inliers = find(dists < dist_threshold);
            if length(inliers) > max_cnt
                max_cnt = length(inliers);
                best_inliers = inliers;
                best_model = [n, D];
            end
        end

        if ~isempty(best_inliers)
            pts_in = points(best_inliers, :);
            mean_p = mean(pts_in, 1);
            centered = pts_in - mean_p;
            [U,~,~] = svd(centered' * centered);
            n_opt = U(:,3)';
            D_opt = -dot(n_opt, mean_p);
            best_model = [n_opt, D_opt];
        end
    end

    function update_log(msg)
        current = get(hResultText, 'String');
        if ischar(current), current = {current}; end
        new_str = [{['[' datestr(now, 'HH:MM:SS') '] ' msg]}; current];
        set(hResultText, 'String', new_str);
        fprintf('%s\n', msg);
    end

    function clear_plots()
        delete(findobj(hAx, 'Type', 'images.roi.Polygon'));
        delete(findobj(hAx, 'Type', 'text'));
        delete(findobj(hAx, 'Type', 'quiver'));
        title(hAx, 'Annotations cleared');
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
