function measure_spalling_area_curved(rgb_path, depth_path, intrinsics_path)
%MEASURE_SPALLING_AREA_CURVED Measure spalling area on a curved surface.
%   MEASURE_SPALLING_AREA_CURVED(RGB_PATH, DEPTH_PATH, INTRINSICS_PATH)
%   opens an interactive interface. First annotate one or more intact
%   reference regions to reconstruct the local quadratic reference surface,
%   then annotate the spalled regions to measure.
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

    fprintf('Initializing curved-surface spalling-area measurement...\n');

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
    hFig = figure('Name', 'Curved-surface spalling-area measurement', ...
                  'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'figure', ...
                  'Position', [50, 50, 1400, 900]);

    hPanelLeft = uipanel('Parent', hFig, 'Position', [0 0 0.75 1]);
    hPanelRight = uipanel('Parent', hFig, 'Position', [0.75 0 0.25 1], 'Title', 'Controls');

    hAx = axes('Parent', hPanelLeft, 'Position', [0.05 0.05 0.9 0.9]);
    imshow(rgb_img, 'Parent', hAx); hold on;
    title('Step 1: Define the intact reference surface', 'FontSize', 14, 'Color', 'b');

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

        fprintf('Fitting the quadratic reference surface...\n');

        [model_plane, inliers_idx] = custom_ransac_plane(pts_3d, 3.0, 2000); % 3 mm threshold
        if isempty(model_plane) || numel(inliers_idx) < 200
            warning('Too few RANSAC inliers; fitting the quadratic surface using all reference points.');
            inliers_pts = pts_3d;
        else
            inliers_pts = pts_3d(inliers_idx, :);
        end

        REF_PLANE.ref_pts = inliers_pts;
        REF_PLANE = fit_quadric_cap_from_points(REF_PLANE);
        REF_PLANE.type = 'quadric';

        set(hStatus, 'String', 'Reference surface defined', 'ForegroundColor', [0 0.6 0]);
        set(btnMeasure, 'Enable', 'on');

        update_log(sprintf(['Quadratic reference surface fitted:\n' ...
            'coeff=[a b c d e f]=[%.3e %.3e %.3e %.3e %.3e %.3e]\n' ...
            'k1=%.3e, k2=%.3e (1/mm)\nR1=%.1fmm, R2=%.1fmm'], ...
            REF_PLANE.quad.coef(1), REF_PLANE.quad.coef(2), REF_PLANE.quad.coef(3), ...
            REF_PLANE.quad.coef(4), REF_PLANE.quad.coef(5), REF_PLANE.quad.coef(6), ...
            REF_PLANE.quad.k1, REF_PLANE.quad.k2, REF_PLANE.quad.R1_mm, REF_PLANE.quad.R2_mm));

        title(hAx, 'Reference surface locked. Spalling regions can now be measured.', 'Color', 'k');
    end

    function mode_explosion_measure()
        if isempty(REF_PLANE)
            errordlg('Define the intact reference surface first.'); return;
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
    fprintf('\n========== Curved-surface spalling-area measurement: %s ==========\n', label_str);

    if ~isfield(ref_plane, 'type') || ~strcmp(ref_plane.type, 'quadric') || ~isfield(ref_plane, 'quad')
        warning('Quadratic reference parameters are missing. Define the intact reference surface first.');
        real_area_cm2 = 0;
        return;
    end

    % Restrict ray-surface intersections to the spalling-mask bounding box.
    [vv, uu] = find(mask);
    if isempty(vv)
        real_area_cm2 = 0;
        return;
    end
    vmin = max(min(vv)-1, 1);
    vmax = min(max(vv)+1, size(mask,1));
    umin = max(min(uu)-1, 1);
    umax = min(max(uu)+1, size(mask,2));

    subMask = mask(vmin:vmax, umin:umax);
    [U, V] = meshgrid(umin:umax, vmin:vmax);
    U = double(U);
    V = double(V);

    % Cast one camera ray per pixel and intersect it with the reference surface.
    Dx = (U - p.cx) / p.fx;
    Dy = (V - p.cy) / p.fy;
    Dz = ones(size(Dx));

    [t_hit, hit_valid] = intersect_rays_with_quadric(Dx, Dy, Dz, ref_plane.quad);
    hit_valid = hit_valid & subMask;

    if nnz(hit_valid) < 10
        warning('Too few valid ray-surface intersections; area may be unstable.');
        real_area_cm2 = 0;
        return;
    end

    X = t_hit .* Dx;
    Y = t_hit .* Dy;
    Z = t_hit .* Dz;
    X(~hit_valid) = NaN;
    Y(~hit_valid) = NaN;
    Z(~hit_valid) = NaN;

    % Integrate surface area over the pixel grid using two triangles per cell.
    area_mm2 = surface_area_sum_from_pixelgrid(X, Y, Z, subMask);
    real_area_cm2 = area_mm2 / 100.0;
    fprintf('  Surface area on the quadratic reference: %.3f cm^2\n', real_area_cm2);

    try
        visualize_quadric_fit_and_patch(ref_plane, X, Y, Z, label_str);
    catch ME
        warning('SpallingArea:VizFailed', '%s', ['3D visualization failed: ' ME.message]);
    end
end

function REF = fit_quadric_cap_from_points(REF)
    P = REF.ref_pts;
    if size(P,1) < 200
        error('Too few reference points for stable quadratic-surface fitting.');
    end

    % Local coordinates use the minimum-variance PCA direction as e3.
    c0 = mean(P, 1);
    Q = P - c0;
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

    a = coef(1); b = coef(2); c = coef(3); d = coef(4); e = coef(5);
    ws = d;
    wt = e;
    wss = 2*a;
    wtt = 2*b;
    wst = c;

    denom = 1 + ws^2 + wt^2;
    H = ((1+wt^2)*wss - 2*ws*wt*wst + (1+ws^2)*wtt) / (2 * denom^(3/2));
    K = (wss*wtt - wst^2) / (denom^2);

    disc = max(H^2 - K, 0);
    k1 = H + sqrt(disc);
    k2 = H - sqrt(disc);

    R1 = inf;
    R2 = inf;
    if abs(k1) > 1e-12, R1 = 1/abs(k1); end
    if abs(k2) > 1e-12, R2 = 1/abs(k2); end

    REF.quad.origin = c0(:);
    REF.quad.R = R;
    REF.quad.coef = coef(:).';
    REF.quad.k1 = k1;
    REF.quad.k2 = k2;
    REF.quad.R1_mm = R1;
    REF.quad.R2_mm = R2;
end

function [t_hit, valid] = intersect_rays_with_quadric(Dx, Dy, Dz, quad)
    sz = size(Dx);
    dx = Dx(:);
    dy = Dy(:);
    dz = Dz(:);

    Rm = quad.R;
    o = quad.origin(:);
    c = quad.coef(:);
    a = c(1); b = c(2); cc = c(3); d = c(4); e = c(5); f = c(6);

    D = [dx, dy, dz];
    Dloc = D * Rm;
    al1 = Dloc(:,1);
    al2 = Dloc(:,2);
    al3 = Dloc(:,3);

    oloc = (-o.') * Rm;
    be1 = oloc(1);
    be2 = oloc(2);
    be3 = oloc(3);

    A2 = a*(al1.^2) + b*(al2.^2) + cc*(al1.*al2);
    A1 = a*(2*be1*al1) + b*(2*be2*al2) + cc*(be2*al1 + be1*al2) ...
       + d*al1 + e*al2 - al3;
    A0_scalar = a*(be1.^2) + b*(be2.^2) + cc*(be1*be2) + d*be1 + e*be2 + f - be3;
    A0 = A0_scalar * ones(size(al1));

    t_hit = nan(size(dx));
    valid = false(size(dx));

    lin_mask = abs(A2) < 1e-12;
    if any(lin_mask)
        tl = -A0(lin_mask) ./ A1(lin_mask);
        ok = isfinite(tl) & (tl > 0);
        t_hit(lin_mask) = tl;
        valid(lin_mask) = ok;
    end

    quad_mask = ~lin_mask;
    if any(quad_mask)
        Aq = A2(quad_mask);
        Bq = A1(quad_mask);
        Cq = A0(quad_mask);

        disc = Bq.^2 - 4*Aq.*Cq;
        ok_disc = disc >= 0;
        tq = nan(size(Aq));

        if any(ok_disc)
            sqrtD = sqrt(disc(ok_disc));
            Aq2 = Aq(ok_disc);
            Bq2 = Bq(ok_disc);

            t1 = (-Bq2 - sqrtD) ./ (2*Aq2);
            t2 = (-Bq2 + sqrtD) ./ (2*Aq2);

            tmin = nan(size(t1));
            both = (t1 > 0) & (t2 > 0);
            tmin(both) = min(t1(both), t2(both));
            only1 = (t1 > 0) & ~(t2 > 0);
            tmin(only1) = t1(only1);
            only2 = (t2 > 0) & ~(t1 > 0);
            tmin(only2) = t2(only2);
            tq(ok_disc) = tmin;
        end

        t_hit(quad_mask) = tq;
        valid(quad_mask) = isfinite(tq) & (tq > 0);
    end

    valid = valid & (t_hit > 0) & (t_hit < 10000);
    t_hit = reshape(t_hit, sz);
    valid = reshape(valid, sz);
end

function area_mm2 = surface_area_sum_from_pixelgrid(X, Y, Z, subMask)
    [H, W] = size(subMask);
    area_mm2 = 0;

    for i = 1:H-1
        for j = 1:W-1
            if ~(subMask(i,j) && subMask(i,j+1) && subMask(i+1,j) && subMask(i+1,j+1))
                continue;
            end

            p00 = [X(i,j),   Y(i,j),   Z(i,j)];
            p10 = [X(i,j+1), Y(i,j+1), Z(i,j+1)];
            p01 = [X(i+1,j), Y(i+1,j), Z(i+1,j)];
            p11 = [X(i+1,j+1), Y(i+1,j+1), Z(i+1,j+1)];

            if any(isnan([p00 p10 p01 p11]))
                continue;
            end

            area_mm2 = area_mm2 + tri_area_3d(p00, p10, p01) ...
                + tri_area_3d(p11, p01, p10);
        end
    end
end

function A = tri_area_3d(p1, p2, p3)
    a = p2 - p1;
    b = p3 - p1;
    A = 0.5 * norm(cross(a, b));
end

function visualize_quadric_fit_and_patch(ref_plane, Xcap, Ycap, Zcap, label_str)
    if ~isfield(ref_plane, 'ref_pts') || ~isfield(ref_plane, 'quad')
        return;
    end

    P = ref_plane.ref_pts;
    q = ref_plane.quad;

    N = size(P,1);
    step = max(round(N/4000), 1);
    Ps = P(1:step:end, :);

    figure('Name', ['Quadratic surface reconstruction: ', label_str], ...
        'NumberTitle', 'off', 'Color', 'w', 'Position', [100 80 1400 650]);

    ax1 = subplot(1,2,1);
    scatter3(ax1, Ps(:,1), Ps(:,2), Ps(:,3), 6, Ps(:,3), 'filled');
    grid(ax1,'on');
    axis(ax1,'equal');
    xlabel(ax1,'X(mm)'); ylabel(ax1,'Y(mm)'); zlabel(ax1,'Z(mm)');
    title(ax1, 'Intact reference point cloud (subsampled)');
    rotate3d(ax1,'on');

    ax2 = subplot(1,2,2);
    wExag = 80;
    local_all = (P - q.origin(:).') * q.R;
    s_all = local_all(:,1);
    t_all = local_all(:,2);

    smin = prctile(s_all, 2);  smax = prctile(s_all, 98);
    tmin = prctile(t_all, 2);  tmax = prctile(t_all, 98);

    ns = 80;
    nt = 80;
    [S, T] = meshgrid(linspace(smin, smax, ns), linspace(tmin, tmax, nt));
    coef = q.coef;
    W = coef(1)*S.^2 + coef(2)*T.^2 + coef(3)*S.*T + coef(4)*S + coef(5)*T + coef(6);

    surf(ax2, S, T, W*wExag, 'EdgeColor', 'none', 'FaceAlpha', 0.90);
    camlight(ax2,'headlight');
    lighting(ax2,'gouraud');
    axis(ax2,'vis3d');
    view(ax2, 45, 25);
    hold(ax2,'on');

    idx = find(isfinite(Xcap));
    if ~isempty(idx)
        pick = idx(1:max(round(numel(idx)/2000),1):end);
        Pcap = [Xcap(pick), Ycap(pick), Zcap(pick)];
        PcapL = (Pcap - q.origin(:).') * q.R;
        scatter3(ax2, PcapL(:,1), PcapL(:,2), PcapL(:,3)*wExag, 8, 'k', 'filled');
    end

    grid(ax2,'on');
    axis(ax2,'tight');
    axis(ax2,'vis3d');
    xlabel(ax2,'s (mm)'); ylabel(ax2,'t (mm)'); zlabel(ax2, sprintf('w x %g (mm)', wExag));
    title(ax2, 'Quadratic reference surface and spalling footprint (exaggerated w axis)');
    camlight(ax2,'headlight');
    lighting(ax2,'gouraud');
    rotate3d(ax2,'on');

    txt = sprintf('k1=%.3e 1/mm, k2=%.3e 1/mm\\nR1=%.1f mm, R2=%.1f mm', ...
        q.k1, q.k2, q.R1_mm, q.R2_mm);
    text(ax2, 0, 0, 0, ['  ',txt], 'Color','r', 'FontSize',11, 'FontWeight','bold');
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
