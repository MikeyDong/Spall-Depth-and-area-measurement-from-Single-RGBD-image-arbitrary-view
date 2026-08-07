clear; clc; close all;

%% Input files
% rgb_path = "path/to/rgb_image.png";
% depth_path = "path/to/depth_image.png";
% intrinsics_path = "path/to/intrinsics.txt";
rgb_path = "E:\Azure\Processed_Results_2026.1.20_NoSmooth\Dataset_Summary_5th_Last\rgb\0120陆哥试件_C3NFov_Unbined_Dxx_3072p测深相机不固定更近_frame_000011.png";
depth_path = "E:\Azure\Processed_Results_2026.1.20_NoSmooth\Dataset_Summary_5th_Last\transformed_depth\0120陆哥试件_C3NFov_Unbined_Dxx_3072p测深相机不固定更近_frame_000011.png";
intrinsics_path = "E:\Azure\Processed_Results_1218_NoSmooth\Dataset_Summary_5th_Last\给陆哥图像\rgb\1222HC双边2.5Mpa_C13NFov_Unbined_D60_3072p.txt";;

%% Valid-depth preview
% Blue indicates valid depth. Inspect the coverage before selecting intact
% reference regions, then click OK to start the measurement.
visualize_valid_depth(rgb_path, depth_path, intrinsics_path);
uiwait(msgbox( ...
    'Inspect the blue valid-depth coverage, then click OK to start measurement.', ...
    'Valid-depth preview', 'modal'));

%% Measurement
% Run one measurement function at a time.

% Example: planar spalling-area measurement
measure_spalling_area_planar(rgb_path, depth_path, intrinsics_path);

% Curved-surface spalling-area measurement
% measure_spalling_area_curved(rgb_path, depth_path, intrinsics_path);

% Planar spalling-depth measurement
% measure_spalling_depth_planar(rgb_path, depth_path, intrinsics_path);

% Curved-surface spalling-depth measurement
% measure_spalling_depth_curved(rgb_path, depth_path, intrinsics_path);
