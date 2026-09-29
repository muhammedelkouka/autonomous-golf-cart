%clc; clear; close all;

%% ============================================================
% REFINED SCAN-TO-MAP NDT SLAM (BASED ON CODE A)
% Robust, Stable, Ground Vehicle Friendly
% ============================================================

%% ================= USER SETTINGS ============================
pcapFile  = "reading5.pcap";
veloModel = "VLP16";

roi = [-40 40 -40 40 -1.5 4];

downsampleRes = 0.20;
ndtGridSize   = 3.0;

maxMapPoints  = 800000;
keyframeDist  = 1.0;    % meters
maxStep       = 1.5;    % max allowed motion per frame (m)

%% ================= INITIALIZATION ===========================
veloReader = velodyneFileReader(pcapFile, veloModel);

globalMap   = [];
globalTform = rigidtform3d;
trajectory  = [];

lastKeyPose = globalTform;
frameID = 0;

fprintf('=== Robust Scan-to-Map NDT SLAM Started ===\n');

%% ================= MAIN LOOP ================================
while hasFrame(veloReader)
    frameID = frameID + 1;

    %% ---- Read frame ----
    ptCloudRaw = readFrame(veloReader);
    idx = findPointsInROI(ptCloudRaw, roi);
    ptCloud = select(ptCloudRaw, idx);

    if ptCloud.Count < 500
        continue;
    end

    % Downsample
    ptCloud = pcdownsample(ptCloud, 'gridAverage', downsampleRes);

    % Statistical outlier removal
    ptCloud = pcdenoise(ptCloud, ...
        'NumNeighbors', 20, ...
        'Threshold', 1.0);

    %% ---- Initialize map ----
    if isempty(globalMap)
        globalMap = ptCloud;
        trajectory = [trajectory; globalTform.Translation];
        continue;
    end

    %% ================= SCAN-TO-MAP NDT ======================
    try
        [tform, ~, rmse] = pcregisterndt( ...
            ptCloud, globalMap, ndtGridSize, ...
            'InitialTransform', globalTform, ...
            'MaxIterations', 100, ...
            'OutlierRatio', 0.6);
    catch
        continue;
    end

    % Reject bad registration
    if rmse > 1.0
        warning('Frame %d: High RMSE (%.2f), skipping', frameID, rmse);
        continue;
    end

    % Motion sanity check
    step = norm(tform.Translation - globalTform.Translation);
    if step > maxStep
        warning('Frame %d: Unrealistic motion %.2f m, skipping', frameID, step);
        continue;
    end

    % Accept transform
    globalTform = tform;
    trajectory  = [trajectory; globalTform.Translation];

    %% ================= KEYFRAME MAP UPDATE ==================
    if norm(globalTform.Translation - lastKeyPose.Translation) ...
            > keyframeDist

        ptCloudGlobal = pctransform(ptCloud, globalTform);

        globalMap = pcmerge(globalMap, ...
            ptCloudGlobal, downsampleRes);

        if globalMap.Count > maxMapPoints
            globalMap = pcdownsample(globalMap, ...
                'gridAverage', downsampleRes * 1.5);
        end

        lastKeyPose = globalTform;
    end

    if mod(frameID,20) == 0
        fprintf('Frame %d | Map pts: %d | Pos: [%.2f %.2f %.2f]\n', ...
            frameID, globalMap.Count, globalTform.Translation);
    end
end

%% ================= VISUALIZATION ============================
figure('Name','Robust Scan-to-Map NDT SLAM');
pcshow(globalMap); hold on;
plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), ...
    'r', 'LineWidth', 3);
title('Robust Scan-to-Map NDT SLAM (Filtered)');
axis equal; grid on;

disp('=== SLAM Completed Successfully ===');
