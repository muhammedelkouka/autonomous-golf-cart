%clc; clear; close all;

%% ===== USER SETTINGS =====
pcapFile  = "reading5.pcap";   % Your PCAP file
veloModel = "VLP16";

% ROI: [Xmin Xmax Ymin Ymax Zmin Zmax]
roi = [-30 30 -30 30 -2 3];   

% Tuning Parameters for Open Areas
downsampleRes = 0.30;   
ndtGridSize   = 2.5;    
maxMapPoints  = 500000; 

%% ===== INITIALIZE READER =====
try
    veloReader = velodyneFileReader(pcapFile, veloModel);
catch
    error('Cannot find or open the PCAP file. Check the filename.');
end

%% ===== INITIALIZE VARIABLES =====
globalMap   = [];
globalTform = rigidtform3d; % Current estimated pose
trajectory  = [];           
frameID     = 0;

hFig = figure('Name', 'LiDAR SLAM Progress');
hAx = axes(hFig);

%% ===== MAIN LOOP (Frame-to-Map) =====
while hasFrame(veloReader)
    frameID = frameID + 1;
    
    % 1. Read and Filter
    ptCloudRaw = readFrame(veloReader);
    indices = findPointsInROI(ptCloudRaw, roi);
    ptCloud = select(ptCloudRaw, indices);
    
    % 2. Downsample
    ptCloud = pcdownsample(ptCloud, 'gridAverage', downsampleRes);
    
    if ptCloud.Count < 300
        continue;
    end
    
    % 3. First Frame Initialization
    if isempty(globalMap)
        globalMap = ptCloud;
        trajectory = [trajectory; globalTform.Translation];
        continue;
    end
    
    % 4. Registration (Frame-to-Map)
    try
        % Using 'InitialTransform' helps the NDT converge in open spaces
        [tform, movingReg] = pcregisterndt(ptCloud, globalMap, ndtGridSize, ...
                             "InitialTransform", globalTform, ...
                             "MaxIterations", 40);
        
        % In some versions, RMSE isn't returned directly. 
        % If you need the error value, you can calculate it or use:
        % rmseValue = movingReg.Count; % Placeholder or custom metric
        
        globalTform = tform;
        
    catch
        warning('Frame %d: Registration failed.', frameID);
        continue;
    end
    
    % 5. Transform Current Scan to Global Frame
    ptCloudGlobal = pctransform(ptCloud, globalTform);
    
    % 6. Merge into Global Map
    globalMap = pcmerge(globalMap, ptCloudGlobal, downsampleRes);
    
    % Store trajectory
    trajectory = [trajectory; globalTform.Translation];
    
    % 7. Fixed Console Logging & Live View
    if mod(frameID, 10) == 0
        % FIXED: Avoided using rmse variable that was causing the error
        fprintf('Frame %d | Map Points: %d\n', frameID, globalMap.Count);
        
        if mod(frameID, 20) == 0
            pcshow(ptCloudGlobal, 'Parent', hAx);
            hold(hAx, 'on');
            plot3(hAx, trajectory(:,1), trajectory(:,2), trajectory(:,3), 'r', 'LineWidth', 2);
            hold(hAx, 'off');
            drawnow limitrate;
        end
    end
    
    % Memory safety
    if globalMap.Count > maxMapPoints
        globalMap = pcdownsample(globalMap, 'gridAverage', downsampleRes * 1.2);
    end
end

%% ===== FINAL VISUALIZATION =====
figure('Name','Final 3D Reconstructed Map');
pcshow(globalMap);
hold on;
plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'r-', 'LineWidth', 3);
title('Final SLAM Map & Trajectory');
grid on; axis equal;

%% ===== SAVE RESULTS =====
pcwrite(globalMap, 'final_environment_map.pcd');
save('slam_results.mat', 'globalMap', 'trajectory');
disp('Mapping complete.');