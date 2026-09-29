%clc; clear; close all;

%% ===== USER SETTINGS =====
pcapFile  = "reading5.pcap"; 
veloModel = "VLP16";

% ROI Tuning: Narrow the Z-range to focus on walls/ground (Ignore sky noise)
% Looking at your image, points at Z > 10 seem like noise or distant trees.
roi = [-40 40 -40 40 -1.5 4];   

% SLAM Hyperparameters
downsampleRes = 0.20;   % Slightly denser for better feature tracking
ndtGridSize   = 3.0;    % Larger grid to handle the "Open Area" sparsity
maxMapPoints  = 800000;

%% ===== INITIALIZE =====
veloReader = velodyneFileReader(pcapFile, veloModel);
globalMap   = [];
globalTform = rigidtform3d; 
trajectory  = [];           
frameID     = 0;

%% ===== MAIN LOOP =====
while hasFrame(veloReader)
    frameID = frameID + 1;
    
    ptCloudRaw = readFrame(veloReader);
    indices = findPointsInROI(ptCloudRaw, roi);
    ptCloud = select(ptCloudRaw, indices);
    ptCloud = pcdownsample(ptCloud, 'gridAverage', downsampleRes);
    
    if ptCloud.Count < 400, continue; end
    
    if isempty(globalMap)
        globalMap = ptCloud;
        trajectory = [trajectory; globalTform.Translation];
        continue;
    end
    
    % REGISTRATION WITH HIGHER TOLERANCE FOR TURNS
    try
        % MaxIterations increased to 100 to help during the corridor-to-open turn
        [tform, movingReg] = pcregisterndt(ptCloud, globalMap, ndtGridSize, ...
                             "InitialTransform", globalTform, ...
                             "MaxIterations", 100, ...
                             "OutlierRatio", 0.5); % Ignores moving objects/noise
        
        globalTform = tform;
    catch
        continue;
    end
    
    % Update Map
    ptCloudGlobal = pctransform(ptCloud, globalTform);
    
    % Only merge every 2nd frame to keep map from bloating too fast
    if mod(frameID, 2) == 0
        globalMap = pcmerge(globalMap, ptCloudGlobal, downsampleRes);
    end
    
    trajectory = [trajectory; globalTform.Translation];
    
    if mod(frameID, 20) == 0
        fprintf('Frame %d | Map Points: %d | Z-Pos: %.2f\n', ...
                frameID, globalMap.Count, globalTform.Translation(3));
    end
end

%% ===== VISUALIZE =====
figure;
pcshow(globalMap); title('Improved Open Area Map');
hold on;
plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'r', 'LineWidth', 3);