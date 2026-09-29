clc; clear; close all;

%% ===== USER SETTINGS =====
pcapFile  = "reading7.pcap"; 
imuFile   = "imu_all_data_2025-12-21_17-16-25.csv";
veloModel = "VLP16";

% --- SEGMENTATION SETTING ---
maxFrames = 1000; % Set this to 2000 to process only the first segment

% ROI & Downsampling
roi = [-40 40 -40 40 -1.5 10];   
downsampleRes = 0.2;   
ndtGridSize   = 3.0;    

%% ===== LOAD IMU DATA =====
fprintf('Loading IMU data...\n');
imuData = readtable(imuFile);
imuTime = imuData.Timestamp;
posX = imuData.Pos_X;
posY = imuData.Pos_Y;

%% ===== INITIALIZE SLAM =====
veloReader = velodyneFileReader(pcapFile, veloModel);
globalMap   = [];
globalTform = rigidtform3d; 
trajectory  = [];           
frameID     = 0;
lidarDt     = 0.1; 

%% ===== MAIN LOOP =====
fprintf('Starting SLAM (Processing first %d frames)...\n', maxFrames);

% The loop now checks BOTH if the file has frames AND if we are under the limit
while hasFrame(veloReader) && frameID < maxFrames
    frameID = frameID + 1;
    currentTime = (frameID-1) * lidarDt;
    
    % 1. Sync IMU (Interpolation)
    curIMU_X = interp1(imuTime, posX, currentTime, 'linear', 'extrap');
    curIMU_Y = interp1(imuTime, posY, currentTime, 'linear', 'extrap');
    
    if frameID == 1
        lastIMU = [curIMU_X, curIMU_Y];
        initialGuess = globalTform;
    else
        dTrans = [curIMU_X - lastIMU(1), curIMU_Y - lastIMU(2), 0];
        predTranslation = globalTform.Translation + dTrans;
        initialGuess = rigidtform3d(globalTform.R, predTranslation);
        lastIMU = [curIMU_X, curIMU_Y];
    end
    
    % 2. Get and Clean LiDAR Frame
    ptCloudRaw = readFrame(veloReader);
    indices = findPointsInROI(ptCloudRaw, roi);
    ptCloud = select(ptCloudRaw, indices);
    ptCloud = pcdownsample(ptCloud, 'gridAverage', downsampleRes);
    
    if ptCloud.Count < 300, continue; end
    
    if isempty(globalMap)
        globalMap = ptCloud;
        trajectory = [trajectory; [0 0 0]];
        continue;
    end
    
    % 3. Registration (NDT)
    try
        [tform, ~] = pcregisterndt(ptCloud, globalMap, ndtGridSize, ...
                             "InitialTransform", initialGuess, ...
                             "MaxIterations", 50);
        
        % Force Z=0 and Extract Yaw Only
        newTrans = tform.Translation;
        newTrans(3) = 0; 
        eul = rotm2eul(tform.R); 
        yawOnly = eul(1); 
        
        newRot = eul2rotm([yawOnly, 0, 0]); 
        globalTform = rigidtform3d(newRot, newTrans);
    catch
        globalTform = initialGuess;
    end
    
    % 4. Build the Map
    if frameID == 1 || norm(globalTform.Translation - trajectory(end,:)) > 0.2
        ptCloudGlobal = pctransform(ptCloud, globalTform);
        globalMap = pcmerge(globalMap, ptCloudGlobal, downsampleRes);
        trajectory = [trajectory; globalTform.Translation];
    end
    
    % Progress Update
    if mod(frameID, 100) == 0
        fprintf('Progress: %d/%d frames processed...\n', frameID, maxFrames);
    end
end

%% ===== SAVE DATA =====
% Appending the frame count to the filename so you know which segment this is
mapName = sprintf('Map_Segment_1_%d.ply', maxFrames);
dataName = sprintf('Data_Segment_1_%d.mat', maxFrames);

pcwrite(globalMap, mapName); 
save(dataName, 'trajectory', 'downsampleRes', 'ndtGridSize');

fprintf('\nSUCCESS: Segment processing complete.\n');
fprintf('Saved: %s and %s\n', mapName, dataName);

%% ===== FINAL VISUALIZATION =====
figure('Name', 'Segmented Map', 'Color', 'k');
pcshow(globalMap); 
hold on;
plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'r', 'LineWidth', 2);
title(['Map Segment: First ', num2str(maxFrames), ' Frames']);
axis equal; view(3);