%% SIMPLE LiDAR LOCALIZATION WITH REFERENCE PATH AND CURRENT SCAN VISUALIZATION
clear; close all; clc;
fprintf('Starting LiDAR Localization with Reference Path and Current Scan Visualization...\n');

%% 1. LOAD DATA
% Map
mapFile = 'Master_Structural_Ref_Adaptive.ply';
fprintf('Loading map: %s... ', mapFile);
mapCloud = pcread(mapFile);
fprintf('Done (%d points)\n', mapCloud.Count);

% PCAP data
pcapFile = 'reading7.pcap';
fprintf('Opening PCAP: %s... ', pcapFile);
veloReader = velodyneFileReader(pcapFile, 'VLP16');
totalFrames = veloReader.NumberOfFrames;
fprintf('Done (%d frames)\n', totalFrames);

% Reference path
refFile = 'Full_Trajectory.ply';
if exist(refFile, 'file')
    fprintf('Loading reference path: %s... ', refFile);
    refCloud = pcread(refFile);
    refPath = refCloud.Location(:, 1:2); % Use only X,Y
    hasRefPath = true;
    fprintf('Done (%d points)\n', size(refPath, 1));
else
    fprintf('Reference path not found, continuing without it\n');
    hasRefPath = false;
    refPath = [];
end

%% 2. SET PARAMETERS
params.frameRate = 10; % Hz
params.voxelSize = 3.0;
params.gridStep = 0.8;
params.roi = [-40 40 -40 40 -1.5 4.0];
params.minPoints = 150;
params.maxIterations = 60;

% Speed limits (3-7 km/h converted to m/frame)
speed_kmh_min = 3.0;
speed_kmh_max = 7.0;
params.speedMin = speed_kmh_min * 1000/3600 / params.frameRate; % m/frame
params.speedMax = speed_kmh_max * 1000/3600 / params.frameRate; % m/frame

fprintf('Speed limits: %.1f-%.1f km/h (%.3f-%.3f m/frame at %.0f Hz)\n', ...
    speed_kmh_min, speed_kmh_max, params.speedMin, params.speedMax, params.frameRate);

% Reference path guidance
params.rmseHighThreshold = 8;      % RMSE above this is "high"
params.rmseHighFrames = 8;         % How many high-RMSE frames before guidance
params.guidanceStrength = 0.2;     % How strongly to pull towards reference
params.maxGuideDistance = 15.0;    % Max distance to apply guidance

%% 3. INITIALIZE
currentPose = eye(4);
prevScan = [];
trajectory = [];     % Store all positions
speeds = [];         % Store speeds in m/frame
rmseHistory = [];    % Store RMSE values
guidanceActive = 0;  % Count guidance frames
highRMSEcount = 0;   % Count consecutive high-RMSE frames
frameCount = 0;

% Create results folder
resultsDir = sprintf('results_%s', datestr(now, 'yyyymmdd_HHMMSS'));
mkdir(resultsDir);
fprintf('Results will be saved to: %s\n\n', resultsDir);

%% 4. SETUP SIMPLE VISUALIZATION
if totalFrames > 0
    fig = figure('Name', 'LiDAR Localization', 'Position', [100 100 1200 600]);
    
    % Main plot
    subplot(2,2,[1,3]);
    hold on; grid on; axis equal;
    scatter(mapCloud.Location(:,1), mapCloud.Location(:,2), 1, [0.7 0.7 0.7], '.');
    if hasRefPath
        plot(refPath(:,1), refPath(:,2), 'c-', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    end
    hTraj = plot(0,0, 'r-', 'LineWidth', 2, 'DisplayName', 'Trajectory');
    hPos = scatter(0,0, 100, 'g', 'filled', 'DisplayName', 'Current Pose');
    hScan = scatter(0,0, 10, 'g', 'filled', 'DisplayName', 'Current Scan'); % NEW
    xlabel('X (m)'); ylabel('Y (m)'); title('Localization'); legend('Location','best');
    
    % RMSE plot
    subplot(2,2,2);
    hRmse = plot(0,0, 'b-', 'LineWidth', 1.5); hold on;
    yline(params.rmseHighThreshold,'r--','High RMSE');
    xlabel('Frame'); ylabel('RMSE (m)'); title('Registration Error'); grid on; ylim([0 1.5]);
    
    % Speed plot
    subplot(2,2,4);
    hSpeed = plot(0,0, 'g-', 'LineWidth', 1.5); hold on;
    yline(params.speedMin,'g--','Min Speed'); yline(params.speedMax,'r--','Max Speed');
    xlabel('Frame'); ylabel('Speed (m/frame)'); title('Speed'); grid on;
    
    drawnow;
end

%% 5. MAIN PROCESSING LOOP
fprintf('Starting processing...\n');
fprintf('Frame    | RMSE   | Speed  | Guidance | Status\n');
fprintf('---------|--------|--------|----------|--------\n');

startTime = tic;

while hasFrame(veloReader) && frameCount < totalFrames
    frameCount = frameCount + 1;
    
    % Read frame
    scanRaw = readFrame(veloReader);
    
    % Filter ROI and downsample
    indices = findPointsInROI(scanRaw, params.roi);
    scanFiltered = select(scanRaw, indices);
    
    if scanFiltered.Count < params.minPoints
        continue;
    end
    
    scanDown = pcdownsample(scanFiltered, 'gridAverage', params.gridStep);
    
    % Skip first frame
    if isempty(prevScan)
        prevScan = scanDown;
        trajectory(frameCount,:) = currentPose(4,1:3);
        speeds(frameCount) = 0;
        continue;
    end
    
    % Step 1: Consecutive frame registration
    try
        [relTform, ~, rmseLocal] = pcregisterndt(scanDown, prevScan, params.voxelSize, ...
            'MaxIterations', 30, 'Tolerance', [0.01, 0.005]);
        guessPose = relTform.T * currentPose;
    catch
        guessPose = currentPose;
    end
    
    % Step 2: Map registration
    try
        [mapTform, ~, rmse] = pcregisterndt(scanDown, mapCloud, params.voxelSize, ...
            'InitialTransform', affine3d(guessPose), ...
            'MaxIterations', params.maxIterations, ...
            'Tolerance', [0.01, 0.005]);
        
        if all(isfinite(mapTform.T(:)))
            currentPose = mapTform.T;
            prevScan = scanDown;
            regSuccess = true;
        else
            currentPose = guessPose;
            regSuccess = false;
        end
    catch
        currentPose = guessPose;
        regSuccess = false;
        rmse = NaN;
    end
    
    % Store RMSE
    if regSuccess && ~isnan(rmse)
        rmseHistory(frameCount) = rmse;
    else
        rmseHistory(frameCount) = NaN;
    end
    
    % Apply speed constraints (2D only)
    position = currentPose(4,1:3);
    if frameCount > 1
        lastPos = trajectory(frameCount-1, 1:2);
        currentPos2D = position(1:2);
        distance = norm(currentPos2D - lastPos);
        
        % Check speed limits
        if distance > params.speedMax * 1.5
            direction = (currentPos2D - lastPos) / distance;
            currentPos2D = lastPos + direction * params.speedMax;
            position(1:2) = currentPos2D;
            currentPose(4,1:2) = currentPos2D;
        elseif distance < params.speedMin * 0.5 && distance > 0.01
            direction = (currentPos2D - lastPos) / distance;
            currentPos2D = lastPos + direction * params.speedMin;
            position(1:2) = currentPos2D;
            currentPose(4,1:2) = currentPos2D;
        end
        
        actualDistance = norm(position(1:2) - lastPos);
        speeds(frameCount) = actualDistance;
    else
        speeds(frameCount) = 0;
    end
    
    % Step 3: Reference path guidance (if RMSE is high)
    guidanceApplied = false;
    
    if hasRefPath && regSuccess && ~isnan(rmse)
        if rmse > params.rmseHighThreshold
            highRMSEcount = highRMSEcount + 1;
        else
            highRMSEcount = 0;
            guidanceActive = 0;
        end
        
        if highRMSEcount >= params.rmseHighFrames
            guidanceActive = guidanceActive + 1;
            
            % Find nearest reference point
            dists = sqrt(sum((refPath - position(1:2)).^2,2));
            [minDist, nearestIdx] = min(dists);
            
            if minDist < params.maxGuideDistance
                targetPos = refPath(nearestIdx,:);
                guidanceVec = (targetPos - position(1:2)) * params.guidanceStrength;
                newPos2D = position(1:2) + guidanceVec;
                currentPose(4,1:2) = newPos2D;
                position(1:2) = newPos2D;
                guidanceApplied = true;
                
                if mod(guidanceActive, 5) == 0
                    try
                        [newTform, ~, newRmse] = pcregisterndt(scanDown, mapCloud, ...
                            params.voxelSize*0.7, 'MaxIterations', 40, ...
                            'Tolerance', [0.01,0.005], ...
                            'InitialTransform', affine3d(currentPose));
                        if newRmse < params.rmseHighThreshold*0.8
                            currentPose = newTform.T;
                            position = currentPose(4,1:3);
                            highRMSEcount = 0;
                            guidanceActive = 0;
                        end
                    catch
                    end
                end
            end
        end
    end
    
    % Store trajectory
    trajectory(frameCount,:) = position;
    
    %% UPDATE DISPLAY (EVERY 10 FRAMES)
    if mod(frameCount, 10) == 0 && exist('hTraj','var')
        % Main plot
        subplot(2,2,[1,3]);
        set(hTraj, 'XData', trajectory(1:frameCount,1), 'YData', trajectory(1:frameCount,2));
        set(hPos, 'XData', position(1), 'YData', position(2));
        
        % Transform current scan to map frame
        ptScanGlobal = pctransform(scanDown, affine3d(currentPose));
        set(hScan, 'XData', ptScanGlobal.Location(:,1), 'YData', ptScanGlobal.Location(:,2));
        
        % Adjust view
        xlim([position(1)-30, position(1)+30]);
        ylim([position(2)-30, position(2)+30]);
        
        % RMSE plot
        subplot(2,2,2);
        validRmse = rmseHistory(1:frameCount);
        validRmse = validRmse(~isnan(validRmse));
        if ~isempty(validRmse)
            set(hRmse, 'XData', 1:length(validRmse), 'YData', validRmse);
        end
        
        % Speed plot
        subplot(2,2,4);
        validSpeeds = speeds(1:frameCount);
        validSpeeds = validSpeeds(~isnan(validSpeeds));
        if ~isempty(validSpeeds)
            set(hSpeed, 'XData', 1:length(validSpeeds), 'YData', validSpeeds);
        end
        
        drawnow limitrate;
    end
    
    % Status print every 100 frames (MATLAB-safe)
    if mod(frameCount, 100) == 0
        speedKmh = speeds(frameCount) * params.frameRate * 3.6;
        if guidanceApplied
            guidStatus = 'ON';
        else
            guidStatus = 'OFF';
        end
        
        if highRMSEcount > 0
            rmseStatus = 'High RMSE';
        else
            rmseStatus = 'OK';
        end
        
        fprintf('%6d | %6.3f | %6.1f | %8s | %s\n', ...
            frameCount, rmse, speedKmh, guidStatus, rmseStatus);
    end
end

%% 6. SAVE RESULTS
fprintf('\nProcessing complete! Processed %d frames in %.1f seconds\n', ...
    frameCount, toc(startTime));
writematrix(trajectory, fullfile(resultsDir,'trajectory.csv'));
writematrix(rmseHistory(~isnan(rmseHistory))', fullfile(resultsDir,'rmse.csv'));
speedKmh = speeds * params.frameRate * 3.6;
writematrix([speeds', speedKmh'], fullfile(resultsDir,'speed.csv'));
fprintf('Results saved to: %s\n', resultsDir);
