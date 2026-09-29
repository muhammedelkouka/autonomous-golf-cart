%% REAL-TIME LiDAR LOCALIZATION: HYBRID SYSTEM
clear; close all; clc;
fprintf('Initializing Hybrid Localization (Consecutive + Map Reference)...\n');

%% 1. CONFIGURATION & DATA I/O
mapFile = 'Master_Structural_Ref_Adaptive.ply';
pcapFile = 'reading7.pcap';
mapCloud = pcread(mapFile);
veloReader = velodyneFileReader(pcapFile, 'VLP16');

% Configuration
cfg.voxelSize = 3.0;       
cfg.gridStep = 0.8;        
cfg.roi = [-40 40 -40 40 -1.5 4.0];
cfg.maxIterations = 60;
cfg.tolerance = [0.01, 0.005];
cfg.minPointsRequired = 150;
cfg.logFile = 'localization_log.csv';
cfg.enableMetrics = true;
cfg.smoothingWindow = 5;
cfg.loopClosureThreshold = 5.0; % meters

% Validate configuration
cfg = validateConfig(cfg);

% Downsample map for display
priorMapSmall = pcdownsample(mapCloud, 'gridAverage', 1.5);
currentTform = eye(4);
prevMoving = []; 
results.trajectory = [];
recentPositions = []; % For loop closure detection

% Initialize metrics
if cfg.enableMetrics
    metrics = struct();
    metrics.frameTimes = [];
    metrics.pointCounts = [];
    metrics.registrationSuccess = [];
    metrics.rmseValues = [];
    metrics.timestamp = [];
end

%% 2. LIVE STREAM SETUP
figLive = figure('Name', 'Hybrid LiDAR Localization', 'Color', [0.1 0.1 0.1], ...
    'Position', [100 100 1200 800]);

% Main axes for point clouds
axStream = axes('Parent', figLive, 'Position', [0.05 0.15 0.7 0.8], 'Color', [0.1 0.1 0.1]);
hold(axStream, 'on');

hMap = pcshow(priorMapSmall, 'MarkerSize', 1, 'Parent', axStream);
hRaw = scatter3(axStream, 0,0,0,10,[1 0.2 0.2], 'DisplayName', 'Raw Scan');
hMatched = scatter3(axStream,0,0,0,15,[0 1 0.2], 'filled', 'DisplayName', 'Localized');
hPath = plot3(axStream,0,0,0,'w','LineWidth',1.5, 'DisplayName', 'Trajectory');

view(axStream, 2); axis(axStream, 'equal'); grid(axStream, 'on');
set(axStream, 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
xlabel(axStream, 'X (m)'); ylabel(axStream, 'Y (m)'); zlabel(axStream, 'Z (m)');
title(axStream, 'Real-time LiDAR Localization', 'Color', 'white');
legend(axStream, 'TextColor', 'w', 'Color', [0.2 0.2 0.2], 'Location', 'northeast');

% Status panel
statusPanel = uipanel('Parent', figLive, 'Position', [0.77 0.70 0.22 0.25], ...
    'BackgroundColor', [0.15 0.15 0.15], 'BorderType', 'line', ...
    'HighlightColor', [0.3 0.3 0.3], 'Title', 'System Status', ...
    'ForegroundColor', 'white', 'FontSize', 10);

hStatus = uicontrol(statusPanel, 'Style', 'text', ...
    'Position', [10 10 180 80], ...
    'String', 'Initializing...', ...
    'BackgroundColor', [0.15 0.15 0.15], ...
    'ForegroundColor', 'white', ...
    'FontSize', 9, ...
    'HorizontalAlignment', 'left');

% Metrics panel
metricsPanel = uipanel('Parent', figLive, 'Position', [0.77 0.40 0.22 0.25], ...
    'BackgroundColor', [0.15 0.15 0.15], 'BorderType', 'line', ...
    'HighlightColor', [0.3 0.3 0.3], 'Title', 'Performance Metrics', ...
    'ForegroundColor', 'white', 'FontSize', 10);

hMetrics = uicontrol(metricsPanel, 'Style', 'text', ...
    'Position', [10 10 180 80], ...
    'String', 'Waiting for data...', ...
    'BackgroundColor', [0.15 0.15 0.15], ...
    'ForegroundColor', 'white', ...
    'FontSize', 9, ...
    'HorizontalAlignment', 'left');

% Control panel
controlPanel = uipanel('Parent', figLive, 'Position', [0.77 0.10 0.22 0.25], ...
    'BackgroundColor', [0.15 0.15 0.15], 'BorderType', 'line', ...
    'HighlightColor', [0.3 0.3 0.3], 'Title', 'Controls', ...
    'ForegroundColor', 'white', 'FontSize', 10);

uicontrol(controlPanel, 'Style', 'pushbutton', ...
    'Position', [10 60 80 30], ...
    'String', 'Pause', ...
    'Callback', @togglePause, ...
    'BackgroundColor', [0.3 0.3 0.6]);

uicontrol(controlPanel, 'Style', 'pushbutton', ...
    'Position', [100 60 80 30], ...
    'String', 'Save Data', ...
    'Callback', @saveData, ...
    'BackgroundColor', [0.3 0.6 0.3]);

% Global variables for controls
global isPaused;
isPaused = false;

%% 3. MAIN PROCESSING LOOP
frameIdx = 0;
totalFrames = veloReader.NumberOfFrames;
fprintf('Total frames to process: %d\n', totalFrames);

while hasFrame(veloReader) && frameIdx < totalFrames
    % Check for pause state
    while isPaused
        pause(0.1);
        drawnow;
    end
    
    frameStart = tic;
    frameIdx = frameIdx + 1;
    
    % A. Capture and Pre-process
    scanRaw = readFrame(veloReader);
    moving = select(scanRaw, findPointsInROI(scanRaw, cfg.roi));
    moving = pcdownsample(moving, 'gridAverage', cfg.gridStep);
    
    % Initialization Guard
    if moving.Count < cfg.minPointsRequired
        fprintf('Frame %d: Too few points (%d). Skipping...\n', frameIdx, moving.Count);
        continue;
    end
    
    if isempty(prevMoving)
        prevMoving = moving;
        results.trajectory(frameIdx,:) = currentTform(4,1:3);
        continue;
    end

    % B. Step 1: Consecutive Matching (Motion Guess)
    guessTform = currentTform; % Default guess is "no movement"
    try
        % Match current to previous
        [relTform, ~, ~] = pcregisterndt(moving, prevMoving, cfg.voxelSize, ...
            'MaxIterations', 30, 'Tolerance', cfg.tolerance);
        guessTform = relTform.T * currentTform;
    catch ME
        fprintf('Frame %d: Local registration failed: %s\n', frameIdx, ME.message);
    end

    % C. Step 2: Map Matching (Global Correction)
    regSuccess = false;
    movingReg = moving; % Default 
    rmse = NaN;
    
    try
        [mapTform, movingReg, rmse] = pcregisterndt(moving, mapCloud, cfg.voxelSize, ...
            'InitialTransform', affine3d(guessTform), ...
            'MaxIterations', cfg.maxIterations, ...
            'Tolerance', cfg.tolerance);
        
        if isValidTransform(mapTform.T)
            regSuccess = true;
            currentTform = mapTform.T; 
            prevMoving = moving; % Update reference for next frame
        else
            currentTform = guessTform;
        end
    catch ME
        fprintf('Frame %d: Map registration failed: %s\n', frameIdx, ME.message);
        currentTform = guessTform;
    end

    % D. Update trajectory with smoothing
    pos = currentTform(4,1:3);
    results.trajectory(frameIdx,:) = pos;
    
    % Smooth trajectory if enough points
    if frameIdx > cfg.smoothingWindow
        results.trajectory = smoothTrajectory(results.trajectory, cfg.smoothingWindow);
    end
    
    % E. Loop closure detection
    recentPositions = [recentPositions; pos];
    if size(recentPositions, 1) > 50
        recentPositions(1, :) = [];
        
        % Check if current position is close to any historical position
        if size(recentPositions, 1) > 10
            distances = sqrt(sum((recentPositions(1:end-10, :) - pos).^2, 2));
            if any(distances < cfg.loopClosureThreshold)
                fprintf('Frame %d: Potential loop closure detected (%.2f m from previous position)\n', ...
                    frameIdx, min(distances));
            end
        end
    end
    
    % F. Update metrics
    frameTime = toc(frameStart);
    if cfg.enableMetrics
        metrics.frameTimes(frameIdx) = frameTime;
        metrics.pointCounts(frameIdx) = moving.Count;
        metrics.registrationSuccess(frameIdx) = regSuccess;
        metrics.rmseValues(frameIdx) = rmse;
        metrics.timestamp(frameIdx) = now;
    end
    
    % G. Update Visualization
    if mod(frameIdx, 2) == 0  % Update every other frame for performance
        % Show localized points in global space
        set(hMatched, 'XData', movingReg.Location(:,1), ...
                     'YData', movingReg.Location(:,2), ...
                     'ZData', movingReg.Location(:,3));
        
        % Update path
        validTraj = results.trajectory(~isnan(results.trajectory(:,1)), :);
        if ~isempty(validTraj)
            set(hPath, 'XData', validTraj(:,1), ...
                       'YData', validTraj(:,2), ...
                       'ZData', validTraj(:,3));
        end
        
        % Update status display
        if regSuccess
            statusStr = sprintf('Frame: %d/%d\nPoints: %d\nRMSE: %.3f m\nFPS: %.1f\nLoop Closure: %s', ...
                frameIdx, totalFrames, moving.Count, rmse, 1/frameTime, ...
                'Monitoring');
        else
            statusStr = sprintf('Frame: %d/%d\nPoints: %d\nStatus: Using Local Guess\nFPS: %.1f', ...
                frameIdx, totalFrames, moving.Count, 1/frameTime);
        end
        set(hStatus, 'String', statusStr);
        
        % Update metrics display
        if cfg.enableMetrics && frameIdx > 1
            avgFPS = mean(1./metrics.frameTimes(1:frameIdx));
            successRate = mean(metrics.registrationSuccess(1:frameIdx)) * 100;
            avgPoints = mean(metrics.pointCounts(1:frameIdx));
            
            metricsStr = sprintf('Avg FPS: %.1f\nSuccess Rate: %.1f%%\nAvg Points: %.0f\nProcessing Time: %.1f ms', ...
                avgFPS, successRate, avgPoints, frameTime*1000);
            set(hMetrics, 'String', metricsStr);
        end
        
        % Dynamic Camera centering
        set(axStream, 'XLim', [pos(1)-30, pos(1)+30], 'YLim', [pos(2)-30, pos(2)+20]);
        drawnow limitrate;
    end
    
    % Periodically save metrics
    if mod(frameIdx, 100) == 0 && cfg.enableMetrics
        saveMetricsToFile(metrics, cfg.logFile);
        fprintf('Frame %d: Metrics saved to %s\n', frameIdx, cfg.logFile);
    end
end

%% 4. POST-PROCESSING AND SAVE RESULTS
fprintf('\nProcessing complete!\n');
fprintf('Total frames processed: %d\n', frameIdx);

% Save final results
save('localization_results.mat', 'results', 'metrics', 'cfg');

% Plot final trajectory
figResults = figure('Name', 'Final Results', 'Position', [200 200 800 600]);
subplot(2,2,1);
plot(results.trajectory(:,1), results.trajectory(:,2), 'b-', 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)');
title('Final Trajectory'); grid on; axis equal;

subplot(2,2,2);
if cfg.enableMetrics
    plot(metrics.frameTimes*1000, 'b-');
    xlabel('Frame'); ylabel('Processing Time (ms)');
    title('Processing Time per Frame'); grid on;
end

subplot(2,2,3);
if cfg.enableMetrics
    plot(metrics.rmseValues, 'r-');
    xlabel('Frame'); ylabel('RMSE (m)');
    title('Registration Error'); grid on;
end

subplot(2,2,4);
if cfg.enableMetrics
    histogram(metrics.pointCounts, 20);
    xlabel('Point Count'); ylabel('Frequency');
    title('Point Distribution per Frame'); grid on;
end

%% 5. HELPER FUNCTIONS
function cfg = validateConfig(cfg)
    % Validate configuration parameters
    requiredFields = {'voxelSize', 'gridStep', 'roi', 'maxIterations', 'tolerance', 'minPointsRequired'};
    for i = 1:length(requiredFields)
        if ~isfield(cfg, requiredFields{i})
            error('Missing required config field: %s', requiredFields{i});
        end
    end
    
    % Validate ranges
    assert(cfg.voxelSize > 0, 'voxelSize must be positive');
    assert(cfg.gridStep > 0, 'gridStep must be positive');
    assert(cfg.maxIterations > 0, 'maxIterations must be positive');
    assert(cfg.minPointsRequired > 0, 'minPointsRequired must be positive');
    assert(length(cfg.roi) == 6, 'ROI must have 6 elements');
end

function valid = isValidTransform(T)
    % Check if transformation matrix is valid
    valid = all(isfinite(T(:))) && abs(det(T(1:3,1:3)) - 1) < 0.1;
end

function smoothedTraj = smoothTrajectory(trajectory, windowSize)
    % Apply moving average smoothing to trajectory
    if size(trajectory, 1) < windowSize
        smoothedTraj = trajectory;
        return;
    end
    
    % Create kernel for moving average
    kernel = ones(windowSize, 1) / windowSize;
    smoothedTraj = zeros(size(trajectory));
    
    % Apply smoothing to each dimension
    for i = 1:3
        if ~all(isnan(trajectory(:, i)))
            validIdx = ~isnan(trajectory(:, i));
            temp = trajectory(:, i);
            temp(~validIdx) = 0; % Replace NaN with 0 for filtering
            
            % Apply convolution
            smoothed = conv(temp, kernel, 'same');
            
            % Adjust edges
            halfWin = floor(windowSize/2);
            for j = 1:halfWin
                smoothed(j) = mean(temp(1:j+halfWin));
                smoothed(end-j+1) = mean(temp(end-j-halfWin+1:end));
            end
            
            % Restore NaN values
            smoothed(~validIdx) = NaN;
            smoothedTraj(:, i) = smoothed;
        else
            smoothedTraj(:, i) = trajectory(:, i);
        end
    end
end

function saveMetricsToFile(metrics, filename)
    % Save metrics to CSV file
    if isempty(metrics.frameTimes)
        return;
    end
    
    % Create table from metrics
    frameNumbers = (1:length(metrics.frameTimes))';
    
    % Ensure all arrays are same length
    minLength = min([length(metrics.frameTimes), length(metrics.pointCounts), ...
                     length(metrics.registrationSuccess), length(metrics.rmseValues)]);
    
    dataTable = table(frameNumbers(1:minLength), ...
                      metrics.frameTimes(1:minLength)', ...
                      metrics.pointCounts(1:minLength)', ...
                      metrics.registrationSuccess(1:minLength)', ...
                      metrics.rmseValues(1:minLength)', ...
                      'VariableNames', {'Frame', 'ProcessingTime', 'PointCount', 'RegistrationSuccess', 'RMSE'});
    
    writetable(dataTable, filename);
end

function togglePause(~, ~)
    % Toggle pause state
    global isPaused;
    isPaused = ~isPaused;
    if isPaused
        fprintf('Processing paused.\n');
    else
        fprintf('Processing resumed.\n');
    end
end

function saveData(~, ~)
    % Save current data
    global results metrics cfg;
    
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = sprintf('localization_data_%s.mat', timestamp);
    
    save(filename, 'results', 'metrics', 'cfg');
    fprintf('Data saved to %s\n', filename);
    
    % Also save trajectory as CSV
    csvFilename = sprintf('trajectory_%s.csv', timestamp);
    writematrix(results.trajectory, csvFilename);
    fprintf('Trajectory saved to %s\n', csvFilename);
end

%% 6. ADVANCED REGISTRATION FUNCTION (Optional - uncomment if needed)
function [T, registeredCloud, success, metrics] = adaptiveRegistration(moving, fixed, initialGuess, config)
    % Try multiple registration strategies
    
    strategies = {'NDT', 'ICP', 'GICP'};
    success = false;
    metrics = struct();
    
    for strategyIdx = 1:length(strategies)
        try
            switch strategies{strategyIdx}
                case 'NDT'
                    [T, registeredCloud, rmse] = pcregisterndt(...
                        moving, fixed, config.voxelSize, ...
                        'InitialTransform', affine3d(initialGuess), ...
                        'MaxIterations', config.maxIterations, ...
                        'Tolerance', config.tolerance);
                    
                case 'ICP'
                    [T, registeredCloud, rmse] = pcregistericp(...
                        moving, fixed, ...
                        'Metric', 'pointToPlane', ...
                        'InitialTransform', affine3d(initialGuess), ...
                        'MaxIterations', config.maxIterations);
                    
                case 'GICP'
                    [T, registeredCloud, rmse] = pcregistergicp(...
                        moving, fixed, ...
                        'InitialTransform', affine3d(initialGuess));
            end
            
            if isValidTransform(T.T)
                success = true;
                metrics.strategy = strategies{strategyIdx};
                metrics.rmse = rmse;
                metrics.iterations = T.InlierIndices;
                break;
            end
        catch ME
            fprintf('Strategy %s failed: %s\n', strategies{strategyIdx}, ME.message);
            continue;
        end
    end
    
    if ~success
        T = affine3d(initialGuess);
        registeredCloud = moving;
        metrics.strategy = 'Fallback';
        metrics.rmse = NaN;
        metrics.iterations = [];
    end
end