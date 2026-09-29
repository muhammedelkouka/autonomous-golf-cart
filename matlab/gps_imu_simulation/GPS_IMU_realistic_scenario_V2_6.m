% GIU Campus - Professional A* Path Planner with GPS/IMU EKF Fusion
% Master Thesis: Robotics and Automation - FINAL CORRECTED VERSION
% Enhanced with Realistic Urban Canyon Scenario

%clear; clc; close all;

%% ----------------- USER CONFIG -----------------
kmzFile = 'GIU map test 3.kmz';
resampleSpacing_m = 0.5;
pathHalfWidth_m   = 1.5;
edgeWidth_m       = 0.5;
gridResolution_m  = 0.25;
maxVisualPoints   = 15000;
exportVideo       = false;
videoFileName     = 'GIU_EKF_Fusion_Scenario_Test.mp4';

% Vehicle and visualization parameters
vehicleLength_m = 1.2;
vehicleWidth_m  = 0.8;
robotRadius_m   = max(vehicleLength_m, vehicleWidth_m) / 2;
zoomWindow_m    = 20;

% Planner parameters
alphaRoadBias   = 6.0;
allowDiag       = true;

% Smoothing parameters
splineSmoothingDist_m = 0.5;

% --- TARGET SPEED CONFIGURATION ---
targetSpeed_mps = 1.5; % Set speed between 1-2 m/s, using 1.5 m/s

%% ----------------- SCENARIO CONFIG -----------------
% Select the EKF Fusion simulation scenario by uncommenting one line:
% 1: Normal GPS/IMU Fusion (Baseline)
% 2: Weak GPS at Start (High initial position uncertainty
% 3: Full-Path GPS Dropout (Intermittent signal loss)
% 4: IMU-Only Dead Reckoning (No GPS updates or with GPS of update rate 1 HZ)
% 5: Realistic Urban Canyon (Initial GPS denial + random dropouts)
scenario = 2; % Testing new realistic urban canyon scenario

switch scenario
    case 1 % Normal GPS/IMU Fusion (Baseline)
        scenarioName = 'Baseline: Normal GPS/IMU Fusion';
        GPS.positionNoiseStd_m    = 1.0; % Base noise is 1.0m
        GPS.dropoutProbability    = 0.05;
        GPS.weakSignalDistance_m  = 0;
        GPS.weakSignalNoiseFactor = 1.0;
        GPS.maxDropoutDuration_s  = 2.0;
        GPS.updateRate_Hz         = 5;
        
    case 2 % Weak GPS at Start (First 30m)
        scenarioName = 'Scenario 2: Weak GPS at Start (First 30m)';
        GPS.positionNoiseStd_m    = 2.0; % Base noise is 2.0m
        GPS.dropoutProbability    = 0.05;
        GPS.weakSignalDistance_m  = 30.0; % High noise for first 30m
        GPS.weakSignalNoiseFactor = 5.0; % 5x more noise (10m std dev)
        GPS.maxDropoutDuration_s  = 3.0;
        GPS.updateRate_Hz         = 5;

    case 3 % Full-Path GPS Dropout (Intermittent signal loss)
        scenarioName = 'Scenario 3: Full-Path GPS Dropout (High Rate)';
        GPS.positionNoiseStd_m    = 2.0;
        GPS.dropoutProbability    = 0.25;
        GPS.weakSignalDistance_m  = 0;
        GPS.weakSignalNoiseFactor = 1.0;
        GPS.maxDropoutDuration_s  = 5.0;
        GPS.updateRate_Hz         = 5;
        
    case 4 % IMU-Only Dead Reckoning (No GPS updates)
        scenarioName = 'Scenario 4: IMU-Only or with GPS of update rate 1 HZ (Pixel 6a)';
        GPS.positionNoiseStd_m    = 2.0;
        GPS.dropoutProbability    = 0.15; % 15% dropout rate
        GPS.weakSignalDistance_m  = 0;
        GPS.weakSignalNoiseFactor = 1.0;
        GPS.maxDropoutDuration_s  = 8.0; % Longer dropouts
        GPS.updateRate_Hz         = 1;   % Reduced update rate
        %  GPS.positionNoiseStd_m    = 2.0;
        % GPS.dropoutProbability    = 1.0;
        % GPS.weakSignalDistance_m  = 0;
        % GPS.weakSignalNoiseFactor = 1.0;
        % GPS.maxDropoutDuration_s  = Inf;
        % GPS.updateRate_Hz         = 2;  % Rate doesn't matter (100% dropout)
    case 5 % Realistic Urban Canyon with Initial GPS Denial
        scenarioName = 'Scenario 5: Realistic Urban Canyon (Initial Denial + Random Dropouts)';
        GPS.positionNoiseStd_m    = 2.0;    % Realistic 10m urban GPS error
        GPS.dropoutProbability    = 0.15;    % 15% random dropout probability
        GPS.weakSignalDistance_m  = 30.0;    % No GPS for first 30m (corridor/building exit)
        GPS.weakSignalNoiseFactor = 100.0;   % Effectively no GPS in first 30m
        GPS.maxDropoutDuration_s  = 10.0;    % Longer dropouts possible
        GPS.updateRate_Hz         = 1;       % Realistic 1Hz consumer GPS
        
    otherwise
        error('Invalid scenario selected.');
end
fprintf('--- GIU EKF Sensor Fusion Analysis ---\n');
fprintf('Scenario Selected: %s\n', scenarioName);

% ----------------- GPS & IMU BASE CONFIG -----------------
GPS.simulateDelay         = true;
GPS.delay_s               = 0.2;

% Enhanced IMU Configuration based on scenario requirements
if scenario == 5
    % Enhanced IMU with more realistic constraints for urban canyon
    IMU.updateRate_Hz = 20;              % Standard consumer IMU rate
    IMU.orientationNoise_deg = 2.5;      % Slightly worse heading noise
    IMU.accelNoise_mps2 = 0.2;           % More realistic accelerometer noise
    IMU.gyroNoise_dps = 0.4;             % Increased gyro noise
    IMU.magNoise_uT = 3.0;               % Urban magnetic interference
    IMU.gyroBias_dps = 0.2;              % Slightly higher bias
    IMU.velocityNoise_mps = 0.25;        % More velocity uncertainty
else
    % Standard IMU Configuration based on real Pixel 6a data
    IMU.updateRate_Hz = 20;              % Reduced from 50Hz to match real data
    IMU.orientationNoise_deg = 2.0;
    IMU.accelNoise_mps2 = 0.15;          % Increased noise for real sensor
    IMU.gyroNoise_dps = 0.3;             % Increased gyro noise
    IMU.magNoise_uT = 2.0;               % Magnetometer noise
    IMU.gyroBias_dps = 0.15;             % Slightly increased bias
    IMU.velocityNoise_mps = 0.2;         % More realistic velocity noise
end

% ----------------- EKF SENSOR FUSION CONFIG -----------------
EKF.updateRate_Hz = 50;

% Adjust EKF parameters based on scenario
if scenario == 4
    % IMU-Only: Higher uncertainty
    EKF.initialP = diag([50^2, 50^2, 5^2, (30*pi/180)^2, (0.1*pi/180)^2]);
    EKF.Q_factor = [0.5, 0.5, 0.5, 0.1, 0.0001]; % Higher process noise
elseif scenario == 5
    % Urban Canyon: High initial confidence but challenging conditions
    EKF.initialP = diag([1^2, 1^2, 2^2, (5*pi/180)^2, (0.05*pi/180)^2]); % High initial confidence
    EKF.Q_factor = [0.3, 0.3, 0.4, 0.15, 0.0002]; % Higher process noise for challenging conditions
else
    % Normal scenarios
    EKF.initialP = diag([25^2, 25^2, 1^2, (10*pi/180)^2, (0.05*pi/180)^2]);
    EKF.Q_factor = [0.1, 0.1, 0.2, 0.05, 0.00001];
end

EKF.Q = diag(EKF.Q_factor) * (1/EKF.updateRate_Hz);

% Measurement noise (R) - Initializer (required by constructor)
EKF.R = diag([GPS.positionNoiseStd_m^2, GPS.positionNoiseStd_m^2]);
fprintf('EKF R Matrix initialized with Base Noise (%.1fm).\n', GPS.positionNoiseStd_m);

%% ----------------- LOAD KMZ & PATH (Standard Logic) -----------------
fprintf('Loading %s ...\n', kmzFile);
unzipFolder = 'temp_kmz';
if exist(unzipFolder,'dir'), rmdir(unzipFolder,'s'); end
unzip(kmzFile, unzipFolder);
kmlFiles = dir(fullfile(unzipFolder, '*.kml'));
if isempty(kmlFiles), error('No KML inside KMZ!'); end
kmlFile = fullfile(kmlFiles(1).folder, kmlFiles(1).name);
S = kml2struct(kmlFile);
pathIdx = [];
for k=1:numel(S)
    name = lower(S(k).Name);
    if contains(name, 'path') || contains(name,'road') || contains(name,'expected') || contains(name,'route')
        pathIdx(end+1) = k;
    end
end
if isempty(pathIdx), error('No path features found in KML.'); end
allLats = vertcat(S(:).Lat); allLons = vertcat(S(:).Lon);
lat0 = mean(allLats); lon0 = mean(allLons);
deg2rad_func = @(x) x*pi/180;
R_earth = 6371000;
latlon2meters = @(lat,lon) deal( (deg2rad_func(lon-lon0).*cos(deg2rad_func(lat0))*R_earth), ...
                                 (deg2rad_func(lat-lat0)*R_earth) );
pathLengths = zeros(size(pathIdx));
for i=1:numel(pathIdx)
    lat = S(pathIdx(i)).Lat; lon = S(pathIdx(i)).Lon;
    d = haversineDist(lat, lon);
    pathLengths(i) = sum(d);
end
[~, pick] = max(pathLengths); selectedK = pathIdx(pick);
lat = S(selectedK).Lat(:); lon = S(selectedK).Lon(:);
[xs, ys] = latlon2meters(lat, lon);
[pathX_ref, pathY_ref] = resamplePathByDistance(xs, ys, resampleSpacing_m, 'pchip');

%% ----------------- BUILD NAV GRID, OBSTACLES, & A* (Standard Logic) -----------------
pad = 10;
minX = min(pathX_ref)-pad; maxX = max(pathX_ref)+pad;
minY = min(pathY_ref)-pad; maxY = max(pathY_ref)+pad;
xv = (minX:gridResolution_m:maxX); yv = (minY:gridResolution_m:maxY);
[gridX, gridY] = meshgrid(xv, yv);
[nRows, nCols] = size(gridX);
segStart = [pathX_ref(1:end-1), pathY_ref(1:end-1)];
segEnd   = [pathX_ref(2:end), pathY_ref(2:end)];
Dmin = distPointToSegments(gridX, gridY, segStart, segEnd);
navGrid = zeros(size(Dmin), 'uint8');
navGrid(Dmin <= pathHalfWidth_m) = 1;
navGrid((Dmin > pathHalfWidth_m) & (Dmin <= pathHalfWidth_m+edgeWidth_m)) = 2;

% --- OBSTACLE REMOVAL PER REQUEST ---
obstacleCenters = zeros(0, 2);
obstacleR = zeros(0, 1);

costMap = ones(size(navGrid)); costMap(navGrid==0) = Inf; costMap(navGrid==3) = Inf;
Dnorm = Dmin / (max(Dmin(:)) + eps);
inCorr = (navGrid==1 | navGrid==2);
costMap(inCorr) = 1 + alphaRoadBias * Dnorm(inCorr);
startPos = [pathX_ref(1), pathY_ref(1)]; goalPos  = [pathX_ref(end), pathY_ref(end)];
[~, startCol] = min(abs(gridX(1,:) - startPos(1))); [~, startRow] = min(abs(gridY(:,1) - startPos(2)));
[~, goalCol]  = min(abs(gridX(1,:) - goalPos(1))); [~, goalRow]  = min(abs(gridY(:,1) - goalPos(2)));
startGrid = [startRow, startCol]; goalGrid  = [goalRow,  goalCol];
if navGrid(startGrid(1), startGrid(2))==3, [startGrid(1), startGrid(2)] = findNearestFreeCell(navGrid~=3, startGrid(1), startGrid(2)); end
if navGrid(goalGrid(1), goalGrid(2))==3, [goalGrid(1), goalGrid(2)] = findNearestFreeCell(navGrid~=3, goalGrid(1), goalGrid(2)); end

fprintf('Running A*...\n');
[found, pathGrid] = astarGridWithCost(costMap, startGrid, goalGrid, allowDiag, gridResolution_m);

if ~found
    warning('A* failed. Falling back to reference path.');
    pathX = pathX_ref; pathY = pathY_ref;
else
    pathX_coarse = gridX(sub2ind(size(gridX), pathGrid(:,1), pathGrid(:,2)));
    pathY_coarse = gridY(sub2ind(size(gridY), pathGrid(:,1), pathGrid(:,2)));
    [pathX_s, pathY_s] = resamplePathByDistance(pathX_coarse, pathY_coarse, splineSmoothingDist_m, 'pchip');
    window = max(3, round(1.0 / splineSmoothingDist_m));
    pathX = movmean(pathX_s, window); pathY = movmean(pathY_s, window);
end

%% ----------------- CORRECTED DISTANCE CALCULATION -----------------
% Compute headings and velocity for ground truth
headings = zeros(size(pathX));
for i=1:numel(pathX)
    if i == 1
        headings(i) = atan2(pathY(2)-pathY(1), pathX(2)-pathX(1));
    elseif i == numel(pathX)
        headings(i) = atan2(pathY(end)-pathY(end-1), pathX(end)-pathX(end-1));
    else
        headings(i) = atan2(pathY(i+1)-pathY(i), pathX(i+1)-pathX(i));
    end
end

% CORRECTED: Calculate segment distances properly
dx = diff(pathX);
dy = diff(pathY);
segLenPath = sqrt(dx.^2 + dy.^2);  % This calculates actual Euclidean distances
cumDist = [0; cumsum(segLenPath)]; % This gives correct cumulative distance

% Calculate total path distance for verification
totalPathDistance = cumDist(end);
fprintf('Actual total path distance: %.1f meters\n', totalPathDistance);

% Sanity check - typical campus paths should be 200-1500 meters
if totalPathDistance > 3000
    warning('Path distance seems too long (%.1f m). Check coordinate conversion.', totalPathDistance);
end

velocityX_raw = gradient(pathX, cumDist); 
velocityY_raw = gradient(pathY, cumDist);
vectorScale = vehicleLength_m * 0.8;
speed_magnitude = targetSpeed_mps; 
velocityX = velocityX_raw * speed_magnitude;
velocityY = velocityY_raw * speed_magnitude;
speed = hypot(velocityX, velocityY);

nPts = numel(pathX);

% Calculate simulation time step
dt = resampleSpacing_m / targetSpeed_mps;
fprintf('Simulation set to run at %.1f m/s. Base time step (dt) = %.4f s.\n', targetSpeed_mps, dt);

%% ----------------- INITIALIZE GPS, IMU & EKF FUSION -----------------
fprintf('Initializing GPS, IMU & EKF sensor fusion...\n');

% GPS initialization 
gpsState.lastUpdateTime = -Inf;
gpsState.isDropout = false;
gpsState.dropoutStartTime = 0;
gpsState.dropoutDuration = 0;
gpsState.delayedDataBuffer = struct('timestamp', {}, 'truePosition', {}, 'noisyPosition', {}, ...
                                   'positionAccuracy', {}, 'hdop', {}, 'satellites', {}, 'isRTK', {}, 'isValid', {}); 
gpsState.truePosition = [0, 0];

% IMU initialization
imuState.lastUpdateTime = -Inf;
imuState.trueHeading = headings(1);
imuState.noisyHeading = headings(1);
imuState.gyroBias = IMU.gyroBias_dps * pi/180;

% EKF initialization (Using the ExtendedKalmanFilter class)
dt_ekf = 1 / EKF.updateRate_Hz;
x0 = [pathX(1); pathY(1); speed(1); headings(1); 0]; % [x, y, v, theta, omega_bias]
ekf = ExtendedKalmanFilter(x0, EKF.initialP, EKF.Q, EKF.R); 
fprintf('EKF initialized with non-linear kinematic model.\n');

% Storage for all data
fusedTrackX = []; fusedTrackY = [];
trueTrackX = []; trueTrackY = [];
fusedErrorEllipseData = {};

%% ----------------- FIGURE 1: OVERVIEW (Geoaxes) -----------------
fprintf('Creating overview figure...\n');
[gridLat, gridLon] = meters2latlon(gridX, gridY, lat0, lon0);
[pathLat_ref, pathLon_ref] = meters2latlon(pathX_ref, pathY_ref, lat0, lon0);
[pathLat, pathLon] = meters2latlon(pathX, pathY, lat0, lon0);

fig1 = figure('Position',[50 50 1400 900],'Name','GIU Campus - Overview with GPS/IMU Fusion');
gx = geoaxes; geobasemap(gx,'satellite'); hold on;
title(sprintf('GIU Campus - EKF Sensor Fusion: %s', scenarioName),'FontWeight','bold');

geoplot(gx, pathLat_ref, pathLon_ref, 'b--', 'LineWidth', 1.5, 'DisplayName','Reference Centerline');

% Road Edge (Caution) for Figure 1
edgeMask = (navGrid==2);
[er, ec] = find(edgeMask);
if ~isempty(er)
    samp2 = round(linspace(1, numel(er), min(numel(er), maxVisualPoints / 4)));
    edgeLat = gridLat(sub2ind(size(gridLat), er(samp2), ec(samp2)));
    edgeLon = gridLon(sub2ind(size(gridLon), er(samp2), ec(samp2)));
    geoscatter(gx, edgeLat, edgeLon, 4, [1.0 0.8 0.2], 'filled', 'MarkerEdgeColor', 'none', 'DisplayName', 'Road Edge (Caution)');
end

geoplot(gx, pathLat, pathLon, 'm-', 'LineWidth', 2, 'DisplayName','Planned A* Path');

geoscatter(gx, pathLat(1), pathLon(1), 120, 'g', '^', 'filled','MarkerEdgeColor','k','LineWidth',1.5,'DisplayName','Start');
geoscatter(gx, pathLat(end), pathLon(end), 120, 'r', 's', 'filled','MarkerEdgeColor','k','LineWidth',1.5,'DisplayName','Goal');

% Add sensor tracks (only True and Fused) - MODIFIED: Fused track now RED
trueTrack = geoplot(gx, nan, nan, 'c--', 'LineWidth', 1.5, 'DisplayName','True Trajectory');
fusedTrackPlot = geoplot(gx, nan, nan, 'r-', 'LineWidth', 3, 'DisplayName','Fused Estimate');  % Changed to red
fusedPositionMarker = geoscatter(gx, nan, nan, 100, 'r', 'd', 'filled', 'MarkerEdgeColor','k', 'LineWidth',2, 'DisplayName','Fused Position');  % Changed to red

legend('show','Location','northeast');

% info panel (enhanced for EKF) - DISABLED/HIDDEN
% infoPanel = uipanel('Parent',fig1,'Units','normalized','Position',[0.02 0.02 0.24 0.2],'BackgroundColor','w');
% infoText = uicontrol('Parent',infoPanel,'Style','text','Units','normalized','Position',[0.02 0.02 0.96 0.96], ...
%     'String', sprintf('Scenario: %s\nIMU Bias: %.3f °/s\nGPS Noise: %.1f m\nEKF: Initializing...', ...
%     scenarioName, IMU.gyroBias_dps, GPS.positionNoiseStd_m), ...
%     'FontSize',10,'HorizontalAlignment','left','BackgroundColor','w');

%% ----------------- FIGURE 2: ENHANCED ZOOM VIEW -----------------
fprintf('Creating enhanced zoom view...\n');
fig2 = figure('Position',[1100 100 1000 900],'Name','Enhanced Zoom View with Sensor Fusion');
ax2 = axes('Parent',fig2); 
hold(ax2,'on');
axis(ax2,'equal'); grid(ax2,'on'); box(ax2,'on');
title(ax2,'Vehicle Navigation with EKF Sensor Fusion (True vs. Fused)','FontSize',14,'FontWeight','bold');
xlabel(ax2,'East (m)','FontSize',12); ylabel(ax2,'North (m)','FontSize',12);

% Plot Static Features - MODIFIED: Fused color set to RED
colors = struct('allowed', [0.2 0.6 1.0], 'planned', [0.8 0.1 0.8], 'true', [0.1 0.8 0.8], 'fused', [1 0 0]);  % Changed fused to red

% Plot A* Planned Path
plot(ax2, pathX, pathY, '-', 'Color', colors.planned, 'LineWidth', 2, 'DisplayName','Planned Path');

% Road Edge (Caution) for Figure 2
[er, ec] = find(navGrid == 2);
if ~isempty(er)
    samp2 = round(linspace(1, numel(er), min(numel(er), maxVisualPoints / 4)));
    scatter(ax2, gridX(sub2ind(size(gridX), er(samp2), ec(samp2))), ...
                  gridY(sub2ind(size(gridY), er(samp2), ec(samp2))), ...
                  6, [1.0 0.8 0.2], 'filled', 'MarkerEdgeColor', 'none', ...
                  'DisplayName', 'Road Edge (Caution)');
end

% Dynamic Elements
trueTrackZoom = plot(ax2, nan, nan, '--', 'Color', colors.true, 'LineWidth', 1.5, 'DisplayName','True Trajectory');
truePosMarker = plot(ax2, nan, nan, '+', 'Color', colors.true, 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName','True Position');
fusedTrackZoom = plot(ax2, nan, nan, '-', 'Color', colors.fused, 'LineWidth', 3, 'DisplayName','Fused Track');
fusedPosMarker = plot(ax2, nan, nan, 'd', 'Color', colors.fused, 'MarkerSize', 12, 'LineWidth', 3, 'MarkerFaceColor', colors.fused, 'DisplayName','Fused Position');
fusedErrorEllipse = plot(ax2, nan, nan, ':', 'Color', colors.fused, 'LineWidth', 2, 'DisplayName','EKF Uncertainty (2σ)');

% EKF Info Display (on plot)
infoTextZoom = text(ax2, 0.02, 0.98, 'Initializing...', 'Units', 'normalized', ...
                   'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
                   'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
                   'Margin', 5, 'FontWeight', 'bold');

legend(ax2, 'show', 'Location', 'northeast', 'FontSize', 9);

% Setup vehicle visualization
vehicleRect = []; velocityVector = []; imuAxes = [];

% video writer (optional)
if exportVideo, v = VideoWriter(videoFileName,'MPEG-4'); v.FrameRate = 25; open(v); end

%% ----------------- MAIN SIMULATION LOOP WITH EKF FUSION -----------------
simulationTime = 0;
ekfUpdatePeriod = 1 / EKF.updateRate_Hz;
lastEKFUpdateTime = -inf;

fprintf('Starting sensor fusion simulation (dt=%.4fs)...\n', dt);

for i=1:nPts
    rx = pathX(i); ry = pathY(i); % Ground truth position
    simulationTime = simulationTime + dt;
    currentPathDistance = cumDist(i);
    
    % --- Determine Current GPS Noise Level (for Scenario 2 & 5) ---
    currentNoiseStd = GPS.positionNoiseStd_m;
    if (scenario == 2 || scenario == 5) && currentPathDistance <= GPS.weakSignalDistance_m
        currentNoiseStd = GPS.positionNoiseStd_m * GPS.weakSignalNoiseFactor;
    end
    
    % --- Update GPS Simulation ---
    truePosition = [rx, ry];
    [gpsData, gpsValid, gpsState] = updateGPSSimulator_Enhanced(gpsState, GPS, truePosition, simulationTime, currentNoiseStd, currentPathDistance);
    
    % --- Update IMU Simulation (Enhanced for Pixel 6a) ---
    [imuState, imuData] = updateIMUSimulator_Enhanced(imuState, IMU, headings(i), speed(i), simulationTime);
    
    % --- EKF Prediction/Update Loop ---
    if simulationTime - lastEKFUpdateTime >= ekfUpdatePeriod - 1e-6
        
        % 1. EKF Prediction
        u_predict = [imuData.noisyVelocity; imuData.measuredOmega];
        ekf.predict(u_predict, dt_ekf);
        
        % 2. EKF Update (only if a valid, delayed GPS measurement is ready)
        [gpsMeasurement, ~] = getLatestGPSData(gpsState, GPS, simulationTime);
        
        if ~isempty(gpsMeasurement) && gpsMeasurement.isValid && all(isfinite(gpsMeasurement.noisyPosition))
            
            % EKF Fix 3: Dynamic R matrix is set using the accuracy of the *received* GPS fix
            acc_sq = gpsMeasurement.positionAccuracy^2; 
            ekf.R = diag([acc_sq, acc_sq]);            

            z = gpsMeasurement.noisyPosition(:); 
            ekf.update(z);
            
            isGpsUpdated = true;
            gpsHdop = gpsMeasurement.hdop;
        else
            isGpsUpdated = false;
            gpsHdop = nan;
        end
        
        lastEKFUpdateTime = lastEKFUpdateTime + dt_ekf; 
    else
        isGpsUpdated = false;
        gpsHdop = nan;
    end
    
    % --- Logging for Visualization ---
    fusedPosition = ekf.x(1:2);
    fusedHeading = ekf.x(4);
    fusedCovariance = ekf.P;
    
    trueTrackX(end+1) = truePosition(1);
    trueTrackY(end+1) = truePosition(2);
    fusedTrackX(end+1) = fusedPosition(1);
    fusedTrackY(end+1) = fusedPosition(2);
    fusedErrorEllipseData{end+1} = fusedCovariance;
    
    % --- Visualization Update ---
    
    % Update Figure 1 (Geo Map)
    [trueLat, trueLon] = meters2latlon(trueTrackX, trueTrackY, lat0, lon0);
    [fusedLat, fusedLon] = meters2latlon(fusedTrackX, fusedTrackY, lat0, lon0);
    
    set(trueTrack, 'LatitudeData', trueLat, 'LongitudeData', trueLon);
    set(fusedTrackPlot, 'LatitudeData', fusedLat, 'LongitudeData', fusedLon);
    set(fusedPositionMarker, 'LatitudeData', fusedLat(end), 'LongitudeData', fusedLon(end));
    
    % Update Figure 2 (Zoom View)
    displayPosition = fusedPosition';
    displayHeading = fusedHeading;
    
    % Vehicle and Visualization Setup
    if isempty(vehicleRect) || ~isvalid(vehicleRect)
        vehicleRect = createVehicleRectangle(displayPosition(1), displayPosition(2), displayHeading, vehicleLength_m, vehicleWidth_m, ax2);
        velocityVector = quiver(ax2, displayPosition(1), displayPosition(2), ...
            real(velocityX(i)*vectorScale/targetSpeed_mps), real(velocityY(i)*vectorScale/targetSpeed_mps), 0, ...
            'r', 'LineWidth', 3, 'MaxHeadSize', 0.8, 'DisplayName','Velocity');
        imuAxes = createIMUAxes(displayPosition(1), displayPosition(2), displayHeading, ax2);
    else
        updateVehicleRectangle(vehicleRect, displayPosition(1), displayPosition(2), displayHeading, vehicleLength_m, vehicleWidth_m);
        set(velocityVector, 'XData', displayPosition(1), 'YData', displayPosition(2), ...
            'UData', real(velocityX(i)*vectorScale/targetSpeed_mps), 'VData', real(velocityY(i)*vectorScale/targetSpeed_mps));
        updateIMUAxes(imuAxes, displayPosition(1), displayPosition(2), displayHeading);
    end

    set(trueTrackZoom, 'XData', trueTrackX, 'YData', trueTrackY);
    set(fusedTrackZoom, 'XData', fusedTrackX, 'YData', fusedTrackY);
    set(truePosMarker, 'XData', truePosition(1), 'YData', truePosition(2));
    set(fusedPosMarker, 'XData', fusedPosition(1), 'YData', fusedPosition(2));
    
    % Update fused error ellipse (2-sigma covariance)
    P_pos = fusedCovariance(1:2,1:2);
    [V,D] = eig(P_pos);
    sigma = 2*sqrt(diag(D)); % 2-sigma ellipse
    theta_e = linspace(0,2*pi,40);
    ellipse = [cos(theta_e)'*sigma(1), sin(theta_e)'*sigma(2)] * V';
    set(fusedErrorEllipse, 'XData', fusedPosition(1) + ellipse(:,1), ...
                          'YData', fusedPosition(2) + ellipse(:,2));

    % Update Info Panel
    positionError = norm(truePosition' - fusedPosition);
    covarianceTrace = trace(fusedCovariance(1:2,1:2));
    
    if isGpsUpdated
        gpsStatus = 'GPS: UPDATE SUCCESS';
        gpsColor = [0 0.8 0];
    else
        gpsStatus = 'GPS: PREDICT (DR)';
        gpsColor = [0.8 0 0];
    end
    
    infoStrZoom = sprintf(['Time: %.1fs | Dist: %.1fm / %.1fm\n' ...
                          '%s (HDOP: %.1f)\n' ...
                          'Error: %.2fm | Cov: %.2fm² (X:%.2f, Y:%.2f, H:%.1f°)\n' ...
                          'EKF Head: %.1f° | Bias: %.3f °/s'], ...
                          simulationTime, currentPathDistance, totalPathDistance, ...
                          gpsStatus, gpsHdop, positionError, covarianceTrace, ...
                          sqrt(fusedCovariance(1,1)), sqrt(fusedCovariance(2,2)), sqrt(fusedCovariance(4,4))*180/pi, ...
                          fusedHeading*180/pi, ekf.x(5)*180/pi);
    
    set(infoTextZoom, 'String', infoStrZoom, 'Color', gpsColor);
    
    % Info panel update - DISABLED/HIDDEN
    % infoStrMain = sprintf(['Scenario: %s\nIMU Bias: %.3f °/s\nGPS Noise: %.1f m (Curr: %.1f m)\n\n' ...
    %                       '%s\nEKF Pos Error: %.2f m\nEKF Bias Est: %.3f °/s\n\n' ...
    %                       'Target Speed: %.1f m/s'], ...
    %                       scenarioName, IMU.gyroBias_dps, GPS.positionNoiseStd_m, currentNoiseStd, ...
    %                       gpsStatus, positionError, ekf.x(5)*180/pi, ...
    %                       targetSpeed_mps);
    % set(infoText, 'String', infoStrMain);

    % Center zoom window on the fused estimate
    xlim(ax2, [displayPosition(1)-zoomWindow_m/2, displayPosition(1)+zoomWindow_m/2]);
    ylim(ax2, [displayPosition(2)-zoomWindow_m/2, displayPosition(2)+zoomWindow_m/2]);
    
    drawnow limitrate;
    if exportVideo, writeVideo(v,getframe(fig2)); end
    pause(0.01);
end

if exportVideo, close(v); end
fprintf('Simulation complete.\n');

%% ----------------- PERFORMANCE ANALYSIS -----------------
fprintf('\n--- Sensor Fusion Performance Analysis (Scenario %d) ---\n', scenario);

if isempty(fusedTrackX)
    fprintf('No data available for performance analysis.\n');
else
    % Align and calculate errors
    minLength = min(length(fusedTrackX), length(pathX));
    truePositions = [pathX(1:minLength), pathY(1:minLength)];
    fusedPositions = [fusedTrackX(1:minLength)', fusedTrackY(1:minLength)'];
    
    positionDiffs = fusedPositions - truePositions;
    fusedErrors = sqrt(sum(positionDiffs.^2, 2));
    
    fprintf('Position RMSE: %.3f m\n', sqrt(mean(fusedErrors.^2)));
    fprintf('Mean Position Error: %.3f m\n', mean(fusedErrors));
    fprintf('Max Position Error: %.3f m\n', max(fusedErrors));
    
    % Special analysis for IMU-only scenario
    if scenario == 4
        fprintf('\n--- IMU-Only Dead Reckoning Analysis ---\n');
        finalPositionError = norm(truePositions(end,:) - fusedPositions(end,:));
        fprintf('Final position error: %.2f m\n', finalPositionError);
        fprintf('Total distance traveled: %.1f m\n', totalPathDistance);
        fprintf('Average drift rate: %.3f m/m\n', finalPositionError/totalPathDistance);
        fprintf('Note: This demonstrates pure inertial navigation drift characteristics.\n');
    end
    
    % Enhanced analysis for Scenario 5
    if scenario == 5
        fprintf('\n--- Urban Canyon Challenge Analysis ---\n');
        fprintf('Initial GPS-denied distance: %.1f m\n', GPS.weakSignalDistance_m);
        fprintf('Urban GPS error: %.1f m\n', GPS.positionNoiseStd_m);
        fprintf('GPS dropout probability: %.1f%%\n', GPS.dropoutProbability*100);
        
        % Calculate error in initial GPS-denied phase
        initialPhaseEnd = find(cumDist >= GPS.weakSignalDistance_m, 1);
        if ~isempty(initialPhaseEnd) && initialPhaseEnd <= length(fusedTrackX)
            initialError = norm([fusedTrackX(initialPhaseEnd) - pathX(initialPhaseEnd), ...
                                fusedTrackY(initialPhaseEnd) - pathY(initialPhaseEnd)]);
            fprintf('Error after initial %d m GPS denial: %.2f m\n', GPS.weakSignalDistance_m, initialError);
            fprintf('Initial phase drift rate: %.3f m/m\n', initialError/GPS.weakSignalDistance_m);
        end
        
        finalPositionError = norm(truePositions(end,:) - fusedPositions(end,:));
        fprintf('Final position error: %.2f m\n', finalPositionError);
        fprintf('Total distance traveled: %.1f m\n', totalPathDistance);
        fprintf('Overall drift rate: %.3f m/m (%.1f%%)\n', finalPositionError/totalPathDistance, (finalPositionError/totalPathDistance)*100);
        fprintf('Note: This demonstrates urban navigation with GPS challenges.\n');
    end
end

if exist(unzipFolder,'dir'), rmdir(unzipFolder,'s'); end
fprintf('\n=== EKF FUSION SCENARIO COMPLETED SUCCESSFULLY ===\n');

%% ----------------- ENHANCED GPS SIMULATION FUNCTION -----------------
function [gpsData, isValid, gpsState] = updateGPSSimulator_Enhanced(gpsState, config, truePosition, currentTime, positionNoiseStd, currentPathDistance)
    gpsState.truePosition = truePosition;
    
    % --- NEW: Check for initial GPS denial (first 30m) ---
    if currentPathDistance <= config.weakSignalDistance_m
        % Complete GPS denial in initial corridor
        gpsData = struct('timestamp', currentTime, 'truePosition', truePosition, 'noisyPosition', [NaN NaN], ...
                         'positionAccuracy', positionNoiseStd * config.weakSignalNoiseFactor, ...
                         'hdop', NaN, 'satellites', 0, 'isRTK', false, 'isValid', false);
        isValid = false;
        
        % Still update dropout state for when we exit the corridor
        gpsState.lastUpdateTime = currentTime;
        gpsState = updateDropoutState(gpsState, config, currentTime);
        return;
    end
    
    % --- Normal GPS processing (after initial 30m) ---
    if ~isfinite(gpsState.lastUpdateTime), timeSinceLast = Inf;
    else, timeSinceLast = currentTime - gpsState.lastUpdateTime; end
    minDt = 1 / config.updateRate_Hz;

    if timeSinceLast < minDt
        [gpsData, gpsState] = getLatestGPSData(gpsState, config, currentTime);
        isValid = ~gpsState.isDropout && ~isempty(gpsData) && gpsData.isValid && ~any(isnan(gpsData.noisyPosition));
        return;
    end

    gpsState.lastUpdateTime = currentTime;
    gpsState = updateDropoutState(gpsState, config, currentTime);

    if gpsState.isDropout
        gpsData = struct('timestamp', currentTime, 'truePosition', truePosition, 'noisyPosition', [NaN NaN], ...
                         'positionAccuracy', positionNoiseStd, 'hdop', NaN, 'satellites', 0, 'isRTK', false, 'isValid', false);
        isValid = false;
        return;
    end

    % --- NEW: Outlier rejection for realistic GPS ---
    baseNoise = positionNoiseStd * randn(1,2);
    
    % Add occasional large outliers (5% chance of >20m error)
    if rand() < 0.05
        outlierFactor = 2 + 3*rand(); % 2x to 5x normal error
        noisyPos = truePosition + baseNoise * outlierFactor;
        positionAccuracy = positionNoiseStd * outlierFactor;
    else
        noisyPos = truePosition + baseNoise;
        positionAccuracy = positionNoiseStd;
    end

    gpsData = struct('timestamp', currentTime, ...
                     'truePosition', truePosition, ...
                     'noisyPosition', noisyPos, ...
                     'positionAccuracy', positionAccuracy, ...
                     'hdop', 1.5 + 1.0*rand(),... % Worse HDOP in urban areas
                     'satellites', randi([4,8]),...  % Fewer satellites in urban canyons
                     'isRTK', false,... % No RTK in challenging conditions
                     'isValid', true);

    if config.simulateDelay
        gpsState.delayedDataBuffer(end+1) = gpsData;
    else
        gpsState.delayedDataBuffer = gpsData;
    end

    isValid = true;
end

%% ----------------- ENHANCED IMU SIMULATION FUNCTION -----------------
function [imuState, imuData] = updateIMUSimulator_Enhanced(imuState, config, trueHeading, trueSpeed, currentTime)
    % Enhanced version with magnetometer and more realistic noise modeling
    
    persistent lastTrueHeading lastTime biasAccumulation;
    if isempty(lastTrueHeading) || isempty(lastTime) || isempty(biasAccumulation)
        lastTrueHeading = trueHeading;
        lastTime = currentTime;
        biasAccumulation = 0;
    end
    
    % Time calculation
    dt = currentTime - lastTime;
    if dt < 1e-6
        dt = 1/config.updateRate_Hz;
        trueOmega = 0;
    else
        trueOmega = wrapToPi(trueHeading - lastTrueHeading) / dt;
    end
    
    % Update persistent variables
    lastTrueHeading = trueHeading;
    lastTime = currentTime;
    
    % Bias accumulation (more realistic)
    biasAccumulation = biasAccumulation + imuState.gyroBias * dt;
    
    % Sensor measurements with realistic noise
    gyroNoise = (config.gyroNoise_dps * pi/180) / sqrt(dt) * randn();
    measuredOmega = trueOmega + imuState.gyroBias + gyroNoise;
    
    % Accelerometer simulation (lateral and longitudinal components)
    accelNoise = config.accelNoise_mps2 * randn(3,1);
    trueAccel = [trueSpeed * cos(trueHeading); trueSpeed * sin(trueHeading); 9.81]; % Include gravity
    measuredAccel = trueAccel + accelNoise;
    
    % Magnetometer simulation (simplified Earth magnetic field)
    magNoise = config.magNoise_uT * randn(3,1);
    % Assume magnetic North at 0° for simplicity
    trueMag = [cos(trueHeading); sin(trueHeading); 0] * 50; % 50uT field strength
    measuredMag = trueMag + magNoise;
    
    % Heading from magnetometer (with noise)
    magHeading = atan2(measuredMag(2), measuredMag(1));
    magHeadingNoise = config.magNoise_uT/50 * randn(); % Scale noise appropriately
    measuredMagHeading = magHeading + magHeadingNoise;
    
    % Velocity and heading with noise
    noisyVelocity = trueSpeed + config.velocityNoise_mps * randn();
    headingNoise = config.orientationNoise_deg * pi/180 * randn();
    noisyHeading = trueHeading + headingNoise + biasAccumulation;
    noisyHeading = atan2(sin(noisyHeading), cos(noisyHeading));
    
    imuData = struct(...
        'timestamp', currentTime, ...
        'trueHeading', trueHeading, ...
        'measuredOmega', measuredOmega, ...
        'measuredAccel', measuredAccel, ...
        'measuredMag', measuredMag, ...
        'measuredMagHeading', measuredMagHeading, ...
        'noisyVelocity', noisyVelocity, ...
        'noisyHeading', noisyHeading, ...
        'isValid', true ...
    );
end

%% ----------------- GPS HELPER FUNCTIONS -----------------
function [gpsData, gpsState] = getLatestGPSData(gpsState, config, currentTime)
    gpsData = struct('timestamp', currentTime, 'truePosition', gpsState.truePosition, ...
                     'noisyPosition', [NaN NaN], 'positionAccuracy', config.positionNoiseStd_m, ...
                     'hdop', NaN, 'satellites', 0, 'isRTK', false, 'isValid', false);

    if config.simulateDelay
        if isempty(gpsState.delayedDataBuffer)
            return;
        end
        cutoff = currentTime - config.delay_s;
        timestamps = [gpsState.delayedDataBuffer.timestamp];
        validIdx = find(timestamps <= cutoff);
        if isempty(validIdx)
            return;
        else
            gpsData = gpsState.delayedDataBuffer(validIdx(end));
        end
    else 
        if ~isempty(gpsState.delayedDataBuffer)
            gpsData = gpsState.delayedDataBuffer(end);
        end
    end
end

function gpsState = updateDropoutState(gpsState, config, currentTime)
    if gpsState.isDropout
        if (currentTime - gpsState.dropoutStartTime) >= gpsState.dropoutDuration
            gpsState.isDropout = false;
        end
    else
        if rand() < config.dropoutProbability
            gpsState.isDropout = true;
            gpsState.dropoutStartTime = currentTime;
            gpsState.dropoutDuration = config.maxDropoutDuration_s * rand();
        end
    end
end

%% ----------------- EXTENDED KALMAN FILTER CLASS -----------------

%% ----------------- HELPER FUNCTIONS (Visualization/A*) -----------------
function [found, pathIdxs] = astarGridWithCost(costMap, startIdx, goalIdx, allowDiag, gridRes)
    found = false; [nR,nC] = size(costMap); 
    startLin = sub2ind([nR,nC], startIdx(1), startIdx(2)); 
    goalLin  = sub2ind([nR,nC], goalIdx(1), goalIdx(2)); 
    gScore = inf(nR*nC,1); fScore = inf(nR*nC,1); cameFrom = zeros(nR*nC,1); openSet = false(nR*nC,1);
    
    if allowDiag
        nbrs = [ -1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1]; 
        moveCostBase = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)]; 
    else
        nbrs = [ -1 0; 0 -1; 0 1; 1 0 ]; 
        moveCostBase = [1, 1, 1, 1]; 
    end
    
    costVec = costMap(:); 
    gScore(startLin) = 0;
    heuristicFun = @(linIdx) heuristic_lin(linIdx, goalLin, nC, nR, gridRes);
    fScore(startLin) = heuristicFun(startLin); 
    openSet(startLin) = true; 
    maxIter = numel(gScore) * 10; iter = 0;
    
    while any(openSet)
        iter = iter + 1; 
        if iter > maxIter, break; end
        openIdxs = find(openSet); 
        [~, minPos] = min(fScore(openIdxs)); 
        current = openIdxs(minPos);
        
        if current == goalLin, found = true; break; end
        openSet(current) = false; 
        [cr, cc] = ind2sub([nR,nC], current);
        
        for ni = 1:size(nbrs,1)
            nr = cr + nbrs(ni,1); nc = cc + nbrs(ni,2); 
            if nr < 1 || nr > nR || nc < 1 || nc > nC, continue; end
            neighLin = sub2ind([nR,nC], nr, nc); 
            if isinf(costVec(neighLin)), continue; end
            
            moveCost = moveCostBase(ni) * (0.5*(costVec(current)+costVec(neighLin)));
            tentative_g = gScore(current) + moveCost;
            
            if tentative_g < gScore(neighLin)
                cameFrom(neighLin) = current; 
                gScore(neighLin) = tentative_g;
                fScore(neighLin) = tentative_g + heuristicFun(neighLin); 
                openSet(neighLin) = true;
            end
        end
    end
    
    if ~found, pathIdxs = []; return; end
    
    cur = goalLin; pathRev = cur;
    while cur ~= startLin
        cur = cameFrom(cur); 
        if cur == 0, found = false; pathIdxs = []; return; end
        pathRev(end+1,1) = cur;
    end
    pathRev = flip(pathRev); 
    [rr,cc] = ind2sub([nR,nC], pathRev); 
    pathIdxs = [rr, cc];
end

function h = heuristic_lin(linIdx, goalLin, nC, nR, gridRes)
    [r,c] = ind2sub([nR,nC], linIdx); 
    [rg,cg] = ind2sub([nR,nC], goalLin);
    dx = (c - cg) * (gridRes); 
    dy = (r - rg) * (gridRes); 
    h = sqrt(dx.^2 + dy.^2);
end

function [rOut,cOut] = findNearestFreeCell(freeMask, rStart, cStart)
    [nR, nC] = size(freeMask); 
    if freeMask(rStart,cStart), rOut=rStart; cOut=cStart; return; end
    
    maxRadius = max(nR,nC);
    for R=1:maxRadius
        rmin = max(1, rStart-R); rmax = min(nR, rStart+R);
        cmin = max(1, cStart-R); cmax = min(nC, cStart+R);
        for rr=rmin:rmax
            for cc=cmin:cmax
                if freeMask(rr,cc), rOut=rr; cOut=cc; return; end
            end
        end
    end
    rOut = rStart; cOut = cStart;
end

function D=distPointToSegments(gridX,gridY,segStart,segEnd)
    [nR,nC]=size(gridX); ptsX=gridX(:); ptsY=gridY(:); Dvec=inf(numel(ptsX),1); Nseg=size(segStart,1); chunk=20000;
    for s=1:Nseg
        sx=segStart(s,1); sy=segStart(s,2); ex=segEnd(s,1); ey=segEnd(s,2); vx=ex-sx; vy=ey-sy; segLen2=vx*vx+vy*vy;
        if segLen2 == 0
            for st=1:chunk:numel(ptsX), en=min(st+chunk-1,numel(ptsX)); px=ptsX(st:en); py=ptsY(st:en); dseg=sqrt((px-sx).^2+(py-sy).^2); Dvec(st:en)=min(Dvec(st:en),dseg); end
        else
            for st=1:chunk:numel(ptsX), en=min(st+chunk-1,numel(ptsX)); px=ptsX(st:en); py=ptsY(st:en); wx=px-sx; wy=py-sy; t=(wx*vx+wy*vy)./segLen2; tClip=min(max(t,0),1); projx=sx+tClip*vx; projy=sy+tClip*vy; dseg=sqrt((px-projx).^2+(py-projy).^2); Dvec(st:en)=min(Dvec(st:en),dseg); end
        end
    end
    D=reshape(Dvec,nR,nC);
end

function [latOut,lonOut]=meters2latlon(x,y,lat0,lon0)
    R=6371000; deg2rad = @(x) x*pi/180; 
    latOut=(y./R)*180/pi+lat0; 
    lonOut=(x./(R*cos(deg2rad(lat0))))*180/pi+lon0;
end

function rect = createVehicleRectangle(x,y,heading,len,wid,ax)
    corners = calculateVehicleCorners(x,y,heading,len,wid); 
    rect = patch(ax,corners(:,1),corners(:,2),[1 0.5 0],'FaceAlpha',0.8,'EdgeColor','k','LineWidth',2);
end

function updateVehicleRectangle(rect,x,y,heading,len,wid)
    corners = calculateVehicleCorners(x,y,heading,len,wid); 
    set(rect,'XData',corners(:,1),'YData',corners(:,2));
end

function corners = calculateVehicleCorners(x,y,heading,len,wid)
    hL = len/2; hW = wid/2; 
    local = [ hL  hW; hL -hW; -hL -hW; -hL  hW]; 
    R = [cos(heading) -sin(heading); sin(heading) cos(heading)]; 
    corners = local*R' + [x y];
end

function d = haversineDist(lat,lon)
    R=6371000; 
    if numel(lat) < 2, d = 0; return; end; 
    deg2rad = @(x) x*pi/180; 
    lat=deg2rad(lat); lon=deg2rad(lon); 
    dlat=diff(lat); dlon=diff(lon); 
    a=sin(dlat/2).^2+cos(lat(1:end-1)).*cos(lat(2:end)).*sin(dlon/2).^2; 
    c=2*atan2(sqrt(a),sqrt(1-a)); 
    d=R.*c;
end

function [xR,yR] = resamplePathByDistance(x,y,sp,method)
    if nargin<4, method='linear'; end; 
    dx=diff(x); dy=diff(y); seg=sqrt(dx.^2+dy.^2); cum=[0;cumsum(seg)]; total=cum(end); 
    if total==0, xR=x; yR=y; return; end; 
    q=(0:sp:total)'; if q(end)<total, q(end+1)=total; end; 
    xR=interp1(cum,x,q,method); yR=interp1(cum,y,q,method);
end

function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end

function imuAxes = createIMUAxes(x, y, heading, ax)
    axisLength = 0.8; 
    x_end = x + axisLength * cos(heading); y_end = y + axisLength * sin(heading); 
    imuAxes.x = quiver(ax, x, y, x_end-x, y_end-y, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.8, 'DisplayName','IMU X (Forward)');
    x_end = x + axisLength * cos(heading + pi/2); y_end = y + axisLength * sin(heading + pi/2); 
    imuAxes.y = quiver(ax, x, y, x_end-x, y_end-y, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.8, 'DisplayName','IMU Y (Left)');
    imuAxes.z = plot(ax, x, y, 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b', 'DisplayName','IMU Z (Up)');
end

function updateIMUAxes(imuAxes, x, y, heading)
    axisLength = 0.8; 
    x_end = x + axisLength * cos(heading); y_end = y + axisLength * sin(heading); 
    set(imuAxes.x, 'XData', x, 'YData', y, 'UData', x_end-x, 'VData', y_end-y);
    x_end = x + axisLength * cos(heading + pi/2); y_end = y + axisLength * sin(heading + pi/2); 
    set(imuAxes.y, 'XData', x, 'YData', y, 'UData', x_end-x, 'VData', y_end-y);
    set(imuAxes.z, 'XData', x, 'YData', y);
end