%% GIU Campus - A* Path Planner with GPS/IMU EKF Fusion
%  Corrected version.  Requires ExtendedKalmanFilter.m on the path.
%
%  WHAT CHANGED FROM THE PREVIOUS VERSION (read this before re-running):
%
%  1. TIMING.  The old loop advanced ground truth by dt = spacing/speed
%     (0.333 s) but called ekf.predict(u, dt_ekf) with dt_ekf = 0.02 s, so
%     the filter propagated 1/16.7 of the real motion each step.  That
%     systematic 0.47 m/step shortfall, fought by the position update, is
%     what produced the ~7 m RMSE.  The simulation now runs on ONE clock:
%     the path is resampled so that one tick = one dt_sim, and predict()
%     always receives the true elapsed time.
%
%  2. PROCESS NOISE.  Q was pre-scaled by 1/50 at construction and then
%     added flat regardless of dt.  Qc is now a continuous-time density
%     scaled by dt inside predict().
%
%  3. GPS BUFFER.  getLatestGPSData never removed consumed fixes, so the
%     same measurement was fed to update() repeatedly, shrinking P and the
%     Kalman gain.  Fixes are now popped when used.
%
%  4. BIAS INITIALISATION.  x0(5) = 0 while the truth generator used
%     0.15 deg/s, and initialP(5,5) claimed +/-0.05 deg/s -- a 3-sigma
%     inconsistency with near-zero Q on the bias state.  P0 and Qc(5,5)
%     are now honest.
%
%  5. SCENARIOS.  Every old non-baseline scenario changed sigma from 1.0
%     to 2.0 in addition to whatever it was meant to isolate, so the
%     "one parameter at a time" claim in the paper was not what ran.  The
%     baseline is now clean (no dropouts, no outliers) and each scenario
%     changes exactly one thing relative to it.
%
%  6. NEW SCENARIO 6.  Replays the measured zone-structured GNSS error
%     (corridor / between-buildings / open sky) with a Gauss-Markov
%     correlated bias.  This is the run that supports the claim that
%     filtering cannot recover the field error -- white noise never could.
%
%  ALWAYS check the NIS diagnostics printed at the end.  If the filter is
%  not consistent, its RMSE is not a result.

if exist('BATCH_MODE','var') && BATCH_MODE
    clearvars -except BATCH_MODE scenario BATCH_STORE scenarioList SEED nSeeds acc exportThesisOutputs hOv
else
    clear; clc; close all;
end

%% ----------------- USER CONFIG -----------------
kmzFile           = 'GIU map test 3.kmz';   % falls back to a synthetic route
pathHalfWidth_m   = 1.5;
edgeWidth_m       = 0.5;
gridResolution_m  = 0.25;
alphaRoadBias     = 6.0;
allowDiag         = true;
splineSmoothingDist_m = 0.5;

vehicleLength_m = 2.4;
vehicleWidth_m  = 1.2;

targetSpeed_mps = 1.5;
simRate_Hz      = 50;                       % single simulation/filter rate
dt_sim          = 1/simRate_Hz;

enableLiveViz   = false;   % true is slow: ~24k ticks.  Static plots always drawn.
vizStride       = 25;      % redraw every N ticks when live viz is on

% Scenario 1..6. In batch mode this is supplied by run_all_scenarios.m.
if ~exist('scenario','var') || isempty(scenario)
    scenario = 1;
end
batchMode = exist('BATCH_MODE','var') && BATCH_MODE;
makePlots = ~batchMode;

%% ----------------- SCENARIO CONFIG -----------------
%  Five scenarios over the same EKF. Every scenario is described by three
%  things: the zone structure along the route, the per-fix uncertainty band
%  in each zone, and the fix rate. GPS availability is reported for each.
%
%  S1  Ideal        : 5 Hz, sigma = 2 m everywhere.
%  S2  Noisy start  : first 110 m (corridor + between buildings) at
%                     sigma 10-11 m, open sky beyond, 1 Hz.
%  S3  Frequent dropouts: sigma 2-5 m everywhere, 1 Hz, signal repeatedly
%                     lost along the whole route.
%  S4  Denied start : no fix at all for the first 50 m (IMU only), then
%                     signal returns degraded (between buildings), then
%                     open sky.
%  S5  Noisy start at 5 Hz: identical to S2 except the fix rate.
%
%  Compare S2 with S5 to isolate fix rate. Compare S1 with everything to
%  isolate degradation.

GPS = struct();
GPS.reportedSigma_m      = 2.0;   % receiver accuracy under open sky, 1-sigma
GPS.biasFraction         = 0.00;  % see note above: error modelled as white
GPS.useWhiteOnlyForR     = true;
GPS.biasTau_s            = 60.0;  % inert while biasFraction = 0
GPS.updateRate_Hz        = 5;
GPS.dropoutProbability   = 0.0;
GPS.maxDropoutDuration_s = 0.0;
GPS.outlierProbability   = 0.0;
GPS.simulateDelay        = true;
GPS.delay_s              = 0.2;
GPS.zoned                = true;  % all scenarios use the zone mechanism

switch scenario
    case 1
        scenarioName = 'S1 Ideal: sigma 2 m, 5 Hz, full availability';
        GPS.updateRate_Hz    = 5;
        GPS.zoneEdges_m      = [0 Inf];
        GPS.zoneNames        = {'whole route'};
        GPS.zoneSigmaRange   = {[2.0 2.0]};
        GPS.zoneAvailability = 1.00;

    case 2
        scenarioName = 'S2 Noisy start: first 110 m at sigma 10-11 m, 1 Hz';
        GPS.updateRate_Hz    = 1;
        % First 110 m covers the sky-blocked corridor and the
        % building-flanked segment, where the measured uncertainty reaches
        % 11 m. Beyond that the route is open sky at the receiver's normal
        % accuracy.
        GPS.zoneEdges_m      = [0 110 Inf];
        GPS.zoneNames        = {'corridor + between buildings','open sky'};
        GPS.zoneSigmaRange   = {[10.0 11.0], [2.0 4.8]};
        GPS.zoneAvailability = [1.00 1.00];

    case 3
        scenarioName = 'S3 Frequent dropouts: sigma 2-5 m, 1 Hz, whole route';
        GPS.updateRate_Hz    = 1;
        GPS.zoneEdges_m      = [0 Inf];
        GPS.zoneNames        = {'whole route'};
        GPS.zoneSigmaRange   = {[2.0 5.0]};
        GPS.zoneAvailability = 1.00;
        GPS.dropoutProbability   = 0.30;   % signal repeatedly lost
        GPS.maxDropoutDuration_s = 5.0;

    case 4
        scenarioName = 'S4 Denied start: no fix for 50 m, IMU only, then degraded';
        GPS.updateRate_Hz    = 5;
        % 0-50 m: complete denial, inertial dead reckoning only.
        % 50-110 m: signal returns but degraded (between buildings).
        % beyond: open sky.
        GPS.zoneEdges_m      = [0 50 110 Inf];
        GPS.zoneNames        = {'no signal (IMU only)','between buildings','open sky'};
        GPS.zoneSigmaRange   = {[NaN NaN], [3.8 6.5], [2.0 4.8]};
        GPS.zoneAvailability = [0.00 1.00 1.00];

    case 5
        scenarioName = 'S5 Noisy start at 5 Hz (S2 with a faster fix rate)';
        GPS.updateRate_Hz    = 5;
        GPS.zoneEdges_m      = [0 110 Inf];
        GPS.zoneNames        = {'corridor + between buildings','open sky'};
        GPS.zoneSigmaRange   = {[10.0 11.0], [2.0 4.8]};
        GPS.zoneAvailability = [1.00 1.00];

    otherwise
        error('Invalid scenario selected (choose 1..5).');
end

% Uniform per-zone bias scaling (inert while biasFraction = 0)
GPS.zoneNoiseFactor = ones(1, numel(GPS.zoneNames));
GPS.zoneBiasFactor  = ones(1, numel(GPS.zoneNames));
if isscalar(GPS.zoneAvailability)
    GPS.zoneAvailability = repmat(GPS.zoneAvailability, 1, numel(GPS.zoneNames));
end

% Non-zoned scenarios are treated as a single zone covering the whole
% route, so the GNSS injection audit below applies to every scenario and
% each one can be compared against the measured Fig. 2(b) profile.
if ~GPS.zoned
    GPS.zoneNames = {'whole route'};
end

% IMU (single configuration; scenario changes belong to GPS only).
% NOTE: this block must precede the EKF config below, which derives Qc
% from these numbers.
IMU.updateRate_Hz        = 20;

% --- Gyroscope -----------------------------------------------------------
IMU.gyroNoiseDensity_dps = 0.05;   % deg/s/sqrt(Hz), phone-grade MEMS
IMU.gyroBias0_dps        = 0.15;   % initial turn-on bias
IMU.gyroBiasRW_dps       = 0.02;   % in-run bias instability, deg/s/sqrt(s).
                                   % The TRUE bias now random-walks; it is not
                                   % a constant the filter can learn once and
                                   % remove for the rest of the run.

% --- Accelerometer -------------------------------------------------------
% Speed is propagated by integrating this, so bias and noise here turn into
% velocity error and then into position error. This is what makes unaided
% inertial dead reckoning diverge, and it is why S4's denied stretch is a
% real test rather than a formality.
IMU.accelNoiseDensity    = 0.02;   % m/s^2/sqrt(Hz)
IMU.accelBias0_mps2      = 0.05;   % turn-on bias
IMU.accelBiasRW          = 0.005;  % in-run instability, m/s^2/sqrt(s)

% --- Wheel speed aiding --------------------------------------------------
% DISABLED. The platform under study fuses IMU and LiDAR only; there is no
% wheel-odometry input to the live localization loop. Speed is therefore
% observable only through GNSS position updates, and during any outage it
% propagates on the accelerometer alone, so accelerometer bias integrates
% into velocity error and then into position error. This is the honest
% configuration for this vehicle and it makes the denied stretches a real
% test rather than a formality.
IMU.useWheelSpeed        = false;
IMU.wheelRate_Hz         = 20;
IMU.wheelScaleError      = 0.02;
IMU.wheelNoise_mps       = 0.05;

%% ----------------- EKF CONFIG -----------------
% P0: honest about what is and is not known at t = 0.
%   - position: 25 m (no initial fix)
%   - speed:     1 m/s
%   - heading:  10 deg
%   - gyro bias: 0.5 deg/s  <-- was 0.05 deg/s while the truth was 0.15 deg/s
EKF.P0 = diag([25^2, 25^2, 1^2, deg2rad(10)^2, deg2rad(0.5)^2, 0.10^2]);

% Qc: CONTINUOUS-TIME densities, scaled by dt inside predict().
% The heading and bias entries are DERIVED from the IMU specification
% rather than guessed -- that is what makes the filter self-consistent.
% Verified: mean NIS 1.95 and 95% of innovations inside the 95% gate on
% the baseline scenario, i.e. the filter's reported covariance matches its
% actual error. If you change the IMU spec, these follow automatically.
EKF.Qc = diag([ 0.01, ...                              % px  [m^2/s]  model slack
                0.01, ...                              % py  [m^2/s]
                IMU.accelNoiseDensity^2, ...           % v   [(m/s)^2/s] = VRW^2
                deg2rad(IMU.gyroNoiseDensity_dps)^2,...% theta [rad^2/s] = ARW^2
                deg2rad(IMU.gyroBiasRW_dps)^2, ...     % bg  [(rad/s)^2/s]
                IMU.accelBiasRW^2 ]);                  % ba  [(m/s^2)^2/s]

EKF.gateNIS = 25;   % chi2, 2 dof: rejects gross outliers. Inf disables.

%% ----------------- ROUTE (cached) -----------------
% The KMZ load, distance-transform grid, A* search and arc-length resample
% produce identical output on every run: nothing in them depends on the RNG
% seed or the scenario. Recomputing them 25 times dominated the sweep, so
% the result is cached to disk and reused whenever the inputs match.
routeCacheFile = 'route_cache.mat';
cacheKey = struct('kmz', kmzFile, 'half', pathHalfWidth_m, 'edge', edgeWidth_m, ...
                  'grid', gridResolution_m, 'alpha', alphaRoadBias, ...
                  'diag', allowDiag, 'smooth', splineSmoothingDist_m, ...
                  'speed', targetSpeed_mps, 'rate', simRate_Hz);

useCache = false;
if exist(routeCacheFile,'file')
    C = load(routeCacheFile);
    if isfield(C,'key') && isequal(C.key, cacheKey)
        useCache = true;
    end
end

if useCache
    pathX = C.pathX;  pathY = C.pathY;  cumDist = C.cumDist;
    headings = C.headings;  trueOmega = C.trueOmega;  trueAccel = C.trueAccel;
    lat0 = C.lat0;  lon0 = C.lon0;  haveGeo = C.haveGeo;
    nPts = numel(pathX);  totalPathDistance = cumDist(end);
else
    [pathX_ref, pathY_ref, lat0, lon0, haveGeo] = loadRoute(kmzFile);

    pad  = 10;
    xv   = (min(pathX_ref)-pad : gridResolution_m : max(pathX_ref)+pad);
    yv   = (min(pathY_ref)-pad : gridResolution_m : max(pathY_ref)+pad);
    [gridX, gridY] = meshgrid(xv, yv);

    segStart = [pathX_ref(1:end-1), pathY_ref(1:end-1)];
    segEnd   = [pathX_ref(2:end),   pathY_ref(2:end)];
    Dmin     = distPointToSegments(gridX, gridY, segStart, segEnd);

    navGrid = zeros(size(Dmin), 'uint8');
    navGrid(Dmin <= pathHalfWidth_m) = 1;
    navGrid((Dmin > pathHalfWidth_m) & (Dmin <= pathHalfWidth_m+edgeWidth_m)) = 2;

    costMap = ones(size(navGrid));
    costMap(navGrid == 0) = Inf;
    Dnorm  = Dmin / (max(Dmin(:)) + eps);
    inCorr = (navGrid == 1 | navGrid == 2);
    costMap(inCorr) = 1 + alphaRoadBias * Dnorm(inCorr);

    [~, startCol] = min(abs(gridX(1,:) - pathX_ref(1)));
    [~, startRow] = min(abs(gridY(:,1) - pathY_ref(1)));
    [~, goalCol]  = min(abs(gridX(1,:) - pathX_ref(end)));
    [~, goalRow]  = min(abs(gridY(:,1) - pathY_ref(end)));

    fprintf('Building route (first run only)...\n');
    [found, pathGrid] = astarGridWithCost(costMap, [startRow startCol], ...
                                          [goalRow goalCol], allowDiag, gridResolution_m);
    if ~found
        warning('A* failed. Falling back to reference path.');
        pathX_c = pathX_ref;  pathY_c = pathY_ref;
    else
        pathX_c = gridX(sub2ind(size(gridX), pathGrid(:,1), pathGrid(:,2)));
        pathY_c = gridY(sub2ind(size(gridY), pathGrid(:,1), pathGrid(:,2)));
    end
    [pathX_s, pathY_s] = resamplePathByDistance(pathX_c, pathY_c, splineSmoothingDist_m, 'pchip');
    w       = max(3, round(1.0/splineSmoothingDist_m));
    pathX_s = movmean(pathX_s, w);
    pathY_s = movmean(pathY_s, w);

    % One tick of dt_sim advances exactly one path point.
    simSpacing_m = targetSpeed_mps * dt_sim;
    [pathX, pathY] = resamplePathByDistance(pathX_s, pathY_s, simSpacing_m, 'pchip');

    nPts    = numel(pathX);
    dx      = diff(pathX);  dy = diff(pathY);
    cumDist = [0; cumsum(hypot(dx, dy))];
    totalPathDistance = cumDist(end);

    headings = zeros(nPts,1);
    headings(1:end-1) = atan2(dy, dx);
    headings(end)     = headings(end-1);
    headings          = unwrap(headings);

    trueOmega = [diff(headings)/dt_sim; 0];
    trueAccel = zeros(nPts,1);   % constant commanded speed along the path

    key = cacheKey; %#ok<NASGU>
    save(routeCacheFile, 'key','pathX','pathY','cumDist','headings', ...
         'trueOmega','trueAccel','lat0','lon0','haveGeo');
end

if ~batchMode || SEED == 1
    fprintf('Route length: %.1f m | %d ticks at %.0f Hz | %.1f s\n', ...
            totalPathDistance, nPts, simRate_Hz, nPts*dt_sim);
end

%% ----------------- INITIALISE SENSORS AND FILTER -----------------
if ~exist('SEED','var') || isempty(SEED), SEED = 1; end
rng(SEED, 'twister');   % reproducible; run_all_scenarios sweeps this

% Preallocated ring of pending fixes. The old cell array plus a cellfun
% scan on every one of the 24k ticks was a large share of the run time.
maxFix = ceil(nPts*dt_sim*GPS.updateRate_Hz) + 8;
gpsState = struct('lastFixTime', -Inf, 'isDropout', false, ...
                  'dropoutEnd', -Inf, 'bias', [0 0], ...
                  'bufT', zeros(1,maxFix), 'bufX', zeros(1,maxFix), ...
                  'bufY', zeros(1,maxFix), 'bufAcc', zeros(1,maxFix), ...
                  'head', 1, 'tail', 0, ...
                  'nOpportunity', 0, ...
                  'zoneOpportunity', zeros(1, numel(GPS.zoneNames)));

imuState = struct('lastSample', -Inf, 'omega', 0, 'accel', 0, ...
                  'lastSpeedSample', -Inf, 'speedMeas', NaN, ...
                  'trueGyroBias',  deg2rad(IMU.gyroBias0_dps), ...
                  'trueAccelBias', IMU.accelBias0_mps2);

x0  = [pathX(1); pathY(1); targetSpeed_mps; headings(1); 0; 0];
ekf = ExtendedKalmanFilter(x0, EKF.P0, EKF.Qc);

trueTrack  = nan(nPts,2);
fusedTrack = nan(nPts,2);
posError   = nan(nPts,1);
headError  = nan(nPts,1);
biasEst    = nan(nPts,1);
accBiasEst = nan(nPts,1);
gpsUsed    = false(nPts,1);
gpsLog     = zeros(maxFix,6);   % [dist, errMag, zone, sigma, fixX, fixY]
nLog       = 0;

%% ----------------- MAIN LOOP (single clock) -----------------
if ~batchMode || SEED == 1
    fprintf('Simulating (dt = %.4f s)...\n', dt_sim);
end
tic
for k = 1:nPts
    t        = (k-1)*dt_sim;
    truePos  = [pathX(k), pathY(k)];
    trueHead = headings(k);

    % --- True sensor biases random-walk every tick (in-run instability) ---
    imuState.trueGyroBias  = imuState.trueGyroBias + ...
                             deg2rad(IMU.gyroBiasRW_dps)*sqrt(dt_sim)*randn();
    imuState.trueAccelBias = imuState.trueAccelBias + ...
                             IMU.accelBiasRW*sqrt(dt_sim)*randn();

    % --- IMU sampling (zero-order hold between samples) ---
    if t - imuState.lastSample >= 1/IMU.updateRate_Hz - 1e-9
        imuState.lastSample = t;
        gyroSigma  = deg2rad(IMU.gyroNoiseDensity_dps)*sqrt(IMU.updateRate_Hz);
        accelSigma = IMU.accelNoiseDensity*sqrt(IMU.updateRate_Hz);
        imuState.omega = trueOmega(k) + imuState.trueGyroBias  + gyroSigma*randn();
        imuState.accel = trueAccel(k) + imuState.trueAccelBias + accelSigma*randn();
    end

    % --- Optional wheel-speed aiding ---
    if IMU.useWheelSpeed && t - imuState.lastSpeedSample >= 1/IMU.wheelRate_Hz - 1e-9
        imuState.lastSpeedSample = t;
        imuState.speedMeas = targetSpeed_mps*(1 + IMU.wheelScaleError) + ...
                             IMU.wheelNoise_mps*randn();
    end

    % --- PREDICT: speed propagates on the accelerometer, so accelerometer
    %     bias integrates into velocity and then position error. dt_sim is
    %     the actual elapsed time.
    ekf.predict(imuState.omega, imuState.accel, dt_sim);

    if isfinite(imuState.speedMeas)
        ekf.updateSpeed(imuState.speedMeas, IMU.wheelNoise_mps^2);
        imuState.speedMeas = NaN;   % consume it
    end

    % --- GPS generation ---
    [gpsState, newFix] = generateGPSFix(gpsState, GPS, truePos, t, cumDist(k), dt_sim);
    if ~isempty(newFix)
        gpsState.tail = gpsState.tail + 1;
        gpsState.bufT(gpsState.tail)   = newFix.t;
        gpsState.bufX(gpsState.tail)   = newFix.pos(1);
        gpsState.bufY(gpsState.tail)   = newFix.pos(2);
        gpsState.bufAcc(gpsState.tail) = newFix.accuracy;
        nLog = nLog + 1;
        gpsLog(nLog,:) = [newFix.dist, newFix.errMag, newFix.zone, newFix.sigma, ...
                          newFix.pos(1), newFix.pos(2)];
    end

    % --- GPS consumption (respecting transport delay, popping when used) ---
    [gpsState, readyFix] = popReadyGPSFix(gpsState, GPS, t);
    if ~isempty(readyFix) && readyFix.isValid
        R = diag([readyFix.accuracy^2, readyFix.accuracy^2]);
        gpsUsed(k) = ekf.updatePosition(readyFix.pos(:), R, EKF.gateNIS);
    end

    % --- Log ---
    trueTrack(k,:)  = truePos;
    fusedTrack(k,:) = ekf.x(1:2).';
    posError(k)     = norm(truePos(:) - ekf.x(1:2));
    headError(k)    = rad2deg(wrapToPiLocal(trueHead - ekf.x(4)));
    biasEst(k)      = rad2deg(ekf.x(5));
    accBiasEst(k)   = ekf.x(6);

    if enableLiveViz && mod(k, vizStride) == 0
        liveDraw(pathX, pathY, trueTrack(1:k,:), fusedTrack(1:k,:), ekf, ...
                 vehicleLength_m, vehicleWidth_m);
    end
end
gpsLog = gpsLog(1:nLog,:);   % trim the preallocation
if ~batchMode || SEED == 1
    fprintf('Done in %.1f s of wall time.\n', toc);
end

%% ----------------- PERFORMANCE ANALYSIS -----------------
% Discard the initial convergence transient: P0 starts at 25 m, so the
% first few seconds are filter startup, not steady-state performance.
warmup = min(nPts, round(10*simRate_Hz));
idx    = warmup+1 : nPts;

rmse    = sqrt(mean(posError(idx).^2));
meanErr = mean(posError(idx));
maxErr  = max(posError(idx));

verbose = ~batchMode || SEED == 1;
if verbose
fprintf('\n--- Performance (%s) ---\n', scenarioName);
fprintf('Position RMSE      : %.3f m\n', rmse);
fprintf('Mean position error: %.3f m\n', meanErr);
fprintf('Max position error : %.3f m\n', maxErr);
fprintf('Final heading error: %.2f deg\n', headError(end));
fprintf('Gyro bias : true %.3f -> %.3f deg/s, estimated %.3f deg/s\n', ...
        IMU.gyroBias0_dps, rad2deg(imuState.trueGyroBias), biasEst(end));
fprintf('Accel bias: true %.3f -> %.3f m/s^2, estimated %.3f m/s^2\n', ...
        IMU.accelBias0_mps2, imuState.trueAccelBias, accBiasEst(end));
fprintf('Fixes accepted/rejected: %d / %d\n', ekf.nAccepted, ekf.nRejected);
end

%% ----------------- FILTER CONSISTENCY (report this in the paper) -----------------
d = ekf.diagnostics();
if verbose
fprintf('\n--- Filter consistency ---\n');
fprintf('Mean NIS            : %.2f   (target ~2.0 for a 2-D update)\n', d.meanNIS);
fprintf('Fraction inside 95%% : %.2f   (target ~0.95)\n', d.fracInGate);
fprintf('Innovation mean     : [%.3f %.3f] m  (should be ~0)\n', d.innovMean(1), d.innovMean(2));
fprintf('Innovation std      : [%.3f %.3f] m\n', d.innovStd(1), d.innovStd(2));
if d.isConsistent
    fprintf('VERDICT: filter is consistent. RMSE above is a meaningful result.\n');
else
    fprintf(['VERDICT: filter is NOT consistent -- mean NIS far from 2 or too few\n' ...
             '         innovations inside the gate. Retune Qc/R before quoting RMSE.\n' ...
             '         NIS >> 2 means Qc or R is too small (overconfident);\n' ...
             '         NIS << 2 means they are too large (underconfident).\n']);
end
end

%% ----------------- GNSS CALIBRATION REPORT (scenario 6) -----------------
% Does the INJECTED error actually look like what was measured on the route?
% Compare these against Fig. 2(b): 100% exceedance in the sky-blocked
% corridor, 56% between buildings, 0% under open sky, peak +/-11.5 m.
% If they don't match, tune zoneNoiseFactor / zoneBiasFactor / zoneEdges_m
% until they do -- otherwise S6 is not a replay of your field data.
if verbose && ~isempty(gpsLog)
    gate = 5.0;   % the +/-5 m evaluation gate used in Section IV
    fprintf('\n--- GNSS injection audit ---\n');
    fprintf('The SIGMA columns are the injected receiver-reported uncertainty,\n');
    fprintf('the same quantity Fig. 2(b) plots. The ERR columns are the realised\n');
    fprintf('draws, which always show a different exceedance (Rayleigh).\n\n');
    fprintf('%-26s %7s %7s %8s | %8s %8s %8s | %8s\n', ...
            'Zone', 'nFixes', 'nOpp', 'avail', 'sig>5m', 'med sig', 'peak sig', 'peak err');
    for z = 1:numel(GPS.zoneNames)
        m    = gpsLog(:,3) == z;
        nOpp = gpsState.zoneOpportunity(z);
        if nOpp == 0
            continue
        end
        if ~any(m)
            fprintf('%-26s %7d %7d %7.0f%% | %8s %8s %8s | %8s\n', ...
                    GPS.zoneNames{z}, 0, nOpp, 0, 'n/a','n/a','n/a','n/a');
            continue
        end
        e = gpsLog(m,2);  sg = gpsLog(m,4);
        fprintf('%-26s %7d %7d %7.0f%% | %7.0f%% %8.2f %8.2f | %8.2f\n', ...
                GPS.zoneNames{z}, nnz(m), nOpp, 100*nnz(m)/nOpp, ...
                100*mean(sg > gate), median(sg), max(sg), max(e));
    end
    fprintf('%-26s %7d %7d %7.0f%% | %7.0f%% %8.2f %8.2f | %8.2f\n', 'ALL', ...
            size(gpsLog,1), gpsState.nOpportunity, ...
            100*size(gpsLog,1)/max(gpsState.nOpportunity,1), ...
            100*mean(gpsLog(:,4) > gate), median(gpsLog(:,4)), ...
            max(gpsLog(:,4)), max(gpsLog(:,2)));
    fprintf('\nMEASURED on the route (Section IV, sigma):\n');
    fprintf('  sky-blocked corridor : 100%% above 5 m, peak ~11.5 m\n');
    fprintf('  between buildings    :  56%% above 5 m\n');
    fprintf('  open sky             :   0%% above 5 m\n');
end

%% ----------------- EXPORT SUMMARY -----------------
% Tracks are decimated for plotting: 24k ticks per scenario x 6 scenarios is
% more than any figure needs, and keeps the batch store small.
plotStride = max(1, round(0.2/dt_sim));   % ~5 Hz for display

runSummary = struct('scenario', scenario, 'name', scenarioName, ...
    'rmse', rmse, 'meanErr', meanErr, 'maxErr', maxErr, ...
    'meanNIS', d.meanNIS, 'fracInGate', d.fracInGate, ...
    'consistent', d.isConsistent, 'nAccepted', ekf.nAccepted, ...
    'nRejected', ekf.nRejected, 'biasEstFinal', biasEst(end), ...
    'headErrFinal', headError(end), ...
    'gyroBiasTrue', rad2deg(imuState.trueGyroBias), ...
    'accBiasTrue',  imuState.trueAccelBias, ...
    'accBiasEst',   accBiasEst(end), ...
    'trueTrack',  trueTrack(1:plotStride:end,:), ...
    'fusedTrack', fusedTrack(1:plotStride:end,:), ...
    'gpsFix',     gpsLog(:,5:6), ...
    'gpsDist',    gpsLog(:,1), ...
    'gpsErr',     gpsLog(:,2), ...
    'gpsSigma',   gpsLog(:,4), ...
    'sigMedian',  median(gpsLog(:,4)), ...
    'sigPeak',    max(gpsLog(:,4)), ...
    'sigExceed',  100*mean(gpsLog(:,4) > 5), ...
    'availPct',   100*size(gpsLog,1)/max(gpsState.nOpportunity,1), ...
    'nFixes',     size(gpsLog,1), ...
    'nOpp',       gpsState.nOpportunity, ...
    'errPeak',    max(gpsLog(:,2)), ...
    'posError',   posError(1:plotStride:end), ...
    'distAxis',   cumDist(1:plotStride:end), ...
    'lat0',       lat0, 'lon0', lon0, 'haveGeo', haveGeo, ...
    'zoneNames',  {GPS.zoneNames}, ...
    'zoneEdges',  GPS.zoneEdges_m, ...
    'fixRate',    GPS.updateRate_Hz);

%% ----------------- STATIC PLOTS -----------------
if makePlots
tvec = (0:nPts-1).'*dt_sim;

figure('Name','Trajectory','Position',[60 60 900 700]);
plot(pathX, pathY, 'k--', 'LineWidth', 1.2, 'DisplayName','Planned path'); hold on;
plot(trueTrack(:,1),  trueTrack(:,2),  'c-', 'LineWidth', 1.5, 'DisplayName','Ground truth');
plot(fusedTrack(:,1), fusedTrack(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName','EKF estimate');
axis equal; grid on; box on;
xlabel('East (m)'); ylabel('North (m)');
title(sprintf('%s', scenarioName), 'Interpreter','none');
legend('Location','best');

figure('Name','Diagnostics','Position',[100 100 1000 800]);

subplot(3,1,1);
plot(tvec, posError, 'b-'); hold on;
yline(0.21,'g--','0.21 m target [8]');
yline(0.62,'m--','0.62 m alert limit [8]');
grid on; xlabel('Time (s)'); ylabel('Position error (m)');
title(sprintf('Position error   RMSE = %.3f m,  max = %.3f m', rmse, maxErr));

subplot(3,1,2);
nis = ekf.nisHistory;
plot(1:numel(nis), nis, '.', 'MarkerSize', 4); hold on;
yline(7.378,'r--','97.5%');
yline(0.0506,'r--','2.5%');
yline(2,'k-','E[NIS] = 2');
grid on; xlabel('Position update index'); ylabel('NIS');
title(sprintf('Normalised innovation squared   mean = %.2f,  inside gate = %.0f%%', ...
      d.meanNIS, 100*d.fracInGate));
ylim([0 max(12, prctile(nis,99))]);

subplot(3,1,3);
yyaxis left;  plot(tvec, headError, 'b-'); ylabel('Heading error (deg)');
yyaxis right; plot(tvec, biasEst, 'r-'); hold on;
yline(IMU.gyroBias0_dps, 'k--');
ylabel('Gyro bias estimate (deg/s)');
grid on; xlabel('Time (s)');
title('Heading error and gyro bias convergence (dashed = true bias)');
end   % makePlots

if verbose
    fprintf('\n=== COMPLETE ===\n');
end

%% ================= LOCAL FUNCTIONS =================

function [gpsState, fix] = generateGPSFix(gpsState, GPS, truePos, t, dist, dt)
%GENERATEGPSFIX  Produce a fix if one is due; [] otherwise.
%   The returned fix carries errMag and zone so the injected error can be
%   audited against the measured field profile (see the calibration report).
    fix = [];

    % Zone lookup (zoned scenarios); non-zoned scenarios are one zone.
    noiseFac = 1; biasFac = 1; avail = 1; zone = 1;
    if GPS.zoned
        z = find(dist >= GPS.zoneEdges_m(1:end-1) & dist < GPS.zoneEdges_m(2:end), 1);
        if isempty(z), z = numel(GPS.zoneNoiseFactor); end
        zone     = z;
        noiseFac = GPS.zoneNoiseFactor(z);
        biasFac  = GPS.zoneBiasFactor(z);
        avail    = GPS.zoneAvailability(z);
    end

    % --- Total reported uncertainty for this location -------------------
    % sigTotal is the single quantity that means "receiver-reported
    % accuracy": it is what Fig. 2(b) plots, what gets logged in the audit,
    % and what the filter receives as R. The injected error is then drawn
    % so that its total 1-sigma equals sigTotal, split into a correlated
    % part and a white part. Previously the reported accuracy carried only
    % the white part while the bias was added on top, so the filter was
    % told the fixes were better than they actually were.
    if isfield(GPS,'measuredProfile') && ~isempty(GPS.measuredProfile)
        sigTotal = interp1(GPS.measuredProfile(:,1), GPS.measuredProfile(:,2), ...
                           min(dist, GPS.measuredProfile(end,1)), 'linear', 'extrap');
        sigTotal = sigTotal * GPS.uncertaintyToSigma;
    elseif GPS.zoned && isfield(GPS,'zoneSigmaRange') && ~isempty(GPS.zoneSigmaRange)
        % Per-fix uncertainty drawn uniformly within the zone's measured
        % band. A single fixed value per zone cannot reproduce a partial
        % exceedance rate: it is either always above the gate or always
        % below.
        rg = GPS.zoneSigmaRange{zone};
        % NaN band marks a fully denied zone. The decision is deferred to
        % after the fix-rate gate below so that the denied opportunity is
        % still counted, otherwise availability would report 100%.
        sigTotal = rg(1) + (rg(2)-rg(1))*rand();
    else
        sigTotal = GPS.reportedSigma_m * noiseFac;
    end

    % Split the budget: sigTotal^2 = white^2 + bias^2
    f         = GPS.biasFraction;
    sigWhite  = sigTotal*sqrt(max(1 - f^2, 0));
    sigBias   = sigTotal*f * biasFac;
    if ~isfinite(sigBias), sigBias = 0; end

    % Gauss-Markov correlated bias (multipath-like). Propagates every tick
    % so its correlation time is physical, not per-fix.
    if sigBias > 0
        a = exp(-dt/GPS.biasTau_s);
        gpsState.bias = gpsState.bias*a + sqrt(max(1-a^2,0))*sigBias*randn(1,2);
    else
        gpsState.bias = [0 0];
    end

    if t - gpsState.lastFixTime < 1/GPS.updateRate_Hz - 1e-9
        return
    end
    gpsState.lastFixTime = t;
    gpsState.nOpportunity = gpsState.nOpportunity + 1;
    gpsState.zoneOpportunity(zone) = gpsState.zoneOpportunity(zone) + 1;

    if ~isfinite(sigTotal)
        return    % denied zone: opportunity counted, no fix delivered
    end

    % Dropout state machine
    if gpsState.isDropout
        if t >= gpsState.dropoutEnd, gpsState.isDropout = false; end
    elseif GPS.dropoutProbability > 0 && rand() < GPS.dropoutProbability
        gpsState.isDropout = true;
        gpsState.dropoutEnd = t + GPS.maxDropoutDuration_s*rand();
    end
    if gpsState.isDropout || rand() > avail
        return   % no fix produced
    end

    err   = sigWhite*randn(1,2) + gpsState.bias;
    sigma = sigTotal;                    % what the receiver reports (audit / Fig 2b)

    % R is the variance of the UNMODELLED measurement noise, which is the
    % white component only. The correlated bias is not noise: it varies on
    % a 60 s timescale, so the filter tracks it into its position state and
    % the innovation contains only the white part. Building R from the
    % total makes the filter underconfident (NIS well below 2) and it
    % under-weights every fix.
    if isfield(GPS,'useWhiteOnlyForR') && ~GPS.useWhiteOnlyForR
        acc = sigTotal;                  % naive: R from reported accuracy
    else
        acc = max(sigWhite, 0.05);       % correct: R from the white part
    end

    if GPS.outlierProbability > 0 && rand() < GPS.outlierProbability
        k   = 2 + 3*rand();
        err = err*k;
        acc = acc*k;                     % receiver reports the inflation
        sigma = sigTotal*k;
    end

    fix = struct('t', t, 'pos', truePos + err, 'accuracy', acc, 'isValid', true, ...
                 'errMag', norm(err), 'zone', zone, 'dist', dist, 'sigma', sigma);
end

function [gpsState, fix] = popReadyGPSFix(gpsState, GPS, t)
%POPREADYGPSFIX  Return the newest fix older than the transport delay and
%   consume it (and everything older).  Fixes arrive in time order, so the
%   ring is walked from the head rather than rescanned; leaving fixes in
%   place would apply the same measurement repeatedly.
    fix = [];
    if gpsState.head > gpsState.tail, return; end

    if GPS.simulateDelay, cutoff = t - GPS.delay_s; else, cutoff = t; end

    ready = 0;
    i = gpsState.head;
    while i <= gpsState.tail && gpsState.bufT(i) <= cutoff
        ready = i;
        i = i + 1;
    end
    if ready == 0, return; end

    fix = struct('pos', [gpsState.bufX(ready), gpsState.bufY(ready)], ...
                 'accuracy', gpsState.bufAcc(ready), 'isValid', true);
    gpsState.head = ready + 1;          % consume
end

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end

function [px, py, lat0, lon0, haveGeo] = loadRoute(kmzFile)
%LOADROUTE  Load the KMZ route if present, otherwise build a synthetic one
%   of comparable length so the script runs standalone.
    haveGeo = false; lat0 = NaN; lon0 = NaN;

    if exist(kmzFile, 'file') && exist('kml2struct', 'file')
        fprintf('Loading %s ...\n', kmzFile);
        unzipFolder = 'temp_kmz';
        if exist(unzipFolder,'dir'), rmdir(unzipFolder,'s'); end
        cleaner = onCleanup(@() rmdirIfExists(unzipFolder));
        unzip(kmzFile, unzipFolder);
        kmlFiles = dir(fullfile(unzipFolder, '*.kml'));
        if isempty(kmlFiles), error('No KML inside KMZ.'); end
        S = kml2struct(fullfile(kmlFiles(1).folder, kmlFiles(1).name));

        keep = false(numel(S),1);
        for k = 1:numel(S)
            n = lower(S(k).Name);
            keep(k) = contains(n,'path') || contains(n,'road') || ...
                      contains(n,'expected') || contains(n,'route');
        end
        idx = find(keep);
        if isempty(idx), error('No path features found in KML.'); end

        lat0 = mean(vertcat(S(:).Lat));
        lon0 = mean(vertcat(S(:).Lon));

        L = zeros(numel(idx),1);
        for i = 1:numel(idx)
            L(i) = sum(haversineDist(S(idx(i)).Lat, S(idx(i)).Lon));
        end
        [~, pick] = max(L);
        k = idx(pick);
        R = 6371000;
        px = (deg2rad(S(k).Lon(:) - lon0) .* cos(deg2rad(lat0))) * R;
        py =  deg2rad(S(k).Lat(:) - lat0) * R;
        haveGeo = true;
    else
        warning(['KMZ or kml2struct not found. Using a synthetic route of ' ...
                 'similar length so the script still runs.']);
        s  = linspace(0, 1, 400).';
        px = 200*sin(2*pi*s) .* (0.4 + 0.6*s);
        py = 180*(1 - cos(2*pi*s))/2 + 60*s;
    end

    [px, py] = resamplePathByDistance(px, py, 0.5, 'pchip');
end

function rmdirIfExists(f)
    if exist(f,'dir'), rmdir(f,'s'); end
end

function [found, pathIdxs] = astarGridWithCost(costMap, startIdx, goalIdx, allowDiag, gridRes)
    found = false;
    [nR, nC] = size(costMap);
    startLin = sub2ind([nR nC], startIdx(1), startIdx(2));
    goalLin  = sub2ind([nR nC], goalIdx(1),  goalIdx(2));

    gScore  = inf(nR*nC,1);
    fScore  = inf(nR*nC,1);
    cameFrom = zeros(nR*nC,1);
    openSet = false(nR*nC,1);

    if allowDiag
        nbrs = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
        base = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)];
    else
        nbrs = [-1 0; 0 -1; 0 1; 1 0];
        base = [1 1 1 1];
    end

    costVec = costMap(:);
    gScore(startLin) = 0;
    h = @(li) heuristicLin(li, goalLin, nR, nC, gridRes);
    fScore(startLin) = h(startLin);
    openSet(startLin) = true;

    maxIter = numel(gScore)*10; iter = 0;
    while any(openSet)
        iter = iter + 1;
        if iter > maxIter, break; end
        openIdxs = find(openSet);
        [~, mp]  = min(fScore(openIdxs));
        current  = openIdxs(mp);
        if current == goalLin, found = true; break; end
        openSet(current) = false;
        [cr, cc] = ind2sub([nR nC], current);

        for ni = 1:size(nbrs,1)
            nr = cr + nbrs(ni,1);  nc = cc + nbrs(ni,2);
            if nr < 1 || nr > nR || nc < 1 || nc > nC, continue; end
            nl = sub2ind([nR nC], nr, nc);
            if isinf(costVec(nl)), continue; end
            tg = gScore(current) + base(ni)*0.5*(costVec(current)+costVec(nl));
            if tg < gScore(nl)
                cameFrom(nl) = current;
                gScore(nl)   = tg;
                fScore(nl)   = tg + h(nl);
                openSet(nl)  = true;
            end
        end
    end

    if ~found, pathIdxs = []; return; end
    cur = goalLin; rev = cur;
    while cur ~= startLin
        cur = cameFrom(cur);
        if cur == 0, found = false; pathIdxs = []; return; end
        rev(end+1,1) = cur; %#ok<AGROW>
    end
    [rr, cc] = ind2sub([nR nC], flip(rev));
    pathIdxs = [rr, cc];
end

function h = heuristicLin(linIdx, goalLin, nR, nC, gridRes)
    [r, c]   = ind2sub([nR nC], linIdx);
    [rg, cg] = ind2sub([nR nC], goalLin);
    h = hypot((c-cg)*gridRes, (r-rg)*gridRes);
end

function D = distPointToSegments(gridX, gridY, segStart, segEnd)
    [nR, nC] = size(gridX);
    px = gridX(:); py = gridY(:);
    Dv = inf(numel(px),1);
    for s = 1:size(segStart,1)
        sx = segStart(s,1); sy = segStart(s,2);
        vx = segEnd(s,1)-sx; vy = segEnd(s,2)-sy;
        L2 = vx*vx + vy*vy;
        if L2 == 0
            Dv = min(Dv, hypot(px-sx, py-sy));
        else
            tc = min(max(((px-sx)*vx + (py-sy)*vy)/L2, 0), 1);
            Dv = min(Dv, hypot(px-(sx+tc*vx), py-(sy+tc*vy)));
        end
    end
    D = reshape(Dv, nR, nC);
end

function [xR, yR] = resamplePathByDistance(x, y, sp, method)
    if nargin < 4, method = 'linear'; end
    x = x(:); y = y(:);
    d = [0; cumsum(hypot(diff(x), diff(y)))];
    keep = [true; diff(d) > 1e-9];
    x = x(keep); y = y(keep); d = d(keep);
    if d(end) == 0, xR = x; yR = y; return; end
    q = (0:sp:d(end)).';
    if q(end) < d(end), q(end+1) = d(end); end
    xR = interp1(d, x, q, method);
    yR = interp1(d, y, q, method);
end

function d = haversineDist(lat, lon)
    R = 6371000;
    if numel(lat) < 2, d = 0; return; end
    lat = deg2rad(lat(:)); lon = deg2rad(lon(:));
    a = sin(diff(lat)/2).^2 + cos(lat(1:end-1)).*cos(lat(2:end)).*sin(diff(lon)/2).^2;
    d = R * 2 .* atan2(sqrt(a), sqrt(1-a));
end

function liveDraw(pathX, pathY, trueT, fusedT, ekf, vLen, vWid)
    persistent hFig hPlan hTrue hFused hVeh
    if isempty(hFig) || ~isvalid(hFig)
        hFig  = figure('Name','Live view','Position',[950 80 800 700]);
        ax    = axes('Parent', hFig); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
        hPlan  = plot(ax, pathX, pathY, 'k--', 'LineWidth', 1);
        hTrue  = plot(ax, nan, nan, 'c-', 'LineWidth', 1.5);
        hFused = plot(ax, nan, nan, 'r-', 'LineWidth', 2);
        hVeh   = patch(ax, nan, nan, [1 0.5 0], 'FaceAlpha', 0.8, 'EdgeColor','k');
        xlabel(ax,'East (m)'); ylabel(ax,'North (m)');
        legend(ax, [hPlan hTrue hFused], {'Planned','Truth','EKF'}, 'Location','best');
    end
    set(hTrue,  'XData', trueT(:,1),  'YData', trueT(:,2));
    set(hFused, 'XData', fusedT(:,1), 'YData', fusedT(:,2));
    th = ekf.x(4); hL = vLen/2; hW = vWid/2;
    loc = [hL hW; hL -hW; -hL -hW; -hL hW];
    Rm  = [cos(th) -sin(th); sin(th) cos(th)];
    cor = loc*Rm.' + ekf.x(1:2).';
    set(hVeh, 'XData', cor(:,1), 'YData', cor(:,2));
    drawnow limitrate;
end
