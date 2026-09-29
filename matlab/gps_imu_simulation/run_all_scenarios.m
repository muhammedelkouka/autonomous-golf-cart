%% RUN_ALL_SCENARIOS  Execute scenarios 1-6 and print a comparison table.
%
%  Usage:  put this file, giu_ekf_fusion.m and ExtendedKalmanFilter.m in the
%          same folder, then run:
%
%              clear classes
%              run_all_scenarios
%
%  Plots are suppressed per-scenario; a summary figure is drawn at the end.
%  To see the full diagnostic plots for one scenario, run giu_ekf_fusion
%  on its own with `scenario = N;` set at the top of that file.
%
%  READ THE CONSISTENCY COLUMN.  A scenario whose filter is not consistent
%  (mean NIS far from 2, or few innovations inside the gate) has an RMSE
%  that means nothing and must not be quoted in the paper.

clear classes %#ok<CLCLS>
clear; clc; close all;
outRoot = '';   % resolved below; also consumed by export_thesis_figures

% Save every per-scenario figure and the numerical analysis to disk when
% the sweep finishes. Set false to look at the on-screen figures only.
exportThesisOutputs = true;

BATCH_MODE  = true;
BATCH_STORE = struct([]);

scenarioList = 1:5;
nSeeds       = 5;      % average over this many noise realisations.
                       % A single seed is not enough: with one draw the
                       % scenario ordering can invert (fewer fixes appearing
                       % to beat more), which is sampling noise, not a result.

for scenario = scenarioList
    fprintf('\n================ SCENARIO %d ================\n', scenario);
    clear acc
    for SEED = 1:nSeeds
        giu_ekf_fusion;                  %#ok<NOPTS>
        if SEED == 1, acc = runSummary; else, acc(SEED) = runSummary; end %#ok<SAGROW>
    end
    % Average the scalar metrics across seeds; keep seed 1's tracks for plots.
    s = acc(1);
    s.rmse       = mean([acc.rmse]);
    s.rmseStd    = std([acc.rmse]);
    s.meanErr    = mean([acc.meanErr]);
    s.maxErr     = mean([acc.maxErr]);
    s.meanNIS    = mean([acc.meanNIS]);
    s.fracInGate = mean([acc.fracInGate]);
    s.consistent = abs(s.meanNIS-2) < 0.7 && s.fracInGate > 0.85;
    s.nSeeds     = nSeeds;
    BATCH_STORE(scenario).s = s;         %#ok<SAGROW>
    fprintf('\n>>> S%d over %d seeds: RMSE %.3f +/- %.3f m, mean NIS %.2f\n', ...
            scenario, nSeeds, s.rmse, s.rmseStd, s.meanNIS);
end

%% ----------------- COMPARISON TABLE -----------------
scenarioList = 1:numel(BATCH_STORE);   % re-declare: the scenario script clears
fprintf('\n\n');
fprintf('=======================================================================================\n');
fprintf('  GPS/IMU EKF FUSION - ALL SCENARIOS\n');
fprintf('  Requirement [8]: 0.21 m horizontal at 95%%, 0.62 m alert limit\n');
fprintf('=======================================================================================\n');
fprintf('%-40s %9s %9s %9s %9s %7s %s\n', ...
        'Scenario', 'RMSE(m)', '+/-sd', 'mean(m)', 'max(m)', 'NIS', 'valid?');
fprintf('---------------------------------------------------------------------------------------\n');

for i = scenarioList
    s = BATCH_STORE(i).s;
    if s.consistent, verdict = 'yes'; else, verdict = 'NO - RETUNE'; end
    fprintf('%-40s %9.3f %9.3f %9.3f %9.3f %7.2f %s\n', ...
            s.name, s.rmse, s.rmseStd, s.meanErr, s.maxErr, s.meanNIS, verdict);
end
fprintf('=======================================================================================\n');

%% ----------------- INTERPRETATION HELPERS -----------------
r = arrayfun(@(k) BATCH_STORE(k).s.rmse,    scenarioList);
m = arrayfun(@(k) BATCH_STORE(k).s.meanErr, scenarioList);

fprintf('\nComparisons:\n');
fprintf('  Noisy start          S1 -> S2 : %.3f -> %.3f m  (x%.1f)\n', r(1), r(2), r(2)/r(1));
fprintf('  Frequent dropouts    S1 -> S3 : %.3f -> %.3f m  (x%.1f)\n', r(1), r(3), r(3)/r(1));
fprintf('  Denied start         S1 -> S4 : %.3f -> %.3f m  (x%.1f)\n', r(1), r(4), r(4)/r(1));
fprintf('  Fix rate 1 -> 5 Hz   S2 -> S5 : %.3f -> %.3f m  (x%.1f)  <- isolates rate\n', r(2), r(5), r(5)/r(2));

% NOTE: reference [8] states its requirement at the 95th percentile
% (0.21 m at 95%, 0.62 m alert limit). RMSE is a different statistic, so
% treat the comparison below as indicative rather than like-for-like.
fprintf('\nAgainst the requirement (RMSE vs 0.21 m target, 0.62 m alert limit):\n');
for i = scenarioList
    s = BATCH_STORE(i).s;
    if s.rmse <= 0.21
        status = 'meets target';
    elseif s.rmse <= 0.62
        status = 'inside alert limit, misses target';
    else
        status = 'OUTSIDE alert limit';
    end
    fprintf('  S%d  RMSE = %6.3f m   %s\n', i, s.rmse, status);
end

%% ----------------- TRAJECTORY GRID (one panel per scenario) -----------------
% Campus-route view: raw GPS fixes, ground truth, and fused estimate.
hOv.trajectories = figure('Name','Scenario trajectories','Position',[40 40 1180 760], 'Color','w');
tl = tiledlayout(2, 3, 'TileSpacing','compact', 'Padding','compact');

for i = scenarioList
    s = BATCH_STORE(i).s;
    nexttile; hold on; box on; grid on;

    if ~isempty(s.gpsFix)
        plot(s.gpsFix(:,1), s.gpsFix(:,2), '.', 'Color',[0.35 0.80 0.35], ...
             'MarkerSize', 3, 'DisplayName','GPS signal');
    end
    plot(s.trueTrack(:,1),  s.trueTrack(:,2),  '-',  'Color',[0.15 0.15 0.15], ...
         'LineWidth', 1.6, 'DisplayName','Ground truth');
    plot(s.fusedTrack(:,1), s.fusedTrack(:,2), '-',  'Color',[0.95 0.75 0.10], ...
         'LineWidth', 1.4, 'DisplayName','EKF estimate');

    axis equal;
    xlabel('East (m)'); ylabel('North (m)');
    title(sprintf('(%c) S%d  RMSE %.2f m', 'a'+i-1, i, s.rmse), 'FontWeight','normal');
    if i == 1
        legend('Location','southeast', 'FontSize', 7, 'Box','off');
    end
end
title(tl, 'GPS/IMU EKF fusion on the campus route', 'FontWeight','bold');

%% ----------------- ERROR VS DISTANCE GRID -----------------
hOv.error_vs_distance = figure('Name','Position error along the route','Position',[70 70 1180 760], 'Color','w');
tl2 = tiledlayout(2, 3, 'TileSpacing','compact', 'Padding','compact');

ymax = max(arrayfun(@(k) BATCH_STORE(k).s.maxErr, scenarioList));
for i = scenarioList
    s = BATCH_STORE(i).s;
    nexttile; hold on; box on; grid on;
    plot(s.distAxis, s.posError, '-', 'Color',[0.20 0.40 0.80], 'LineWidth', 1.0);
    yline(0.21,'g--','LineWidth',1);
    yline(0.62,'m--','LineWidth',1);
    xlabel('Distance travelled (m)'); ylabel('Position error (m)');
    ylim([0 min(ymax*1.05, 20)]);
    title(sprintf('(%c) S%d  max %.2f m', 'a'+i-1, i, s.maxErr), 'FontWeight','normal');
end
title(tl2, 'Position error along the route (green 0.21 m target, magenta 0.62 m alert limit)', ...
      'FontWeight','bold');

%% ----------------- INJECTED GNSS UNCERTAINTY (mirrors Fig. 2b) -----------------
% Same axes as the measured figure: uncertainty against distance travelled,
% with the +/-5 m evaluation gate. Compare panel (f) against Fig. 2(b).
hOv.injected_uncertainty = figure('Name','Injected GNSS uncertainty','Position',[100 100 1180 760], 'Color','w');
tl3 = tiledlayout(2, 3, 'TileSpacing','compact', 'Padding','compact');

for i = scenarioList
    s = BATCH_STORE(i).s;
    nexttile; hold on; box on; grid on;
    plot(s.gpsDist,  s.gpsSigma, '.', 'Color',[0.20 0.45 0.75], 'MarkerSize', 5, ...
         'DisplayName','injected \sigma');
    plot(s.gpsDist,  s.gpsErr,   '.', 'Color',[0.85 0.35 0.25], 'MarkerSize', 3, ...
         'DisplayName','realised error');
    yline(5,'k--','LineWidth',1.2,'DisplayName','\pm5 m gate');
    yline(11.5,'r:','LineWidth',1.0,'DisplayName','measured peak 11.5 m');
    xlabel('Distance travelled (m)'); ylabel('Uncertainty / error (m)');
    ylim([0 25]);
    title(sprintf('(%c) S%d   %.0f%% of \\sigma > 5 m,  peak %.1f m', ...
          'a'+i-1, i, s.sigExceed, s.sigPeak), 'FontWeight','normal');
    if i == 1
        legend('Location','northeast','FontSize',7,'Box','off');
    end
end
title(tl3, 'Injected GNSS uncertainty per scenario (compare with measured Fig. 2b)', ...
      'FontWeight','bold');

%% ----------------- GNSS INPUT SUMMARY TABLE -----------------
fprintf('\n');
fprintf('=======================================================================================\n');
fprintf('  GNSS INPUT ACTUALLY INJECTED (compare against measured Fig. 2b)\n');
fprintf('=======================================================================================\n');
fprintf('%-46s %8s %8s %8s %9s %8s\n', ...
        'Scenario', 'med sig', 'peak sig', 'GPS avail', 'sig>5m', 'peak err');
fprintf('---------------------------------------------------------------------------------------\n');
for i = scenarioList
    s = BATCH_STORE(i).s;
    fprintf('%-46s %8.2f %8.2f %7.0f%% %8.0f%% %8.2f\n', ...
            s.name, s.sigMedian, s.sigPeak, s.availPct, s.sigExceed, s.errPeak);
end
fprintf('---------------------------------------------------------------------------------------\n');
fprintf('MEASURED on the route: corridor 100%% of sigma > 5 m, peak 11.5 m;\n');
fprintf('                       between buildings 56%%; open sky 0%%.\n');
fprintf('=======================================================================================\n');

%% ----------------- SUMMARY FIGURE -----------------
hOv.summary_bars = figure('Name','Scenario comparison','Position',[80 80 1000 520], 'Color','w');
b = bar(scenarioList, [r(:) m(:)], 'grouped');
b(1).DisplayName = 'RMSE';
b(2).DisplayName = 'Mean error';
hold on; grid on;
yline(0.21,'g--','0.21 m target [8]','LineWidth',1.2);
yline(0.62,'m--','0.62 m alert limit [8]','LineWidth',1.2);
set(gca,'XTick',scenarioList, 'XTickLabel', ...
    arrayfun(@(k) sprintf('S%d',k), scenarioList, 'UniformOutput', false));
ylabel('Position error (m)');
title('GPS/IMU EKF position error by scenario');
legend('Location','northwest');

%% ----------------- SAVE THE OVERVIEW FIGURES -----------------
% Saved by explicit handle rather than by searching for open figures, so the
% mapping from figure to filename cannot drift.
if exportThesisOutputs
    % Anchor output to the folder holding these scripts, not to whatever
    % MATLAB's current folder happens to be. A relative path fails whenever
    % the current folder differs or is not writable, and mkdir does not
    % raise on failure, so the error only surfaces later at exportgraphics.
    outRoot = fileparts(which('run_all_scenarios'));
    if isempty(outRoot), outRoot = pwd; end
    figDir = fullfile(outRoot, 'figures');
    if ~exist(figDir, 'dir')
        [ok, msg] = mkdir(figDir);
        if ~ok
            error('Could not create %s: %s', figDir, msg);
        end
    end

    ovFields = fieldnames(hOv);
    for i = 1:numel(ovFields)
        exportgraphics(hOv.(ovFields{i}), ...
                       fullfile(figDir, ['overview_' ovFields{i} '.png']), ...
                       'Resolution', 300);
    end
    fprintf('\nSaved %d overview figures to %s\n', numel(ovFields), figDir);
end

%% ----------------- SAVE PER-SCENARIO FIGURES AND ANALYSIS -----------------
% Writes, per scenario: trajectory, satellite overlay, error-vs-distance and
% uncertainty-vs-distance, each a standalone 300 dpi PNG carrying its own
% title, units and legend. Also writes the summary CSV, the formatted text
% table and a .mat of the full results for later re-plotting.
if exportThesisOutputs
    if exist('export_thesis_figures','file')
        fprintf('Saving per-scenario figures and analysis...\n');
        export_thesis_figures;
    else
        warning(['export_thesis_figures.m not found on the path. Overview ' ...
                 'figures were saved but per-scenario outputs were not.']);
    end
end

fprintf('\n=== ALL SCENARIOS COMPLETE ===\n');
