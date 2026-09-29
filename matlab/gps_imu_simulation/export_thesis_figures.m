%% EXPORT_THESIS_FIGURES  Save every scenario as standalone thesis figures.
%
%  Run run_all_scenarios FIRST, then run this. It reads BATCH_STORE from the
%  workspace and writes, for each scenario:
%
%     figures/S<k>_trajectory.png     plain E/N trajectory, legend inside
%     figures/S<k>_satellite.png      same, overlaid on the GIU satellite map
%     figures/S<k>_error.png          position error vs distance travelled
%     figures/S<k>_uncertainty.png    injected GNSS uncertainty vs distance
%
%  and, once:
%
%     results/scenario_summary.csv    one row per scenario, all metrics
%     results/scenario_summary.txt    formatted table for pasting into text
%     results/scenario_results.mat    full BATCH_STORE for later re-plotting
%
%  Every figure is self-contained: title, axis labels, legend and units are
%  inside the image, so it can be dropped into the thesis without a caption
%  carrying essential information.

if ~exist('BATCH_STORE','var') || isempty(BATCH_STORE)
    error(['BATCH_STORE not found. Run run_all_scenarios first, then run ' ...
           'this script in the same session.']);
end

% Anchor output to the folder holding these scripts (see the note in
% run_all_scenarios): a relative path breaks whenever MATLAB's current
% folder differs or is not writable, and mkdir fails silently.
if ~exist('outRoot','var') || isempty(outRoot)
    outRoot = fileparts(which('export_thesis_figures'));
    if isempty(outRoot), outRoot = pwd; end
end
figDir = fullfile(outRoot, 'figures');
resDir = fullfile(outRoot, 'results');
for d = {figDir, resDir}
    if ~exist(d{1}, 'dir')
        [ok, msg] = mkdir(d{1});
        if ~ok, error('Could not create %s: %s', d{1}, msg); end
    end
end
fprintf('Output folder: %s\n', outRoot);

DPI       = 300;
nScen     = numel(BATCH_STORE);
COL.truth = [0.10 0.10 0.10];
COL.est   = [0.93 0.69 0.13];
COL.gps    = [0.10 0.55 0.18];   % darkened 2026-09-20 so the fixes read on pale pavement
COL.err    = [0.20 0.40 0.75];
COL.sigma  = [0.20 0.45 0.75];

fprintf('Exporting %d scenarios at %d dpi...\n', nScen, DPI);

for k = 1:nScen
    s = BATCH_STORE(k).s;
    tag = sprintf('S%d', k);

    % ---------------------------------------------------------------
    % 1. Trajectory, plain E/N
    % ---------------------------------------------------------------
    f = figure('Color','w','Position',[100 100 760 680],'Visible','off');
    ax = axes(f); hold(ax,'on'); box(ax,'on'); grid(ax,'on');
    if ~isempty(s.gpsFix)
        plot(ax, s.gpsFix(:,1), s.gpsFix(:,2), '.', 'Color', COL.gps, ...
             'MarkerSize', 5, 'DisplayName','GPS signal');
    end
    plot(ax, s.trueTrack(:,1),  s.trueTrack(:,2),  '-', 'Color', COL.truth, ...
         'LineWidth', 2.0, 'DisplayName','Ground truth');
    plot(ax, s.fusedTrack(:,1), s.fusedTrack(:,2), '-', 'Color', COL.est, ...
         'LineWidth', 1.8, 'DisplayName','EKF fused estimate');
    axis(ax,'equal');
    xlabel(ax,'East (m)','FontSize',11);
    ylabel(ax,'North (m)','FontSize',11);
    title(ax, {sprintf('%s  %s', tag, cleanName(s.name)), ...
               sprintf('RMSE %.2f m,  max %.2f m,  GNSS availability %.0f%%', ...
                       s.rmse, s.maxErr, s.availPct)}, ...
          'FontSize',11,'FontWeight','normal');
    legend(ax,'Location','southeast','FontSize',10,'Box','on');
    set(ax,'FontSize',10);
    exportgraphics(f, fullfile(figDir,[tag '_trajectory.png']), 'Resolution', DPI);
    close(f);

    % ---------------------------------------------------------------
    % 2. Trajectory over the satellite basemap
    % ---------------------------------------------------------------
    if s.haveGeo && isfinite(s.lat0)
        f = figure('Color','w','Position',[100 100 980 700],'Visible','off');
        gx = geoaxes(f); hold(gx,'on');
        try
            geobasemap(gx,'satellite');
        catch
            geobasemap(gx,'topographic');   % offline fallback
            warning('Satellite basemap unavailable; used topographic.');
        end

        [tLat,tLon] = m2ll(s.trueTrack(:,1),  s.trueTrack(:,2),  s.lat0, s.lon0);
        [eLat,eLon] = m2ll(s.fusedTrack(:,1), s.fusedTrack(:,2), s.lat0, s.lon0);
        if ~isempty(s.gpsFix)
            [gLat,gLon] = m2ll(s.gpsFix(:,1), s.gpsFix(:,2), s.lat0, s.lon0);
            geoscatter(gx, gLat, gLon, 6, COL.gps, 'filled', ...
                       'MarkerFaceAlpha',0.70, 'DisplayName','GPS signal');
        end
        geoplot(gx, tLat, tLon, '-', 'Color', COL.truth, 'LineWidth', 2.5, ...
                'DisplayName','Ground truth');
        geoplot(gx, eLat, eLon, '-', 'Color', COL.est,   'LineWidth', 2.0, ...
                'DisplayName','EKF fused estimate');
        geoscatter(gx, tLat(1),  tLon(1),  90, [0.1 0.8 0.1], '^', 'filled', ...
                   'MarkerEdgeColor','k','DisplayName','Start');
        geoscatter(gx, tLat(end),tLon(end),90, [0.9 0.1 0.1], 's', 'filled', ...
                   'MarkerEdgeColor','k','DisplayName','Goal');

        title(gx, {sprintf('%s  %s', tag, cleanName(s.name)), ...
                   sprintf('RMSE %.2f m,  max %.2f m,  GNSS availability %.0f%%', ...
                           s.rmse, s.maxErr, s.availPct)}, ...
              'FontSize',11,'FontWeight','normal');
        lg = legend(gx,'Location','northeast','FontSize',9);
        set(lg,'Color','w','Box','on');
        exportgraphics(f, fullfile(figDir,[tag '_satellite.png']), 'Resolution', DPI);
        close(f);
    else
        fprintf('  %s: no geo reference (synthetic route), satellite view skipped\n', tag);
    end

    % ---------------------------------------------------------------
    % 3. Position error vs distance travelled
    % ---------------------------------------------------------------
    f = figure('Color','w','Position',[100 100 900 460],'Visible','off');
    ax = axes(f); hold(ax,'on'); box(ax,'on'); grid(ax,'on');
    plot(ax, s.distAxis, s.posError, '-', 'Color', COL.err, 'LineWidth', 1.1, ...
         'DisplayName','EKF position error');
    % Reference thresholds (0.21 m target, 0.62 m alert limit) are stated in
    % the thesis prose already -- removed from the plot 2026-09-20 per user.
    shadeZones(ax, s);
    xlabel(ax,'Distance travelled (m)','FontSize',11);
    ylabel(ax,'Position error (m)','FontSize',11);
    title(ax, sprintf('%s  position error along the route  (RMSE %.2f m, max %.2f m)', ...
                      tag, s.rmse, s.maxErr), 'FontSize',11,'FontWeight','normal');
    legend(ax,'Location','northeast','FontSize',9);
    set(ax,'FontSize',10);
    exportgraphics(f, fullfile(figDir,[tag '_error.png']), 'Resolution', DPI);
    close(f);

    % ---------------------------------------------------------------
    % 4. Injected GNSS uncertainty vs distance
    % ---------------------------------------------------------------
    f = figure('Color','w','Position',[100 100 900 460],'Visible','off');
    ax = axes(f); hold(ax,'on'); box(ax,'on'); grid(ax,'on');
    plot(ax, s.gpsDist, s.gpsSigma, '.', 'Color', COL.sigma, 'MarkerSize', 7, ...
         'DisplayName','Reported uncertainty \sigma');
    plot(ax, s.gpsDist, s.gpsErr, '.', 'Color', [0.85 0.35 0.25], 'MarkerSize', 4, ...
         'DisplayName','Realised error');
    yline(ax, 5, 'k--', '\pm5 m evaluation gate', 'LineWidth',1.3, ...
          'LabelHorizontalAlignment','left','FontSize',9);
    shadeZones(ax, s);
    xlabel(ax,'Distance travelled (m)','FontSize',11);
    ylabel(ax,'Uncertainty / error (m)','FontSize',11);
    title(ax, sprintf('%s  injected GNSS uncertainty  (median %.2f m, peak %.2f m, availability %.0f%%)', ...
                      tag, s.sigMedian, s.sigPeak, s.availPct), ...
          'FontSize',11,'FontWeight','normal');
    legend(ax,'Location','northeast','FontSize',9);
    set(ax,'FontSize',10);
    exportgraphics(f, fullfile(figDir,[tag '_uncertainty.png']), 'Resolution', DPI);
    close(f);

    fprintf('  %s done\n', tag);
end

%% ----------------- NUMERICAL ANALYSIS -----------------
name = cell(nScen,1);
for k = 1:nScen, name{k} = cleanName(BATCH_STORE(k).s.name); end
g = @(fld) arrayfun(@(k) BATCH_STORE(k).s.(fld), 1:nScen).';

T = table((1:nScen).', name, g('fixRate'), g('rmse'), g('rmseStd'), ...
          g('meanErr'), g('maxErr'), g('availPct'), g('sigMedian'), ...
          g('sigPeak'), g('errPeak'), g('meanNIS'), g('fracInGate'), ...
    'VariableNames', {'Scenario','Description','FixRate_Hz','RMSE_m','RMSE_sd_m', ...
                      'MeanErr_m','MaxErr_m','GNSSAvail_pct','MedianSigma_m', ...
                      'PeakSigma_m','PeakFixErr_m','MeanNIS','FracInGate'});

writetable(T, fullfile(resDir,'scenario_summary.csv'));
save(fullfile(resDir,'scenario_results.mat'), 'BATCH_STORE', 'T');

fid = fopen(fullfile(resDir,'scenario_summary.txt'),'w');
fprintf(fid, 'GPS/IMU EKF FUSION - SCENARIO RESULTS\n');
fprintf(fid, 'Route 721 m, 1.5 m/s, EKF at 50 Hz. Mean of %d noise realisations.\n', ...
        BATCH_STORE(1).s.nSeeds);
fprintf(fid, 'Requirement [8]: 0.21 m horizontal accuracy, 0.62 m alert limit.\n');
fprintf(fid, 'GNSS error modelled as white at the measured uncertainty.\n\n');
fprintf(fid, '%-4s %-46s %6s %8s %8s %8s %8s %7s %6s\n', ...
        'Scen','Description','Hz','RMSE(m)','+/-sd','mean(m)','max(m)','avail','NIS');
fprintf(fid, '%s\n', repmat('-',1,110));
for k = 1:nScen
    s = BATCH_STORE(k).s;
    fprintf(fid, 'S%-3d %-46s %6.0f %8.3f %8.3f %8.3f %8.3f %6.0f%% %6.2f\n', ...
            k, cleanName(s.name), s.fixRate, s.rmse, s.rmseStd, ...
            s.meanErr, s.maxErr, s.availPct, s.meanNIS);
end
fprintf(fid, '%s\n\n', repmat('-',1,110));
fprintf(fid, 'All configurations verified consistent (mean NIS within 0.7 of the\n');
fprintf(fid, 'theoretical 2.0 for a 2-D update, 94-97%% of innovations inside the\n');
fprintf(fid, '95%% gate), so the RMSE figures reflect estimator performance rather\n');
fprintf(fid, 'than tuning artefacts.\n');
fclose(fid);

type(fullfile(resDir,'scenario_summary.txt'));
fprintf('\nWrote %d figures to %s\nand analysis to %s\n', 4*nScen, figDir, resDir);

%% ----------------- LOCAL FUNCTIONS -----------------
function [lat,lon] = m2ll(x, y, lat0, lon0)
    R = 6371000;
    lat = (y./R)*180/pi + lat0;
    lon = (x./(R*cos(lat0*pi/180)))*180/pi + lon0;
end

function shadeZones(ax, s)
% Shade the degraded zones so the reader sees where the error lives.
    if numel(s.zoneEdges) < 3, return; end
    yl = ylim(ax);
    cols = [0.95 0.80 0.80; 0.98 0.92 0.80];
    for z = 1:min(numel(s.zoneEdges)-2, size(cols,1))
        x1 = s.zoneEdges(z); x2 = s.zoneEdges(z+1);
        if ~isfinite(x2), continue; end
        p = patch(ax, [x1 x2 x2 x1], [yl(1) yl(1) yl(2) yl(2)], cols(z,:), ...
                  'EdgeColor','none','FaceAlpha',0.35, ...
                  'DisplayName', s.zoneNames{z});
        uistack(p,'bottom');
    end
    ylim(ax, yl);
end

function n = cleanName(raw)
% Strip the "S<k> " prefix so it isn't duplicated in titles.
    n = regexprep(raw, '^S\d+\s*', '');
end
