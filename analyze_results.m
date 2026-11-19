function S = analyze_results(R, P)
%ANALYZE_RESULTS  Statistics, figures, CSV tables, TOPSIS ranking and an
%   auto-generated key-findings report from a benchmark run.
%
%   analyze_results              % loads results/benchmark_results.mat
%   analyze_results(R)           % use a result struct from run_benchmark
%   S = analyze_results(R, P)    % returns all summary numbers
%
%   Outputs (results/):
%     figures/fig01 ... fig10  PNG figures for the report
%     summary_by_environment.csv, summary_overall.csv, topsis_rankings.csv,
%     complexity_by_environment.csv, all_trials.csv, key_findings.txt

setup_paths();
if nargin < 1 || isempty(R)
    P0 = default_params();
    f = fullfile(P0.resultsDir, 'benchmark_results.mat');
    if ~exist(f, 'file'), error('No results found. Run run_benchmark first.'); end
    tmp = load(f);  R = tmp.R;
end
if nargin < 2 || isempty(P), P = R.P; end
if ~exist(P.figDir, 'dir'), mkdir(P.figDir); end

[nE, nT, nP] = size(R.success);
names = R.names;  envN = R.envNames;  col = R.colors;

%% ===================== 1. Aggregate statistics =====================
minLen = min(R.length, [], 3);                       % best path per trial
relLen = bsxfun(@rdivide, R.length, minLen);         % 1.00 = best found

S.success   = reshape(mean(R.success, 2), nE, nP);
S.timeMed   = agg(R.time, @nmedian) * 1000;          % ms
S.timeMean  = agg(R.time, @nmean) * 1000;
S.timeStd   = agg(R.time, @nstd) * 1000;
S.relLen    = agg(relLen, @nmean);
S.relLenStd = agg(relLen, @nstd);
S.length    = agg(R.length, @nmean);
S.turnPL    = agg(R.turnPerLen, @nmean) * 100;       % rad per 100 cells
S.turning   = agg(R.turning, @nmean);
S.vertices  = agg(R.vertices, @nmean);
S.minClear  = agg(R.minClear, @nmean);
S.meanClear = agg(R.meanClear, @nmean);
S.nodes     = agg(R.nodes, @nmean);

O.success   = reshape(mean(reshape(R.success, [], nP), 1), 1, nP);
O.timeMed   = aggAll(R.time, @nmedian) * 1000;
O.timeMean  = aggAll(R.time, @nmean) * 1000;
O.relLen    = aggAll(relLen, @nmean);
O.turnPL    = aggAll(R.turnPerLen, @nmean) * 100;
O.vertices  = aggAll(R.vertices, @nmean);
O.minClear  = aggAll(R.minClear, @nmean);
O.meanClear = aggAll(R.meanClear, @nmean);
O.nodes     = aggAll(R.nodes, @nmean);
S.overall = O;

% ---- Failure-penalised metrics (survivorship-bias correction) ----
% Averaging path quality over successful runs only flatters unreliable
% planners: they "succeed" mostly on easy scenarios. For ranking, every
% FAILED query is therefore scored with the worst value of that criterion
% observed in the same environment (PAR-style penalty, as in solver
% benchmarking).
penRel = relLen;  penTurn = R.turnPerLen;  penClr = R.meanClear;
for e = 1:nE
    F = ~R.success(e, :, :);
    a = penRel(e, :, :);   v = a(~F);  if isempty(v), v = 2; end
    a(F) = max(v);         penRel(e, :, :) = a;
    a = penTurn(e, :, :);  v = a(~F);  if isempty(v), v = 1; end
    a(F) = max(v);         penTurn(e, :, :) = a;
    a = penClr(e, :, :);   v = a(~F);  if isempty(v), v = 0; end
    a(F) = min(v);         penClr(e, :, :) = a;
end
S.pen.relLen    = agg(penRel, @nmean);
S.pen.turnPL    = agg(penTurn, @nmean) * 100;
S.pen.meanClear = agg(penClr, @nmean);
O.pen.relLen    = aggAll(penRel, @nmean);
O.pen.turnPL    = aggAll(penTurn, @nmean) * 100;
O.pen.meanClear = aggAll(penClr, @nmean);
S.overall = O;
reliable = O.success >= 0.90;                 % for "best-of" statements
if ~any(reliable), reliable(:) = true; end

%% ===================== 2. TOPSIS multi-criteria ranking =====================
critNames = {'Success rate', 'Planning time', 'Path length', 'Smoothness', ...
             'Safety clearance', 'Search effort'};
isBen = logical([1 0 0 0 1 0]);
profiles = struct( ...
    'name',   {'Balanced', 'Real-time', 'Safety-critical', 'Path-quality'}, ...
    'desc',   {'equal weights', 'fast replanning on embedded CPU', ...
               'robot near people / fragile goods', 'energy-efficient, smooth motion'}, ...
    'w',      {[1 1 1 1 1 1]/6, [0.25 0.40 0.10 0.05 0.05 0.15], ...
               [0.30 0.05 0.10 0.15 0.35 0.05], [0.20 0.05 0.35 0.30 0.05 0.05]});
nPr = numel(profiles);

Xall = clean_matrix([O.success' O.timeMed' O.pen.relLen' O.pen.turnPL' ...
                     O.pen.meanClear' O.nodes'], isBen);
S.topsis.overallC = zeros(nPr, nP);  S.topsis.overallRank = zeros(nPr, nP);
for k = 1:nPr
    [c, r] = topsis_rank(Xall, profiles(k).w, isBen);
    S.topsis.overallC(k, :) = c';  S.topsis.overallRank(k, :) = r';
end
S.topsis.envC = zeros(nE, nP);  S.topsis.envRank = zeros(nE, nP);
for e = 1:nE
    Xe = clean_matrix([S.success(e,:)' S.timeMed(e,:)' S.pen.relLen(e,:)' ...
                       S.pen.turnPL(e,:)' S.pen.meanClear(e,:)' S.nodes(e,:)'], isBen);
    [c, r] = topsis_rank(Xe, profiles(1).w, isBen);
    S.topsis.envC(e, :) = c';  S.topsis.envRank(e, :) = r';
end
S.topsis.profiles = profiles;  S.topsis.criteria = critNames;

% weight-sensitivity sweeps (rank-reversal analysis)
sweep = linspace(0, 0.8, 17);
S.sens.sweep = sweep;
S.sens.time  = sens_sweep(Xall, isBen, 2, sweep);
S.sens.clear = sens_sweep(Xall, isBen, 5, sweep);

%% ===================== 3. Environment difficulty & robustness =====================
tort = R.cx.tortuosity;  tort(isnan(tort)) = max(tort(:));
z = @(a) (a - mean(a(:))) / max(std(a(:)), eps);
% difficulty index: mean z-score of four descriptors (narrower bottleneck = harder)
D = (z(R.cx.density) + z(R.cx.narrowness) + z(tort) - z(R.cx.bottleneck)) / 4;
[~, ord] = sort(D(:));
nBin = 4;
bin = zeros(numel(D), 1);
edges = round(linspace(0, numel(D), nBin + 1));
for b = 1:nBin, bin(ord(edges(b) + 1:edges(b + 1))) = b; end
S.robust.succ = zeros(nBin, nP);  S.robust.rel = zeros(nBin, nP);
succ2 = reshape(R.success, nE * nT, nP);  rel2 = reshape(relLen, nE * nT, nP);
for b = 1:nBin
    m = bin == b;
    S.robust.succ(b, :) = mean(succ2(m, :), 1);
    for p = 1:nP, S.robust.rel(b, p) = nmean(rel2(m, p)); end
end
S.difficulty = D;

%% ===================== 4. Figures =====================

% ---- fig01: environment gallery ----
f = new_fig(P, 'Environments', [1400 900]);
for e = 1:nE
    ax = subplot(2, 3, e);
    ex = R.examples{e};
    plot_map(ax, ex.raw, ex.infl, ex.s, ex.g);
    title(ax, {envN{e}, sprintf('density %.0f%%  narrowness %.0f%%', ...
        100 * nmean(R.cx.density(e, :)), 100 * nmean(R.cx.narrowness(e, :)))});
end
ax = subplot(2, 3, 6);  axis(ax, 'off');
txt = {'\bfEnvironment complexity (mean over trials)', '', ...
       sprintf('%-17s %6s %7s %6s %6s', 'Class', 'Dens.', 'Narrow', 'Tort.', 'Bottl.')};
for e = 1:nE
    txt{end + 1} = sprintf('%-17s %5.0f%% %6.0f%% %6.2f %6.2f', envN{e}, ...
        100 * nmean(R.cx.density(e, :)), 100 * nmean(R.cx.narrowness(e, :)), ...
        nmean(R.cx.tortuosity(e, :)), nmean(R.cx.bottleneck(e, :))); %#ok<AGROW>
end
txt = [txt, {'', 'dark = obstacle, grey = inflation (robot radius)', ...
             'green = start, red star = goal'}];
text(ax, 0, 0.5, txt, 'FontName', 'Courier', 'FontSize', 9, 'VerticalAlignment', 'middle');
add_suptitle(f, 'Benchmark environment classes (one example of each)');
save_fig(f, P, 'fig01_environments.png');

% ---- fig02: path overlays ----
f = new_fig(P, 'Path overlays', [1400 900]);
for e = 1:nE
    ax = subplot(2, 3, e);
    ex = R.examples{e};
    plot_map(ax, ex.raw, ex.infl);
    for p = 1:nP
        if ~isempty(ex.paths{p})
            plot(ax, ex.paths{p}(:, 1), ex.paths{p}(:, 2), '-', 'Color', col(p, :), 'LineWidth', 1.8);
        elseif ~isempty(ex.partial{p})
            pp = ex.partial{p};
            plot(ax, pp(:, 1), pp(:, 2), '--', 'Color', col(p, :), 'LineWidth', 1.5);
            plot(ax, pp(end, 1), pp(end, 2), 'x', 'Color', col(p, :), 'MarkerSize', 12, 'LineWidth', 2.5);
        end
    end
    plot(ax, ex.s(1), ex.s(2), 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.1 0.75 0.2], 'MarkerEdgeColor', 'k');
    plot(ax, ex.g(1), ex.g(2), 'p', 'MarkerSize', 13, 'MarkerFaceColor', [0.9 0.1 0.1], 'MarkerEdgeColor', 'k');
    title(ax, envN{e});
end
ax = subplot(2, 3, 6);  hold(ax, 'on');
h = zeros(1, nP);
for p = 1:nP, h(p) = plot(ax, NaN, NaN, '-', 'Color', col(p, :), 'LineWidth', 3); end
axis(ax, 'off');
legend(ax, h, names, 'Location', 'west', 'FontSize', 11);
text(ax, 0.05, 0.05, 'dashed + x = APF stuck in local minimum', 'Units', 'normalized', 'FontSize', 9);
add_suptitle(f, 'Paths found by each planner (trial 1 of every environment)');
save_fig(f, P, 'fig02_path_overlays.png');

% ---- fig03: success rate ----
f = new_fig(P, 'Success rate', [1300 560]);
ax = axes('Parent', f);
grouped_bar(ax, 100 * S.success, col, envN, 'Success rate (%)');
ylim(ax, [0 109]);
legend(ax, names, 'Location', 'eastoutside');
title(ax, sprintf('Success rate per environment (%d trials each)', nT));
save_fig(f, P, 'fig03_success_rate.png');

% ---- fig04: performance metrics ----
f = new_fig(P, 'Metrics', [1500 950]);
ax = subplot(2, 2, 1);
grouped_bar(ax, S.timeMed, col, envN, 'Median planning time (ms, log)');
set(ax, 'YScale', 'log');  title(ax, '(a) Computation time');
ax = subplot(2, 2, 2);
grouped_bar(ax, S.relLen, col, envN, 'Length / best length');
yl = [min(S.relLen(:)) max(S.relLen(:))];
ylim(ax, [max(0.9, yl(1) - 0.05) yl(2) + 0.05]);  title(ax, '(b) Path optimality (1.00 = shortest found)');
ax = subplot(2, 2, 3);
grouped_bar(ax, S.turnPL, col, envN, 'Heading change (rad / 100 cells)');
title(ax, '(c) Smoothness (lower = smoother)');
ax = subplot(2, 2, 4);
grouped_bar(ax, S.meanClear, col, envN, 'Mean clearance (cells)');
yl = [min(S.meanClear(:)) max(S.meanClear(:))];
ylim(ax, [max(0, yl(1) - 0.5) yl(2) + 0.3]);  title(ax, '(d) Safety margin to true obstacles');
legend(ax, names, 'Location', 'eastoutside');
add_suptitle(f, 'Performance metrics over successful runs');
save_fig(f, P, 'fig04_performance_metrics.png');

% ---- fig05: time vs path-quality trade-off ----
f = new_fig(P, 'Trade-off', [1000 720]);
ax = axes('Parent', f);  hold(ax, 'on');  box(ax, 'on');  grid(ax, 'on');
h = zeros(1, nP);
for p = 1:nP
    plot(ax, S.timeMed(:, p), S.relLen(:, p), '.', 'Color', lighten(col(p, :), 0.3), 'MarkerSize', 14);
    h(p) = plot(ax, O.timeMed(p), O.relLen(p), 'o', 'MarkerSize', 6 + 14 * O.success(p), ...
        'MarkerFaceColor', col(p, :), 'MarkerEdgeColor', 'k');
    text(ax, O.timeMed(p) * 1.12, O.relLen(p), names{p}, 'FontSize', 10, 'FontWeight', 'bold');
end
set(ax, 'XScale', 'log');
xlabel(ax, 'Median planning time (ms, log scale)');
ylabel(ax, 'Mean path length / best length');
title(ax, {'Speed vs path-quality trade-off', ...
    'big marker = overall (size ~ success rate), small dots = per environment'});
legend(ax, h, names, 'Location', 'northeast');
save_fig(f, P, 'fig05_tradeoff_time_vs_length.png');

% ---- fig06: radar profiles ----
Sc = norm_scores(Xall, isBen, [2 6]);        % 0..1, 1 = best; log for time & nodes
f = new_fig(P, 'Radar', [1500 820]);
for p = 1:nP
    ax = subplot(2, 4, p);
    radar(ax, Sc(p, :), {'Success', 'Speed', 'Length', 'Smooth', 'Safety', 'Effort'}, col(p, :));
    title(ax, {names{p}, ['(' R.families{p} ')']}, 'FontSize', 10);
end
add_suptitle(f, 'Normalised capability profile of each planner (outer ring = best in class)');
save_fig(f, P, 'fig06_radar_profiles.png');

% ---- fig07: TOPSIS heatmap per environment ----
f = new_fig(P, 'TOPSIS heatmap', [1250 600]);
ax = axes('Parent', f);
Cm = [S.topsis.envC; S.topsis.overallC(1, :)];
Rm = [S.topsis.envRank; S.topsis.overallRank(1, :)];
imagesc(ax, Cm);  caxis(ax, [0 1]);
try, colormap(ax, blue_map()); catch, colormap(blue_map()); end
colorbar(ax);
for i = 1:size(Cm, 1)
    for j = 1:nP
        clrT = 'k'; if Cm(i, j) > 0.6, clrT = 'w'; end
        text(ax, j, i - 0.15, sprintf('%.2f', Cm(i, j)), 'HorizontalAlignment', 'center', ...
            'FontSize', 9, 'Color', clrT);
        text(ax, j, i + 0.18, sprintf('#%d', Rm(i, j)), 'HorizontalAlignment', 'center', ...
            'FontSize', 10, 'Color', clrT, 'FontWeight', 'bold');
    end
end
set(ax, 'XTick', 1:nP, 'XTickLabel', names, 'YTick', 1:nE + 1, 'YTickLabel', [envN, {'OVERALL'}]);
title(ax, 'TOPSIS closeness to the ideal planner (balanced weights)  -  #rank, 1 = best');
save_fig(f, P, 'fig07_topsis_heatmap.png');

% ---- fig08: TOPSIS application profiles ----
f = new_fig(P, 'TOPSIS profiles', [1400 620]);
ax = axes('Parent', f);
grouped_bar(ax, S.topsis.overallC, col, {profiles.name}, 'TOPSIS closeness (higher = better)');
ylim(ax, [0 1.08]);
legend(ax, names, 'Location', 'eastoutside');
for k = 1:nPr
    [~, b] = max(S.topsis.overallC(k, :));
    text(ax, k, 1.03, ['Best: ' names{b}], 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end
title(ax, 'Which planner should I use? Ranking under four application weight profiles');
save_fig(f, P, 'fig08_topsis_profiles.png');

% ---- fig09: sensitivity ----
f = new_fig(P, 'Sensitivity', [1400 560]);
ax = subplot(1, 2, 1);
sens_plot(ax, sweep, S.sens.time, col, 'Weight on planning time');
title(ax, '(a) Rank reversal as speed matters more');
ax = subplot(1, 2, 2);
sens_plot(ax, sweep, S.sens.clear, col, 'Weight on safety clearance');
title(ax, '(b) Rank reversal as safety matters more');
legend(ax, names, 'Location', 'eastoutside');
add_suptitle(f, 'TOPSIS weight-sensitivity analysis (other criteria share the remaining weight)');
save_fig(f, P, 'fig09_topsis_sensitivity.png');

% ---- fig10: robustness vs difficulty ----
f = new_fig(P, 'Robustness', [1400 560]);
binLbl = {'Q1 easy', 'Q2', 'Q3', 'Q4 hard'};
ax = subplot(1, 2, 1);  hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
for p = 1:nP
    plot(ax, 1:nBin, 100 * S.robust.succ(:, p), '-o', 'Color', col(p, :), ...
        'MarkerFaceColor', col(p, :), 'LineWidth', 1.8);
end
set(ax, 'XTick', 1:nBin, 'XTickLabel', binLbl);  xlim(ax, [0.8 nBin + 0.2]);  ylim(ax, [0 105]);
ylabel(ax, 'Success rate (%)');  xlabel(ax, 'Scenario difficulty quartile');
title(ax, '(a) Reliability vs difficulty');
ax = subplot(1, 2, 2);  hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
for p = 1:nP
    plot(ax, 1:nBin, S.robust.rel(:, p), '-o', 'Color', col(p, :), ...
        'MarkerFaceColor', col(p, :), 'LineWidth', 1.8);
end
set(ax, 'XTick', 1:nBin, 'XTickLabel', binLbl);  xlim(ax, [0.8 nBin + 0.2]);
ylabel(ax, 'Path length / best length');  xlabel(ax, 'Scenario difficulty quartile');
title(ax, '(b) Path optimality vs difficulty');
legend(ax, names, 'Location', 'eastoutside');
add_suptitle(f, 'Robustness curves  (difficulty = z-scores of density, narrowness, tortuosity, bottleneck width)');
save_fig(f, P, 'fig10_robustness_vs_difficulty.png');

%% ===================== 5. CSV tables =====================
hdr = {'Environment', 'Planner', 'Family', 'SuccessRate_pct', 'TimeMedian_ms', 'TimeMean_ms', ...
       'TimeStd_ms', 'RelLength', 'RelLengthStd', 'Length_cells', 'Turning_rad', ...
       'TurnPer100cells', 'Vertices', 'MinClearance', 'MeanClearance', 'Nodes', 'TOPSIS_balanced', 'Rank'};
rows = {};
for e = 1:nE
    for p = 1:nP
        rows(end + 1, :) = {envN{e}, names{p}, R.families{p}, 100 * S.success(e, p), ...
            S.timeMed(e, p), S.timeMean(e, p), S.timeStd(e, p), S.relLen(e, p), ...
            S.relLenStd(e, p), S.length(e, p), S.turning(e, p), S.turnPL(e, p), ...
            S.vertices(e, p), S.minClear(e, p), S.meanClear(e, p), S.nodes(e, p), ...
            S.topsis.envC(e, p), S.topsis.envRank(e, p)}; %#ok<AGROW>
    end
end
write_csv(fullfile(P.resultsDir, 'summary_by_environment.csv'), hdr, rows);

hdr = {'Planner', 'Family', 'SuccessRate_pct', 'TimeMedian_ms', 'TimeMean_ms', 'RelLength', ...
       'TurnPer100cells', 'Vertices', 'MinClearance', 'MeanClearance', 'Nodes'};
rows = cell(nP, numel(hdr));
for p = 1:nP
    rows(p, :) = {names{p}, R.families{p}, 100 * O.success(p), O.timeMed(p), O.timeMean(p), ...
        O.relLen(p), O.turnPL(p), O.vertices(p), O.minClear(p), O.meanClear(p), O.nodes(p)};
end
write_csv(fullfile(P.resultsDir, 'summary_overall.csv'), hdr, rows);

hdr = [{'Profile', 'Description'}, strcat(names, '_C'), strcat(names, '_rank')];
rows = cell(nPr, numel(hdr));
for k = 1:nPr
    rows(k, :) = [{profiles(k).name, profiles(k).desc}, num2cell(S.topsis.overallC(k, :)), ...
                  num2cell(S.topsis.overallRank(k, :))];
end
write_csv(fullfile(P.resultsDir, 'topsis_rankings.csv'), hdr, rows);

hdr = {'Environment', 'ObstacleDensity', 'Narrowness', 'MeanClearance', 'Tortuosity', ...
       'BottleneckClearance', 'StartGoalDist'};
rows = cell(nE, numel(hdr));
for e = 1:nE
    rows(e, :) = {envN{e}, nmean(R.cx.density(e, :)), nmean(R.cx.narrowness(e, :)), ...
        nmean(R.cx.meanClearance(e, :)), nmean(R.cx.tortuosity(e, :)), ...
        nmean(R.cx.bottleneck(e, :)), nmean(R.cx.euclid(e, :))};
end
write_csv(fullfile(P.resultsDir, 'complexity_by_environment.csv'), hdr, rows);

hdr = {'Environment', 'Trial', 'Planner', 'Success', 'Time_ms', 'Length', 'RelLength', ...
       'Turning_rad', 'Vertices', 'MinClearance', 'MeanClearance', 'Nodes', 'Difficulty'};
rows = cell(nE * nT * nP, numel(hdr));  i = 0;
for e = 1:nE
    for k = 1:nT
        for p = 1:nP
            i = i + 1;
            rows(i, :) = {envN{e}, k, names{p}, R.success(e, k, p), 1000 * R.time(e, k, p), ...
                R.length(e, k, p), relLen(e, k, p), R.turning(e, k, p), R.vertices(e, k, p), ...
                R.minClear(e, k, p), R.meanClear(e, k, p), R.nodes(e, k, p), D(e, k)};
        end
    end
end
write_csv(fullfile(P.resultsDir, 'all_trials.csv'), hdr, rows);

%% ===================== 6. Console summary + key findings =====================
L = {};
L{end + 1} = sprintf('GLOBAL PATH PLANNING BENCHMARK - KEY FINDINGS');
L{end + 1} = sprintf('%d planners x %d environment classes x %d trials = %d planning queries', ...
    nP, nE, nT, nP * nE * nT);
L{end + 1} = '';
L{end + 1} = sprintf('%-12s %8s %10s %8s %10s %9s %8s %9s', 'Planner', 'Success', ...
    'Time(ms)', 'RelLen', 'Turn/100', 'Vertices', 'MeanClr', 'Nodes');
L{end + 1} = repmat('-', 1, 82);
for p = 1:nP
    L{end + 1} = sprintf('%-12s %7.1f%% %10.2f %8.3f %10.2f %9.1f %8.2f %9.0f', names{p}, ...
        100 * O.success(p), O.timeMed(p), O.relLen(p), O.turnPL(p), O.vertices(p), ...
        O.meanClear(p), O.nodes(p)); %#ok<AGROW>
end
L{end + 1} = '(Time and nodes over all queries; path-quality columns over successful queries.)';
L{end + 1} = '';
rel = find(reliable);
L{end + 1} = sprintf('Reliable planners (>= 90%% success): %s', strjoin(names(rel), ', '));
unrel = find(~reliable);
if ~isempty(unrel)
    L{end + 1} = sprintf(['Unreliable planners: %s - their path-quality averages come mostly ', ...
        'from easy scenarios (survivorship bias), so TOPSIS uses failure-penalised values.'], ...
        strjoin(names(unrel), ', '));
end
L{end + 1} = '';
L{end + 1} = 'Best among reliable planners:';
[~, a] = min(O.timeMed(rel));   a = rel(a);  L{end + 1} = sprintf('* Fastest (median time): %s (%.2f ms)', names{a}, O.timeMed(a));
[~, a] = min(O.relLen(rel));    a = rel(a);  L{end + 1} = sprintf('* Shortest paths: %s (%.1f%% above best on average)', names{a}, 100 * (O.relLen(a) - 1));
[~, a] = min(O.turnPL(rel));    a = rel(a);  L{end + 1} = sprintf('* Smoothest paths: %s (%.2f rad / 100 cells)', names{a}, O.turnPL(a));
[~, a] = max(O.meanClear(rel)); a = rel(a);  L{end + 1} = sprintf('* Largest safety margin: %s (%.2f cells mean clearance)', names{a}, O.meanClear(a));
[~, a] = min(O.nodes(rel));     a = rel(a);  L{end + 1} = sprintf('* Least search effort: %s (%.0f nodes)', names{a}, O.nodes(a));
ia = find(strcmp(R.shorts, 'AST'));  id = find(strcmp(R.shorts, 'DIJ'));
L{end + 1} = sprintf('* A* expands %.0f%% fewer nodes than Dijkstra for the same optimal grid path', ...
    100 * (1 - O.nodes(ia) / O.nodes(id)));
it = find(strcmp(R.shorts, 'THS'));
L{end + 1} = sprintf('* Theta* (any-angle) paths are %.1f%% shorter than A* and turn %.0f%% less', ...
    100 * (1 - O.relLen(it) / O.relLen(ia)), 100 * (1 - O.turnPL(it) / O.turnPL(ia)));
L{end + 1} = '';
L{end + 1} = 'Best planner per environment (TOPSIS, balanced weights):';
for e = 1:nE
    [~, b] = min(S.topsis.envRank(e, :));
    [~, w] = max(S.topsis.envRank(e, :));
    L{end + 1} = sprintf('  %-18s best: %-12s worst: %s', envN{e}, names{b}, names{w}); %#ok<AGROW>
end
L{end + 1} = '';
L{end + 1} = 'Best planner per application profile (TOPSIS, all environments):';
for k = 1:nPr
    [~, o] = sort(S.topsis.overallRank(k, :));
    L{end + 1} = sprintf('  %-16s (%s): 1. %s  2. %s  3. %s', profiles(k).name, ...
        profiles(k).desc, names{o(1)}, names{o(2)}, names{o(3)}); %#ok<AGROW>
end
L{end + 1} = '';
drop = 100 * (S.robust.succ(1, :) - S.robust.succ(end, :));
[~, a] = max(drop);
L{end + 1} = sprintf('* Most difficulty-sensitive: %s (success drops %.0f points from easiest to hardest quartile)', ...
    names{a}, drop(a));
stable = names(all(S.robust.succ > 0.999, 1));
if ~isempty(stable)
    L{end + 1} = sprintf('* Fully robust (100%% in every quartile): %s', strjoin(stable, ', '));
end
fid = fopen(fullfile(P.resultsDir, 'key_findings.txt'), 'w');
fprintf(fid, '%s\n', L{:});
fclose(fid);
fprintf('\n');
fprintf('%s\n', L{:});
fprintf('\nFigures and tables written to %s\n', P.resultsDir);
S.findings = L;
end

%% ===================== local helpers =====================
function A = agg(X, fn)
[nE, ~, nP] = size(X);
A = nan(nE, nP);
for e = 1:nE
    for p = 1:nP
        A(e, p) = fn(X(e, :, p));
    end
end
end

function a = aggAll(X, fn)
nP = size(X, 3);
a = nan(1, nP);
for p = 1:nP
    v = X(:, :, p);
    a(p) = fn(v(:));
end
end

function X = clean_matrix(X, isBen)
% replace NaN (planner never succeeded) by the worst value of that criterion
for j = 1:size(X, 2)
    v = X(:, j);
    if all(isnan(v)), X(:, j) = 1; continue; end
    if isBen(j), worst = min(v(~isnan(v))); else, worst = max(v(~isnan(v))); end
    v(isnan(v)) = worst;
    X(:, j) = v;
end
X(X <= 0) = 1e-6;
end

function C = sens_sweep(X, isBen, j, sweep)
nC = size(X, 2);
C = zeros(numel(sweep), size(X, 1));
for k = 1:numel(sweep)
    w = ones(1, nC) * (1 - sweep(k)) / (nC - 1);
    w(j) = sweep(k);
    C(k, :) = topsis_rank(X, w, isBen)';
end
end

function sens_plot(ax, sweep, C, col, xl)
hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
for p = 1:size(C, 2)
    plot(ax, sweep, C(:, p), '-', 'Color', col(p, :), 'LineWidth', 2);
end
xline_compat(ax, 1/6);
xlabel(ax, xl);  ylabel(ax, 'TOPSIS closeness');  ylim(ax, [0 1]);
end

function xline_compat(ax, x)
yl = [0 1];
plot(ax, [x x], yl, 'k:', 'LineWidth', 1);
text(ax, x, 0.97, ' balanced', 'FontSize', 8);
end

function Sc = norm_scores(X, isBen, logCols)
X(:, logCols) = log10(X(:, logCols));
mn = min(X, [], 1);  mx = max(X, [], 1);
rg = mx - mn;  rg(rg == 0) = 1;
Sc = bsxfun(@rdivide, bsxfun(@minus, X, mn), rg);
Sc(:, ~isBen) = 1 - Sc(:, ~isBen);
Sc = 0.08 + 0.92 * Sc;             % keep a visible minimum
end

function radar(ax, v, lbl, c)
n = numel(v);
ang = pi/2 - (0:n) * 2 * pi / n;
hold(ax, 'on');
for r = 0.25:0.25:1
    plot(ax, r * cos(ang), r * sin(ang), '-', 'Color', [0.85 0.85 0.85]);
end
for k = 1:n
    plot(ax, [0 cos(ang(k))], [0 sin(ang(k))], '-', 'Color', [0.85 0.85 0.85]);
    text(ax, 1.22 * cos(ang(k)), 1.18 * sin(ang(k)), lbl{k}, ...
        'HorizontalAlignment', 'center', 'FontSize', 8);
end
vv = [v v(1)];
patch(ax, vv .* cos(ang), vv .* sin(ang), lighten(c, 0.55), 'EdgeColor', c, 'LineWidth', 2);
axis(ax, 'equal');  axis(ax, 'off');
xlim(ax, [-1.45 1.45]);  ylim(ax, [-1.35 1.35]);
end

function cmap = blue_map()
t = linspace(0, 1, 64)';
cmap = [1 - 0.9 * t, 1 - 0.6 * t, 1 - 0.25 * t];
end
