function out = visualize_scenario(raw, infl, s, g, P, tag)
%VISUALIZE_SCENARIO  Run all 8 planners on one scenario, print a comparison
%   table and draw (1) a 2x4 panel figure showing how each planner searched
%   and (2) an overlay of all final paths.
if nargin < 6, tag = 'scenario'; end
reg = planner_registry();
nP = numel(reg);
clr = clearance_map(raw);
res = cell(1, nP);  tm = zeros(1, nP);  mt = cell(1, nP);

if P.warmup                                   % untimed warm-up (JIT)
    for p = 1:nP, reg(p).fn(infl, s, g, P); end
end
for p = 1:nP
    set_seed(P.baseSeed + p);
    t0 = tic;
    res{p} = reg(p).fn(infl, s, g, P);
    tm(p) = toc(t0);
    mt{p} = path_metrics(res{p}.path, clr);
end

% ---------- console table ----------
fprintf('\n%s  (start [%d %d] -> goal [%d %d])\n', upper(tag), s, g);
fprintf('%-12s %-8s %10s %8s %10s %9s %8s %7s\n', 'Planner', 'Success', ...
    'Time(ms)', 'Length', 'Turn(rad)', 'Vertices', 'MeanClr', 'Nodes');
fprintf('%s\n', repmat('-', 1, 80));
for p = 1:nP
    m = mt{p};
    fprintf('%-12s %-8s %10.1f %8.1f %10.2f %9d %8.2f %7d\n', reg(p).name, ...
        yesno(res{p}.success), 1000 * tm(p), m.length, m.turning, ...
        max(0, round(m.vertices)), m.meanClear, res{p}.nodes);
end

% ---------- 2x4 panels ----------
f1 = new_fig(P, ['Planners - ' tag], [1500 800]);
for p = 1:nP
    ax = subplot(2, 4, p);
    plot_planner_panel(ax, raw, infl, s, g, res{p}, reg(p), mt{p}, tm(p));
end
add_suptitle(f1, sprintf('Global path planners - %s  (explored region + final path)', tag));
save_fig(f1, P, sprintf('demo_%s_panels.png', regexprep(lower(tag), '\W+', '_')));

% ---------- overlay ----------
f2 = new_fig(P, ['Overlay - ' tag], [820 780]);
ax = axes('Parent', f2);
plot_map(ax, raw, infl);
h = [];  lbl = {};
for p = 1:nP
    if res{p}.success
        h(end + 1) = plot(ax, res{p}.path(:, 1), res{p}.path(:, 2), '-', ...
            'Color', reg(p).color, 'LineWidth', 2); %#ok<AGROW>
        lbl{end + 1} = sprintf('%s (%.1f)', reg(p).name, mt{p}.length); %#ok<AGROW>
    end
end
plot(ax, s(1), s(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.1 0.75 0.2], 'MarkerEdgeColor', 'k');
plot(ax, g(1), g(2), 'p', 'MarkerSize', 14, 'MarkerFaceColor', [0.9 0.1 0.1], 'MarkerEdgeColor', 'k');
if ~isempty(h)
    try
        legend(ax, h, lbl, 'Location', 'southoutside', 'NumColumns', 3);
    catch
        try
            legend(ax, h, lbl, 'Location', 'northeast');
        catch
        end
    end
end
title(ax, sprintf('All successful paths - %s', tag));
save_fig(f2, P, sprintf('demo_%s_overlay.png', regexprep(lower(tag), '\W+', '_')));

out.res = res;  out.time = tm;  out.metrics = mt;
end

function s = yesno(b)
if b, s = 'yes'; else, s = 'NO'; end
end
