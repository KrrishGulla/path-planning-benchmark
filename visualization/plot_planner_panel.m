function plot_planner_panel(ax, raw, infl, s, g, res, info, metric, tSec)
%PLOT_PLANNER_PANEL  One planner's result: what it explored + the final path.
%   Graph planners : explored (closed) cells as light dots
%   PRM / RRT / RRT*: roadmap or tree edges as thin lines
%   APF            : equipotential contours; on failure, the walked path and
%                    the local minimum where it got stuck
col = info.color;
plot_map(ax, raw, infl);

if strcmp(info.short, 'APF') && isfield(res.extra, 'U')
    if ~exist('OCTAVE_VERSION', 'builtin')        % equipotential lines (MATLAB)
        try
            contour(ax, res.extra.U, 18, 'LineColor', lighten(col, 0.55));
        catch
        end
    end
    plot(ax, res.explored(:, 1), res.explored(:, 2), '.', ...
        'Color', lighten(col, 0.3), 'MarkerSize', 5);
elseif ~isempty(res.edges)
    E = res.edges;
    X = [E(:, 1) E(:, 3) nan(size(E, 1), 1)]';
    Y = [E(:, 2) E(:, 4) nan(size(E, 1), 1)]';
    plot(ax, X(:), Y(:), '-', 'Color', lighten(col, 0.55), 'LineWidth', 0.5);
    plot(ax, res.explored(:, 1), res.explored(:, 2), '.', ...
        'Color', lighten(col, 0.3), 'MarkerSize', 4);
elseif ~isempty(res.explored)
    plot(ax, res.explored(:, 1), res.explored(:, 2), 's', 'MarkerSize', 2, ...
        'MarkerFaceColor', lighten(col, 0.6), 'MarkerEdgeColor', 'none');
end

if res.success
    plot(ax, res.path(:, 1), res.path(:, 2), '-', 'Color', col, 'LineWidth', 2.4);
    ttl = {info.name, sprintf('L = %.1f   t = %.1f ms   nodes = %d', ...
        metric.length, 1000 * tSec, res.nodes)};
else
    if isfield(res.extra, 'partialPath')
        pp = res.extra.partialPath;
        plot(ax, pp(:, 1), pp(:, 2), '--', 'Color', col, 'LineWidth', 2);
        plot(ax, pp(end, 1), pp(end, 2), 'x', 'Color', [0.85 0 0], ...
            'MarkerSize', 14, 'LineWidth', 3);
    end
    ttl = {info.name, sprintf('FAILED   t = %.1f ms', 1000 * tSec)};
end
plot(ax, s(1), s(2), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.1 0.75 0.2], 'MarkerEdgeColor', 'k');
plot(ax, g(1), g(2), 'p', 'MarkerSize', 12, 'MarkerFaceColor', [0.9 0.1 0.1], 'MarkerEdgeColor', 'k');
title(ax, ttl, 'FontSize', 9);
end
