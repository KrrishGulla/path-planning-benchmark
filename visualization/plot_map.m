function plot_map(ax, raw, infl, s, g)
%PLOT_MAP  Draw the occupancy grid: obstacles dark, inflation zone light grey,
%   free space white; optional start (green circle) and goal (red star).
[H, W] = size(raw);
img = ones(H, W);
img(infl) = 0.82;
img(raw)  = 0.18;
rgb = repmat(img, [1 1 3]);
image(ax, 1:W, 1:H, rgb);
axis(ax, 'image');
hold(ax, 'on');
set(ax, 'XTick', [], 'YTick', [], 'Box', 'on');
if nargin >= 5 && ~isempty(s)
    plot(ax, s(1), s(2), 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.1 0.75 0.2], ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1);
    plot(ax, g(1), g(2), 'p', 'MarkerSize', 13, 'MarkerFaceColor', [0.9 0.1 0.1], ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1);
end
end
