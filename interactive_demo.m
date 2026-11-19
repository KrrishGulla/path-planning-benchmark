function interactive_demo(envType, seed)
%INTERACTIVE_DEMO  Click your own START and GOAL, then watch all 8 planners.
%
%   interactive_demo            % indoor rooms map
%   interactive_demo('maze')    % random forest maze rooms narrow
%
%   Left-click the START, then the GOAL. Clicks are snapped to the nearest
%   free cell of the inflated map.

if nargin < 1, envType = 'rooms'; end
if nargin < 2, seed = 7; end
setup_paths();
P = default_params();
e = find(strcmpi(P.envTypes, envType));
if isempty(e), error('Unknown environment "%s".', envType); end
[raw, infl] = make_scenario(P.envTypes{e}, P, P.baseSeed + seed);

f = new_fig(P, 'Pick start and goal', [760 760]);
ax = axes('Parent', f);
plot_map(ax, raw, infl);
title(ax, 'Click START (1st click) and GOAL (2nd click)', 'FontSize', 12);
[x, y] = ginput(2);
s = snap_free(round([x(1) y(1)]), infl);
g = snap_free(round([x(2) y(2)]), infl);
close(f);

reach = reachable_mask(~infl, s);
if ~reach(g(2), g(1))
    error('Goal is not reachable from start on this map - pick again.');
end
visualize_scenario(raw, infl, s, g, P, [P.envNames{e} ' (interactive)']);
end

function p = snap_free(p, infl)
[H, W] = size(infl);
p = [min(max(p(1), 2), W - 1), min(max(p(2), 2), H - 1)];
if ~infl(p(2), p(1)), return; end
[r, c] = find(~infl);
[~, k] = min((r - p(2)).^2 + (c - p(1)).^2);
p = [c(k) r(k)];
end
