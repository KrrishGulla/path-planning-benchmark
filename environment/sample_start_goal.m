function [s, g, ok] = sample_start_goal(raw, infl, type, P)
%SAMPLE_START_GOAL  Pick a solvable, well-separated start/goal pair.
%   Both points must be free in the inflated map, at least
%   P.minEndpointClearance cells from obstacles, and mutually reachable.
%   'narrow' maps force start on the left and goal on the right so that the
%   narrow gaps must be crossed.
[H, W] = size(raw);
s = [];  g = [];  ok = false;
clr  = clearance_map(raw);
cand = ~infl & clr >= P.minEndpointClearance;
[r, c] = find(cand);
if numel(r) < 2, return; end
dmin = P.minStartGoalDist * sqrt(H^2 + W^2);
lastS = [-1 -1];  reach = [];
for attempt = 1:300
    if strcmpi(type, 'narrow')
        L = find(c <= 0.15 * W);  Rt = find(c >= 0.85 * W);
        if isempty(L) || isempty(Rt), return; end
        i = L(randi(numel(L)));  j = Rt(randi(numel(Rt)));
    else
        i = randi(numel(r));  j = randi(numel(r));
        if hypot(r(i) - r(j), c(i) - c(j)) < dmin, continue; end
    end
    s = [c(i) r(i)];  g = [c(j) r(j)];
    if ~isequal(s, lastS)
        reach = reachable_mask(~infl, s);
        lastS = s;
    end
    if reach(g(2), g(1)), ok = true; return; end
end
end
