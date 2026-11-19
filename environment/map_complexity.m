function C = map_complexity(raw, s, g, refLen)
%MAP_COMPLEXITY  Quantitative difficulty descriptors of a scenario.
%   density       - fraction of cells occupied by obstacles
%   meanClearance - mean distance of free cells to the nearest obstacle
%   narrowness    - fraction of free cells within 3 cells of an obstacle
%                   (how much of the free space is "corridor-like")
%   tortuosity    - near-optimal path length / straight-line distance
%                   (refLen comes from Theta*, the any-angle reference)
%   bottleneck    - WIDEST-PATH clearance between start and goal: the largest
%                   clearance c such that start and goal stay connected
%                   through cells with clearance >= c. This is the width of
%                   the tightest gap the robot cannot avoid - the quantity
%                   behind the "narrow passage problem" of sampling planners,
%                   which global density measures do not capture.
clr = clearance_map(raw);
fc  = clr(~raw);
C.density       = mean(raw(:));
C.meanClearance = mean(fc);
C.narrowness    = mean(fc <= 3);
C.euclid        = hypot(g(1) - s(1), g(2) - s(2));
if nargin < 4 || isnan(refLen), refLen = NaN; end
C.tortuosity    = refLen / C.euclid;
C.bottleneck    = widest_path_clearance(clr, s, g);
end

function b = widest_path_clearance(clr, s, g)
% binary search over candidate clearance levels; each test is a flood fill
cap = min(clr(s(2), s(1)), clr(g(2), g(1)));
lv = unique(clr(clr > 0 & clr <= cap));
lo = 1;  hi = numel(lv);  b = lv(1);
while lo <= hi
    mid = floor((lo + hi) / 2);
    R = reachable_mask(clr >= lv(mid), s);
    if R(g(2), g(1))
        b = lv(mid);  lo = mid + 1;
    else
        hi = mid - 1;
    end
end
end
