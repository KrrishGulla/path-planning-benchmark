function res = grid_search_core(map, start, goal, w, useHeuristic)
%GRID_SEARCH_CORE  Shared best-first search engine for Dijkstra, A* and
%   Weighted A*, so the three differ ONLY in their heuristic (fair comparison).
%
%   map          : logical occupancy grid (true = obstacle), inflated
%   start, goal  : [x y] integer cells (x = column, y = row)
%   w            : heuristic weight (1 -> A*, >1 -> Weighted A*)
%   useHeuristic : false -> Dijkstra (h = 0)
%
%   8-connected moves (cost 1 / sqrt(2)); diagonal corner-cutting is not
%   allowed. Heuristic = octile distance (admissible & consistent for this
%   move set, so A* is optimal on the grid).

res = empty_result();
[H, W] = size(map);
N = H * W;
s = xy2idx(start, H);
t = xy2idx(goal, H);

free   = ~map(:);
g      = inf(N, 1);
fOpen  = inf(N, 1);          % f-value of OPEN nodes, inf otherwise
parent = zeros(N, 1);
closed = false(N, 1);
order  = zeros(N, 1);

[off, stepCost, cA, cB] = grid_moves(H);

if useHeuristic
    [C, R] = meshgrid(1:W, 1:H);
    dx = abs(C(:) - goal(1));
    dy = abs(R(:) - goal(2));
    hAll = w * ((dx + dy) + (sqrt(2) - 2) * min(dx, dy));
else
    hAll = zeros(N, 1);
end

g(s) = 0;
fOpen(s) = hAll(s);
nExp = 0;
found = false;

while true
    [fm, u] = min(fOpen);
    if isinf(fm), break; end
    if u == t, found = true; break; end
    fOpen(u)  = inf;
    closed(u) = true;
    nExp = nExp + 1;
    order(nExp) = u;

    nb = u + off;
    ok = free(nb);
    ok(5:8) = ok(5:8) & free(u + cA) & free(u + cB);   % no corner cutting
    ok = ok & ~closed(nb);
    nb = nb(ok);
    tg = g(u) + stepCost(ok);
    better = tg < g(nb);
    nb = nb(better);
    tg = tg(better);
    g(nb) = tg;
    parent(nb) = u;
    fOpen(nb) = tg + hAll(nb);
end

res.success  = found;
res.nodes    = nExp;
res.explored = idx2xy(order(1:nExp), H);
if found
    p = t;
    chain = zeros(N, 1);  k = 1;  chain(1) = t;
    while p ~= s
        p = parent(p);
        k = k + 1;
        chain(k) = p;
    end
    res.path = idx2xy(flipud(chain(1:k)), H);
end
end
