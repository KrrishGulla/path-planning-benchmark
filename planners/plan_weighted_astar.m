function res = plan_weighted_astar(map, start, goal, P)
%PLAN_WEIGHTED_ASTAR  A* with an inflated heuristic f = g + w*h (w = P.wastar.w).
%   Greedier -> faster, with path cost bounded by w * optimal.
res = grid_search_core(map, start, goal, P.wastar.w, true);
end
