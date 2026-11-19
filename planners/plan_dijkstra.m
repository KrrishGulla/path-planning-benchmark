function res = plan_dijkstra(map, start, goal, P) %#ok<INUSD>
%PLAN_DIJKSTRA  Uniform-cost search (no heuristic). Optimal on the 8-connected
%   grid, but explores in all directions -> highest search effort.
res = grid_search_core(map, start, goal, 1, false);
end
