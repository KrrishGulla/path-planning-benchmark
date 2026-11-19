function res = plan_astar(map, start, goal, P) %#ok<INUSD>
%PLAN_ASTAR  A* with the octile-distance heuristic. Same optimal grid path as
%   Dijkstra but expands far fewer cells.
res = grid_search_core(map, start, goal, 1, true);
end
