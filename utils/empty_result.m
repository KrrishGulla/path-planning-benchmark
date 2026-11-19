function res = empty_result()
%EMPTY_RESULT  Common output structure returned by every planner.
%   path     : Nx2 [x y] waypoints (x = column, y = row), empty on failure
%   success  : true if a collision-free path to the goal was found
%   nodes    : search effort (expanded cells / roadmap vertices / tree nodes / steps)
%   explored : Kx2 points explored, in visiting order (for plots/animation)
%   edges    : Mx4 [x1 y1 x2 y2] tree or roadmap edges (sampling planners)
%   extra    : planner-specific data (e.g. APF potential, RRT* first-solution iter)
res = struct('path', zeros(0, 2), 'success', false, 'nodes', 0, ...
             'explored', zeros(0, 2), 'edges', zeros(0, 4), 'extra', struct());
end
