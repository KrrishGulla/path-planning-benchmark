function R = planner_registry()
%PLANNER_REGISTRY  The 8 planners compared in this study, grouped by family.
%   Each entry: name, short code, algorithm family, function handle, plot colour.

names    = {'Dijkstra', 'A*', 'Weighted A*', 'Theta*', ...
            'PRM', 'RRT', 'RRT*', 'APF'};
shorts   = {'DIJ', 'AST', 'WAS', 'THS', 'PRM', 'RRT', 'RRS', 'APF'};
families = {'Graph search', 'Graph search', 'Graph search', 'Any-angle search', ...
            'Sampling-based', 'Sampling-based', 'Sampling-based', 'Potential field'};
fns      = {@plan_dijkstra, @plan_astar, @plan_weighted_astar, @plan_thetastar, ...
            @plan_prm, @plan_rrt, @plan_rrtstar, @plan_apf};
colors   = [0.30 0.30 0.30;   % Dijkstra   - grey
            0.00 0.45 0.74;   % A*         - blue
            0.30 0.75 0.93;   % W-A*       - light blue
            0.47 0.67 0.19;   % Theta*     - green
            0.93 0.69 0.13;   % PRM        - amber
            0.85 0.33 0.10;   % RRT        - orange
            0.64 0.08 0.18;   % RRT*       - dark red
            0.49 0.18 0.56];  % APF        - purple

R = struct('name', names, 'short', shorts, 'family', families, ...
           'fn', fns, 'color', num2cell(colors, 2)');
end
