function P = default_params()
%DEFAULT_PARAMS  All tunable settings for the global path-planning benchmark.
%   Edit values here (or override fields in run_all.m) instead of touching
%   the planner code. Units are grid cells (1 cell ~ 0.1 m for a small AMR).

root = fileparts(mfilename('fullpath'));

%% ---------------- Environment ----------------
P.mapSize        = [100 100];   % [rows cols]
P.inflateRadius  = 1;           % configuration-space inflation (robot radius, cells)
P.envTypes       = {'random', 'forest', 'maze', 'rooms', 'narrow'};
P.envNames       = {'Random Blocks', 'Cluttered Forest', 'Maze', ...
                    'Indoor Rooms', 'Narrow Passage'};
P.minStartGoalDist = 0.55;      % start-goal distance >= this fraction of map diagonal
P.minEndpointClearance = 3;     % start/goal must be this far from obstacles (cells)

%% ---------------- Benchmark ----------------
P.trialsPerEnv   = 20;          % Monte-Carlo trials per environment class
P.baseSeed       = 2026;        % makes every run fully reproducible
P.warmup         = true;        % untimed warm-up run (removes JIT first-call bias)

%% ---------------- Planner parameters ----------------
P.wastar.w          = 2.0;      % Weighted A* inflation factor (epsilon)

P.prm.nSamples      = 500;      % roadmap vertices
P.prm.k             = 12;       % k-nearest neighbours tried per vertex
P.prm.maxEdge       = 25;       % max edge length (cells)

P.rrt.step          = 3;        % steering step
P.rrt.goalBias      = 0.10;     % probability of sampling the goal
P.rrt.maxIter       = 4000;     % same sampling budget as RRT* (fair)
P.rrt.goalTol       = 3;

P.rrtstar.step      = 3;
P.rrtstar.goalBias  = 0.10;
P.rrtstar.maxIter   = 4000;     % fixed budget (anytime: keeps improving after 1st solution)
P.rrtstar.goalTol   = 3;
P.rrtstar.gamma     = 50;       % rewiring radius r = gamma*sqrt(log n / n)
P.rrtstar.rMin      = 4;
P.rrtstar.rMax      = 10;

P.apf.ka            = 1.0;      % attractive gain (conic potential)
P.apf.kr            = 60;       % repulsive gain
P.apf.d0            = 5;        % repulsion influence distance (cells)
P.apf.maxSteps      = 3000;

%% ---------------- Output ----------------
P.showFigures    = true;
P.saveFigures    = true;
P.resultsDir     = fullfile(root, 'results');
P.figDir         = fullfile(P.resultsDir, 'figures');
end
