function main_demo(envType, seed)
%MAIN_DEMO  Single-scenario demonstration of all 8 planners.
%
%   main_demo                 % indoor rooms map (default)
%   main_demo('maze')         % any of: random forest maze rooms narrow
%   main_demo('narrow', 11)   % choose the random seed
%   main_demo('all')          % one scenario of every environment class
%
%   Shows, for every planner, the region it explored and the path it found,
%   prints a comparison table, and saves figures to results/figures/.

if nargin < 1, envType = 'rooms'; end
if nargin < 2, seed = 7; end
setup_paths();
P = default_params();

if strcmpi(envType, 'all')
    for e = 1:numel(P.envTypes)
        main_demo(P.envTypes{e}, seed);
    end
    return;
end

e = find(strcmpi(P.envTypes, envType));
if isempty(e)
    error('Unknown environment "%s". Use: %s', envType, strjoin(P.envTypes, ', '));
end
[raw, infl, s, g] = make_scenario(P.envTypes{e}, P, P.baseSeed + seed);
visualize_scenario(raw, infl, s, g, P, P.envNames{e});
end
