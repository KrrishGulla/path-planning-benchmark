function R = run_benchmark(P)
%RUN_BENCHMARK  Monte-Carlo comparison of 8 global planners over 5
%   environment classes. Every planner sees exactly the same map, start and
%   goal in each trial (reproducible seeds).
%
%   R = run_benchmark();          % default settings
%   P = default_params(); P.trialsPerEnv = 5; R = run_benchmark(P);  % quick
%
%   Results are stored in R (arrays sized [env x trial x planner]) and saved
%   to results/benchmark_results.mat.

if nargin < 1, P = default_params(); end
setup_paths();
if ~exist(P.resultsDir, 'dir'), mkdir(P.resultsDir); end

reg = planner_registry();
nP = numel(reg);  nE = numel(P.envTypes);  nT = P.trialsPerEnv;
thetaIdx = find(strcmp({reg.short}, 'THS'));

sz = [nE nT nP];
R.success    = false(sz);
R.time       = nan(sz);
R.length     = nan(sz);
R.turning    = nan(sz);
R.turnPerLen = nan(sz);
R.maxTurn    = nan(sz);
R.vertices   = nan(sz);
R.minClear   = nan(sz);
R.meanClear  = nan(sz);
R.nodes      = nan(sz);
cx = {'density', 'meanClearance', 'narrowness', 'tortuosity', 'euclid', 'bottleneck'};
for k = 1:numel(cx), R.cx.(cx{k}) = nan(nE, nT); end
R.examples = cell(1, nE);

% ---- untimed warm-up (removes MATLAB JIT first-call bias) ----
if P.warmup
    [~, infl0, s0, g0] = make_scenario('random', P, 1);
    for p = 1:nP, reg(p).fn(infl0, s0, g0, P); end
end

fprintf('\n=== Global path-planning benchmark: %d planners x %d environments x %d trials ===\n', ...
    nP, nE, nT);
tAll = tic;
for e = 1:nE
    fprintf('\n[%d/%d] %s\n', e, nE, P.envNames{e});
    for k = 1:nT
        seed = P.baseSeed + 1000 * e + k;
        [raw, infl, s, g] = make_scenario(P.envTypes{e}, P, seed);
        clr = clearance_map(raw);
        if k == 1
            ex.raw = raw;  ex.infl = infl;  ex.s = s;  ex.g = g;
            ex.paths = cell(1, nP);  ex.partial = cell(1, nP);
        end
        for p = 1:nP
            set_seed(seed * 13 + p);
            t0 = tic;
            res = reg(p).fn(infl, s, g, P);
            R.time(e, k, p) = toc(t0);
            R.success(e, k, p) = res.success;
            R.nodes(e, k, p) = res.nodes;
            if res.success
                m = path_metrics(res.path, clr);
                R.length(e, k, p)     = m.length;
                R.turning(e, k, p)    = m.turning;
                R.turnPerLen(e, k, p) = m.turnPerLen;
                R.maxTurn(e, k, p)    = m.maxTurn;
                R.vertices(e, k, p)   = m.vertices;
                R.minClear(e, k, p)   = m.minClear;
                R.meanClear(e, k, p)  = m.meanClear;
            end
            if k == 1
                ex.paths{p} = res.path;
                if isfield(res.extra, 'partialPath'), ex.partial{p} = res.extra.partialPath; end
            end
        end
        C = map_complexity(raw, s, g, R.length(e, k, thetaIdx));
        for c = 1:numel(cx), R.cx.(cx{c})(e, k) = C.(cx{c}); end
        if k == 1, R.examples{e} = ex; end
        fprintf('  trial %2d/%d  success: %s\n', k, nT, ...
            sprintf('%d', squeeze(R.success(e, k, :))'));
    end
end
fprintf('\nBenchmark finished in %.1f s\n', toc(tAll));

R.P = P;
R.names    = {reg.name};
R.shorts   = {reg.short};
R.families = {reg.family};
R.colors   = vertcat(reg.color);
R.envTypes = P.envTypes;
R.envNames = P.envNames;
R.created  = datestr(now);
save(fullfile(P.resultsDir, 'benchmark_results.mat'), 'R');
fprintf('Saved results to %s\n', fullfile(P.resultsDir, 'benchmark_results.mat'));
end
