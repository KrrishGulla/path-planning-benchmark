function res = plan_rrtstar(map, start, goal, P)
%PLAN_RRTSTAR  RRT* (Karaman & Frazzoli, 2011) - asymptotically optimal RRT.
%   Adds two steps to RRT: (1) choose-parent - connect each new node to the
%   lowest-cost node in a shrinking radius r = gamma*sqrt(log n / n);
%   (2) rewire - re-route neighbours through the new node if that is cheaper
%   (cost changes are propagated to all descendants). Runs for a fixed
%   iteration budget (anytime behaviour), then connects the best node to goal.

res = empty_result();
[H, W] = size(map);
Q = P.rrtstar;
maxN = Q.maxIter + 2;
T = zeros(maxN, 2);  par = zeros(maxN, 1);  cost = zeros(maxN, 1);
T(1, :) = start;  n = 1;
firstIt = NaN;

for it = 1:Q.maxIter
    if rand < Q.goalBias
        q = goal;
    else
        q = [1 + rand * (W - 1), 1 + rand * (H - 1)];
    end
    d2 = (T(1:n, 1) - q(1)).^2 + (T(1:n, 2) - q(2)).^2;
    [dm, i] = min(d2);
    dm = sqrt(dm);
    if dm < 1e-9, continue; end
    qn = T(i, :) + (q - T(i, :)) * min(Q.step, dm) / dm;
    if map(round(qn(2)), round(qn(1))), continue; end
    if ~line_of_sight(map, T(i, :), qn), continue; end

    % ---- choose parent ----
    r = max(Q.rMin, min(Q.rMax, Q.gamma * sqrt(log(n + 1) / (n + 1))));
    dn2  = (T(1:n, 1) - qn(1)).^2 + (T(1:n, 2) - qn(2)).^2;
    near = find(dn2 <= r^2);
    dn   = sqrt(dn2(near));
    best = i;
    bestC = cost(i) + hypot(qn(1) - T(i, 1), qn(2) - T(i, 2));
    [cs, o] = sort(cost(near) + dn);
    for m = 1:numel(o)
        if cs(m) >= bestC, break; end
        j = near(o(m));
        if line_of_sight(map, T(j, :), qn)
            best = j;  bestC = cs(m);
            break;
        end
    end
    n = n + 1;
    T(n, :) = qn;  par(n) = best;  cost(n) = bestC;

    % ---- rewire ----
    for m = 1:numel(near)
        j = near(m);
        if j == best, continue; end
        nc = bestC + dn(m);
        if nc < cost(j) - 1e-9 && line_of_sight(map, qn, T(j, :))
            delta = nc - cost(j);
            cost(j) = nc;  par(j) = n;
            % propagate the cost decrease to all descendants of j
            level = j;
            while ~isempty(level)
                kids = find(ismember(par(1:n), level));
                cost(kids) = cost(kids) + delta;
                level = kids;
            end
        end
    end

    if isnan(firstIt) && hypot(qn(1) - goal(1), qn(2) - goal(2)) <= Q.goalTol ...
            && line_of_sight(map, qn, goal)
        firstIt = it;
    end
end

% ---- connect best tree node to the goal ----
dg = sqrt((T(1:n, 1) - goal(1)).^2 + (T(1:n, 2) - goal(2)).^2);
cand = find(dg <= max(Q.goalTol, Q.rMax));
[~, o] = sort(cost(cand) + dg(cand));
res.nodes    = n;
res.explored = T(1:n, :);
res.edges    = [T(par(2:n), :) T(2:n, :)];
res.extra.firstSolutionIter = firstIt;
for m = 1:numel(o)
    j = cand(o(m));
    if line_of_sight(map, T(j, :), goal)
        chain = j;  p = j;
        while p ~= 1
            p = par(p);
            chain(end + 1, 1) = p; %#ok<AGROW>
        end
        res.path = [T(flipud(chain), :); goal];
        if hypot(res.path(end-1, 1) - goal(1), res.path(end-1, 2) - goal(2)) < 1e-9
            res.path(end, :) = [];
        end
        res.success = true;
        break;
    end
end
end
