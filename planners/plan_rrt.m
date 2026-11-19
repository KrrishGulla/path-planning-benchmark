function res = plan_rrt(map, start, goal, P)
%PLAN_RRT  Rapidly-exploring Random Tree (LaValle, 1998) with goal bias.
%   Grows a tree from the start towards random samples in steps of
%   P.rrt.step; stops at the FIRST connection to the goal. Fast and
%   probabilistically complete, but paths are jagged and far from optimal.

res = empty_result();
[H, W] = size(map);
Q = P.rrt;
maxN = Q.maxIter + 2;
T = zeros(maxN, 2);  par = zeros(maxN, 1);
T(1, :) = start;  n = 1;
found = false;

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
    n = n + 1;  T(n, :) = qn;  par(n) = i;
    if hypot(qn(1) - goal(1), qn(2) - goal(2)) <= Q.goalTol && line_of_sight(map, qn, goal)
        n = n + 1;  T(n, :) = goal;  par(n) = n - 1;
        found = true;
        break;
    end
end

res.nodes    = n;
res.explored = T(1:n, :);
res.edges    = [T(par(2:n), :) T(2:n, :)];
res.extra.iterations = it;
if found
    chain = n;  p = n;
    while p ~= 1
        p = par(p);
        chain(end + 1, 1) = p; %#ok<AGROW>
    end
    res.path = T(flipud(chain), :);
    res.success = true;
end
end
