function res = plan_prm(map, start, goal, P)
%PLAN_PRM  Probabilistic Roadmap (Kavraki et al., 1996), single query.
%   1) sample P.prm.nSamples free configurations uniformly
%   2) connect each to its k nearest neighbours if the straight edge is
%      collision-free (and shorter than P.prm.maxEdge)
%   3) Dijkstra on the roadmap from start (vertex 1) to goal (vertex 2)
%   Probabilistically complete; struggles when a narrow gap receives no samples.

res = empty_result();
[H, W] = size(map);
freeIdx = find(~map);
n = min(P.prm.nSamples, numel(freeIdx));
pick = freeIdx(randperm(numel(freeIdx), n));
[r, c] = ind2sub([H W], pick);
V = [start; goal; [c r] + (rand(n, 2) - 0.5) * 0.8];   % jitter stays inside cell
nV = n + 2;

D = sqrt(bsxfun(@minus, V(:,1), V(:,1)').^2 + bsxfun(@minus, V(:,2), V(:,2)').^2);
adj = false(nV);
checked = false(nV);
kk = min(P.prm.k + 1, nV);
for i = 1:nV
    [ds, js] = sort(D(i, :));
    js = js(2:kk);  ds = ds(2:kk);
    js = js(ds <= P.prm.maxEdge);
    for j = js
        if checked(i, j), continue; end
        checked(i, j) = true;  checked(j, i) = true;
        if line_of_sight(map, V(i, :), V(j, :))
            adj(i, j) = true;  adj(j, i) = true;
        end
    end
end

% --- Dijkstra on the roadmap ---
dist = inf(nV, 1);  par = zeros(nV, 1);
done = false(nV, 1);  dOpen = inf(nV, 1);
dist(1) = 0;  dOpen(1) = 0;
while true
    [dm, u] = min(dOpen);
    if isinf(dm) || u == 2, break; end
    dOpen(u) = inf;  done(u) = true;
    nb = find(adj(:, u) & ~done);
    tg = dist(u) + D(nb, u);
    b = tg < dist(nb);
    nb = nb(b);  tg = tg(b);
    dist(nb) = tg;  par(nb) = u;  dOpen(nb) = tg;
end

[ei, ej] = find(triu(adj));
res.edges    = [V(ei, :) V(ej, :)];
res.explored = V;
res.nodes    = nV;
res.extra.nEdges = numel(ei);
if isfinite(dist(2))
    chain = 2;  p = 2;
    while p ~= 1
        p = par(p);
        chain(end + 1, 1) = p; %#ok<AGROW>
    end
    res.path = V(flipud(chain), :);
    res.success = true;
end
end
