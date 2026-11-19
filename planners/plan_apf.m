function res = plan_apf(map, start, goal, P)
%PLAN_APF  Artificial Potential Field (Khatib, 1986) on the grid.
%   U = U_att + U_rep
%     U_att = ka * ||q - goal||                              (conic attraction)
%     U_rep = 0.5*kr*(1/d - 1/d0)^2 * min(1, dg/d0)^2   for d <= d0
%   d  = clearance to the nearest obstacle, dg = distance to goal. The goal-
%   distance factor removes the "goal non-reachable with obstacles nearby"
%   (GNRON) problem, so failures here come from genuine LOCAL MINIMA
%   (U-shaped traps, dead ends, gaps) - the classic weakness of APF.
%   The robot performs steepest descent over the 8 neighbours and stops
%   (fails) when no neighbour has lower potential.

res = empty_result();
[H, W] = size(map);
A = P.apf;
clr = clearance_map(map);
[C, R] = meshgrid(1:W, 1:H);
dg = sqrt((C - goal(1)).^2 + (R - goal(2)).^2);
U = A.ka * dg;
m = clr > 0 & clr <= A.d0;
U(m) = U(m) + 0.5 * A.kr * (1 ./ clr(m) - 1 / A.d0).^2 .* min(1, dg(m) / A.d0).^2;
U(map) = inf;

Uv = U(:);  free = ~map(:);
[off, ~, cA, cB] = grid_moves(H);
s = xy2idx(start, H);  t = xy2idx(goal, H);
cur = s;
trace = zeros(A.maxSteps + 1, 1);
trace(1) = s;  k = 1;
found = false;
while k <= A.maxSteps
    if cur == t, found = true; break; end
    nb = cur + off;
    ok = free(nb);
    ok(5:8) = ok(5:8) & free(cur + cA) & free(cur + cB);
    cand = nb(ok);
    if isempty(cand), break; end
    [um, j] = min(Uv(cand));
    if um >= Uv(cur), break; end              % local minimum -> stuck
    cur = cand(j);
    k = k + 1;
    trace(k) = cur;
end

walked = idx2xy(trace(1:k), H);
res.nodes    = k;
res.explored = walked;
U(isinf(U)) = NaN;
res.extra.U = U;
res.extra.partialPath = walked;
if found
    res.path = walked;
    res.success = true;
else
    res.extra.stuckAt = walked(end, :);
end
end
