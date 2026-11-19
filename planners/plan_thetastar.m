function res = plan_thetastar(map, start, goal, P) %#ok<INUSD>
%PLAN_THETASTAR  Basic Theta* (Nash et al., 2007) - any-angle path planning.
%   Like A*, but when the parent of the current cell can see a neighbour
%   directly (line of sight), the neighbour is linked to that parent. Paths
%   are therefore not restricted to 45-degree headings: shorter and with far
%   fewer turns than grid A*. Heuristic: Euclidean distance.

res = empty_result();
[H, W] = size(map);
N = H * W;
s = xy2idx(start, H);
t = xy2idx(goal, H);

free = ~map(:);
[C, R] = meshgrid(1:W, 1:H);
X = C(:);  Y = R(:);
hAll = sqrt((X - goal(1)).^2 + (Y - goal(2)).^2);

g      = inf(N, 1);
fOpen  = inf(N, 1);
parent = zeros(N, 1);
closed = false(N, 1);
order  = zeros(N, 1);
[off, stepCost, cA, cB] = grid_moves(H);

g(s) = 0;  parent(s) = s;  fOpen(s) = hAll(s);
nExp = 0;  found = false;

while true
    [fm, u] = min(fOpen);
    if isinf(fm), break; end
    if u == t, found = true; break; end
    fOpen(u) = inf;  closed(u) = true;
    nExp = nExp + 1;  order(nExp) = u;

    nb = u + off;
    ok = free(nb);
    ok(5:8) = ok(5:8) & free(u + cA) & free(u + cB);
    ok = ok & ~closed(nb);
    nb = nb(ok);  sc = stepCost(ok);

    pu  = parent(u);
    pxy = [X(pu) Y(pu)];
    for k = 1:numel(nb)
        v = nb(k);
        vxy = [X(v) Y(v)];
        if line_of_sight(map, pxy, vxy)                      % path 2
            tg  = g(pu) + sqrt((pxy(1) - vxy(1))^2 + (pxy(2) - vxy(2))^2);
            par = pu;
        else                                                 % path 1
            tg  = g(u) + sc(k);
            par = u;
        end
        if tg < g(v)
            g(v) = tg;  parent(v) = par;  fOpen(v) = tg + hAll(v);
        end
    end
end

res.success  = found;
res.nodes    = nExp;
res.explored = idx2xy(order(1:nExp), H);
if found
    chain = t;  p = t;
    while p ~= s
        p = parent(p);
        chain(end + 1, 1) = p; %#ok<AGROW>
    end
    res.path = idx2xy(flipud(chain), H);
end
end
