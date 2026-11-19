function M = generate_map(type, sz)
%GENERATE_MAP  Procedurally generate a 2-D occupancy grid (true = obstacle).
%   Five environment classes, each stressing planners differently:
%     'random' - scattered rectangular blocks (warehouse / generic clutter)
%     'forest' - many small circular obstacles (orchard, pillars, crowd)
%     'maze'   - corridors with dead ends and a few loops (local-minimum trap)
%     'rooms'  - 3x3 office with walls, doorways and furniture (indoor AMR)
%     'narrow' - full-height walls pierced by narrow gaps (narrow-passage problem)
%   A one-cell border wall is always added.

H = sz(1);  W = sz(2);
switch lower(type)
    case 'random', M = random_blocks(H, W);
    case 'forest', M = forest(H, W);
    case 'maze',   M = maze(H, W);
    case 'rooms',  M = rooms(H, W);
    case 'narrow', M = narrow(H, W);
    otherwise, error('generate_map: unknown type "%s"', type);
end
M([1 end], :) = true;
M(:, [1 end]) = true;
end

% ------------------------------------------------------------------------
function M = random_blocks(H, W)
M = false(H, W);
nObs = randi([14 20]);
for k = 1:nObs
    h = randi([4 16]);  w = randi([4 16]);
    r0 = randi([2 H - h]);  c0 = randi([2 W - w]);
    M(r0:r0 + h - 1, c0:c0 + w - 1) = true;
end
end

% ------------------------------------------------------------------------
function M = forest(H, W)
[C, R] = meshgrid(1:W, 1:H);
M = false(H, W);
n = randi([70 90]);
for k = 1:n
    rad = 1.5 + 2.5 * rand;
    cx = 3 + rand * (W - 6);  cy = 3 + rand * (H - 6);
    M = M | ((C - cx).^2 + (R - cy).^2 <= rad^2);
end
end

% ------------------------------------------------------------------------
function M = maze(H, W)
% Recursive-backtracker (DFS) perfect maze + a few extra openings (loops)
p = 12;  wt = 3;                       % cell pitch, wall thickness
nR = floor((H - wt) / p);  nC = floor((W - wt) / p);
M = true(H, W);
visited = false(nR, nC);
cur = [randi(nR) randi(nC)];
visited(cur(1), cur(2)) = true;
M = carve(M, cur, cur, p, wt);
stack = cur;
while ~isempty(stack)
    cur = stack(end, :);
    nb = [cur(1)-1 cur(2); cur(1)+1 cur(2); cur(1) cur(2)-1; cur(1) cur(2)+1];
    nb = nb(nb(:,1) >= 1 & nb(:,1) <= nR & nb(:,2) >= 1 & nb(:,2) <= nC, :);
    nb = nb(~visited(sub2ind([nR nC], nb(:,1), nb(:,2))), :);
    if isempty(nb)
        stack(end, :) = [];
        continue;
    end
    nx = nb(randi(size(nb, 1)), :);
    M = carve(M, cur, nx, p, wt);
    visited(nx(1), nx(2)) = true;
    stack(end + 1, :) = nx; %#ok<AGROW>
end
% knock down some extra walls -> multiple homotopy classes
nExtra = round(0.12 * nR * nC);
for k = 1:nExtra
    a = [randi(nR) randi(nC)];
    if rand < 0.5, b = a + [0 1]; else, b = a + [1 0]; end
    if b(1) <= nR && b(2) <= nC
        M = carve(M, a, b, p, wt);
    end
end
end

function M = carve(M, a, b, p, wt)
% clear the interiors of grid-cells a and b and the wall between them
r1 = wt + (min(a(1), b(1)) - 1) * p + 1;  r2 = wt + (max(a(1), b(1)) - 1) * p + (p - wt);
c1 = wt + (min(a(2), b(2)) - 1) * p + 1;  c2 = wt + (max(a(2), b(2)) - 1) * p + (p - wt);
M(r1:r2, c1:c2) = false;
end

% ------------------------------------------------------------------------
function M = rooms(H, W)
% 3x3 office floor: walls, a doorway in every internal wall, furniture
M = false(H, W);
nr = 3;  nc = 3;  wt = 2;
rb = round(linspace(1, H - wt + 1, nr + 1));
cb = round(linspace(1, W - wt + 1, nc + 1));
for b = rb, M(b:b + wt - 1, :) = true; end
for b = cb, M(:, b:b + wt - 1) = true; end
% doorways in internal horizontal walls
for i = 2:nr
    for j = 1:nc
        span = (cb(j) + wt):(cb(j + 1) - 1);
        door = randi([6 9]);
        c0 = span(1) + 2 + randi(max(1, numel(span) - door - 4)) - 1;
        M(rb(i):rb(i) + wt - 1, c0:c0 + door - 1) = false;
    end
end
% doorways in internal vertical walls
for j = 2:nc
    for i = 1:nr
        span = (rb(i) + wt):(rb(i + 1) - 1);
        door = randi([6 9]);
        r0 = span(1) + 2 + randi(max(1, numel(span) - door - 4)) - 1;
        M(r0:r0 + door - 1, cb(j):cb(j) + wt - 1) = false;
    end
end
% furniture, kept >= 4 cells from walls so doorways stay open
for i = 1:nr
    for j = 1:nc
        rIn = [rb(i) + wt + 4, rb(i + 1) - 5];
        cIn = [cb(j) + wt + 4, cb(j + 1) - 5];
        for k = 1:randi([1 2])
            h = randi([4 9]);  w = randi([4 9]);
            if rIn(2) - h < rIn(1) || cIn(2) - w < cIn(1), continue; end
            r0 = randi([rIn(1) rIn(2) - h]);  c0 = randi([cIn(1) cIn(2) - w]);
            M(r0:r0 + h - 1, c0:c0 + w - 1) = true;
        end
    end
end
end

% ------------------------------------------------------------------------
function M = narrow(H, W)
% 2-3 full-height walls, each with one narrow gap (~2 free cells after inflation)
M = false(H, W);
nWalls = randi([2 3]);
xs = round(linspace(0.30 * W, 0.70 * W, nWalls)) + randi([-4 4], 1, nWalls);
th = 3;
for k = 1:nWalls
    M(:, xs(k):xs(k) + th - 1) = true;
    gap = randi([4 5]);
    g0 = randi([10 H - 10 - gap]);
    M(g0:g0 + gap - 1, xs(k):xs(k) + th - 1) = false;
end
% light clutter away from the walls
nClutter = randi([6 10]);
for k = 1:nClutter
    h = randi([3 7]);  w = randi([3 7]);
    r0 = randi([3 H - h - 2]);  c0 = randi([3 W - w - 2]);
    cols = c0:c0 + w - 1;
    tooClose = false;
    for x = xs
        if any(cols >= x - 5 & cols <= x + th + 4), tooClose = true; end
    end
    if ~tooClose
        M(r0:r0 + h - 1, cols) = true;
    end
end
end
