function M = path_metrics(path, clrRaw)
%PATH_METRICS  Quality metrics of one path.
%   length     - total Euclidean length (cells)
%   turning    - sum of absolute heading changes (rad): lower = smoother
%   turnPerLen - turning / length (rad per cell): length-independent smoothness
%   maxTurn    - sharpest single heading change (rad)
%   vertices   - number of real turning points (+ start/goal); collinear
%                grid points are not counted (what a tracking controller sees)
%   minClear   - minimum distance to the TRUE (non-inflated) obstacles along
%                the densely re-sampled path: the safety margin
%   meanClear  - mean clearance along the path
M = struct('length', NaN, 'turning', NaN, 'turnPerLen', NaN, 'maxTurn', NaN, ...
           'vertices', NaN, 'minClear', NaN, 'meanClear', NaN);
if size(path, 1) < 2, return; end

seg = diff(path, 1, 1);
L = sqrt(sum(seg.^2, 2));
keep = L > 1e-9;
seg = seg(keep, :);  L = L(keep);
M.length = sum(L);

hd = atan2(seg(:, 2), seg(:, 1));
if numel(hd) > 1
    dth = diff(hd);
    dth = atan2(sin(dth), cos(dth));           % wrap to [-pi, pi]
    M.turning  = sum(abs(dth));
    M.maxTurn  = max(abs(dth));
    M.vertices = 2 + nnz(abs(dth) > 1e-6);
else
    M.turning = 0;  M.maxTurn = 0;  M.vertices = 2;
end
M.turnPerLen = M.turning / max(M.length, eps);

% densify the path and sample the true-map clearance along it
pts = path(1, :);
for k = 1:size(path, 1) - 1
    a = path(k, :);  b = path(k + 1, :);
    nS = max(2, ceil(4 * hypot(b(1) - a(1), b(2) - a(2))) + 1);
    t = linspace(0, 1, nS)';
    pts = [pts; a + t(2:end) * (b - a)]; %#ok<AGROW>
end
H = size(clrRaw, 1);
cv = clrRaw(round(pts(:, 2)) + (round(pts(:, 1)) - 1) * H);
M.minClear  = min(cv);
M.meanClear = mean(cv);
end
