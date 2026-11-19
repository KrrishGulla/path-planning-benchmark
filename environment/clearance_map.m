function D = clearance_map(M)
%CLEARANCE_MAP  Exact Euclidean distance (cells) from each free cell to the
%   nearest obstacle cell (obstacle cells get 0). Toolbox-free replacement
%   for bwdist: only obstacle cells that border free space are candidates.
[H, W] = size(M);
D = zeros(H, W);
pad = true(H + 2, W + 2);
pad(2:end-1, 2:end-1) = M;
touchesFree = ~pad(1:end-2, 2:end-1) | ~pad(3:end, 2:end-1) | ...
              ~pad(2:end-1, 1:end-2) | ~pad(2:end-1, 3:end);
[br, bc] = find(M & touchesFree);
[fr, fc] = find(~M);
if isempty(fr), return; end
if isempty(br), D(~M) = max(H, W); return; end
br = br';  bc = bc';
d = zeros(numel(fr), 1);
chunk = 1500;
for i = 1:chunk:numel(fr)
    j = i:min(i + chunk - 1, numel(fr));
    dd = bsxfun(@minus, fr(j), br).^2 + bsxfun(@minus, fc(j), bc).^2;
    d(j) = sqrt(min(dd, [], 2));
end
D(~M) = d;
end
