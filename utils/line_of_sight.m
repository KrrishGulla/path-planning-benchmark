function ok = line_of_sight(map, p, q)
%LINE_OF_SIGHT  True if the straight segment p->q crosses only free cells.
%   map : logical occupancy grid (true = obstacle)
%   p,q : [x y] continuous coordinates (x = column, y = row)
%   The segment is sampled at 1/4-cell resolution (dense enough for an
%   inflated configuration-space map).
d = sqrt((q(1) - p(1))^2 + (q(2) - p(2))^2);
n = max(2, ceil(4 * d) + 1);
t = linspace(0, 1, n);
x = round(p(1) + t * (q(1) - p(1)));
y = round(p(2) + t * (q(2) - p(2)));
ok = ~any(map(y + (x - 1) * size(map, 1)));
end
