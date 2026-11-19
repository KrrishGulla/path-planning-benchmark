function R = reachable_mask(free, s)
%REACHABLE_MASK  4-connected flood fill of free space from s = [x y].
%   (A 4-connected route is always valid for the 8-connected planners.)
R = false(size(free));
R(s(2), s(1)) = true;
while true
    Rn = R;
    Rn(2:end, :)   = Rn(2:end, :)   | R(1:end-1, :);
    Rn(1:end-1, :) = Rn(1:end-1, :) | R(2:end, :);
    Rn(:, 2:end)   = Rn(:, 2:end)   | R(:, 1:end-1);
    Rn(:, 1:end-1) = Rn(:, 1:end-1) | R(:, 2:end);
    Rn = Rn & free;
    if isequal(Rn, R), break; end
    R = Rn;
end
end
