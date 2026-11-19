function Mi = inflate_map(M, r)
%INFLATE_MAP  Configuration-space obstacle inflation by robot radius r (cells).
%   Lets every planner treat the robot as a point. Toolbox-free (uses conv2).
if r <= 0, Mi = M; return; end
[X, Y] = meshgrid(-r:r, -r:r);
K = double(X.^2 + Y.^2 <= (r + 0.5)^2);
Mi = conv2(double(M), K, 'same') > 0;
Mi([1 end], :) = true;
Mi(:, [1 end]) = true;
end
