function idx = xy2idx(xy, H)
%XY2IDX  [x y] (x = column, y = row) -> linear grid index.
idx = (round(xy(:, 1)) - 1) * H + round(xy(:, 2));
end
