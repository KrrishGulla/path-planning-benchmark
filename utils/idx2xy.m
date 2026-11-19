function xy = idx2xy(idx, H)
%IDX2XY  Linear grid index -> [x y] (x = column, y = row).
idx = idx(:);
x = floor((idx - 1) / H) + 1;
y = idx - (x - 1) * H;
xy = [x y];
end
