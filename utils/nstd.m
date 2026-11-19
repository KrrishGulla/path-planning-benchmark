function s = nstd(x)
%NSTD  Standard deviation ignoring NaN (toolbox-free replacement for nanstd).
x = x(~isnan(x(:)));
if numel(x) < 2, s = 0; else, s = std(x); end
end
