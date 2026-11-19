function m = nmedian(x)
%NMEDIAN  Median ignoring NaN.
x = x(~isnan(x(:)));
if isempty(x), m = NaN; else, m = median(x); end
end
