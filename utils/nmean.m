function m = nmean(x)
%NMEAN  Mean ignoring NaN (toolbox-free replacement for nanmean).
x = x(~isnan(x(:)));
if isempty(x), m = NaN; else, m = mean(x); end
end
