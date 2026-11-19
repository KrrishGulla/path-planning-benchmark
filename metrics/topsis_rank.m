function [closeness, rank] = topsis_rank(X, w, isBenefit)
%TOPSIS_RANK  Technique for Order of Preference by Similarity to Ideal Solution
%   (Hwang & Yoon, 1981) - multi-criteria decision making.
%
%   X         : [alternatives x criteria] decision matrix (no NaN)
%   w         : 1 x criteria weights (normalised to sum 1 here)
%   isBenefit : 1 x criteria logical (true = larger is better)
%
%   closeness : relative closeness to the ideal solution, in [0, 1]
%   rank      : 1 = best alternative
%
%   Steps: vector-normalise each criterion -> apply weights -> find the
%   positive-ideal (best of each column) and negative-ideal (worst of each
%   column) -> Euclidean distance of each alternative to both ->
%   closeness = d- / (d+ + d-).
w = w(:)' / sum(w);
den = sqrt(sum(X.^2, 1));
den(den == 0) = 1;
V = bsxfun(@times, bsxfun(@rdivide, X, den), w);

ideal = max(V, [], 1);   anti = min(V, [], 1);
ideal(~isBenefit) = min(V(:, ~isBenefit), [], 1);
anti(~isBenefit)  = max(V(:, ~isBenefit), [], 1);

dPlus  = sqrt(sum(bsxfun(@minus, V, ideal).^2, 2));
dMinus = sqrt(sum(bsxfun(@minus, V, anti).^2, 2));
closeness = dMinus ./ max(dPlus + dMinus, eps);

[~, order] = sort(closeness, 'descend');
rank = zeros(size(closeness));
rank(order) = 1:numel(order);
end
