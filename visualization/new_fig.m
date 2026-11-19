function f = new_fig(P, name, sz)
%NEW_FIG  Create a white figure of size sz = [w h] pixels.
if nargin < 3, sz = [1200 700]; end
vis = 'on';
if isfield(P, 'showFigures') && ~P.showFigures, vis = 'off'; end
f = figure('Name', name, 'NumberTitle', 'off', 'Color', 'w', ...
           'Visible', vis, 'Position', [40 40 sz]);
end
