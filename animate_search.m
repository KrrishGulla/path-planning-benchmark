function animate_search(envType, seed, gifFile)
%ANIMATE_SEARCH  Side-by-side animation of HOW four planners search.
%   Dijkstra and A* expand cells (watch A* head straight for the goal while
%   Dijkstra floods outwards); RRT and RRT* grow random trees. Every panel
%   reveals the same number of expansions/nodes per frame, so you can see
%   which planner finishes first.
%
%   animate_search                         % rooms map
%   animate_search('maze')
%   animate_search('maze', 7, 'search.gif') % also save an animated GIF

if nargin < 1, envType = 'rooms'; end
if nargin < 2, seed = 7; end
if nargin < 3, gifFile = ''; end
setup_paths();
P = default_params();  if exist('OCTAVE_VERSION','builtin') && isempty(getenv('DISPLAY')), P.showFigures = false; end
e = find(strcmpi(P.envTypes, envType));
if isempty(e), error('Unknown environment "%s".', envType); end
[raw, infl, s, g] = make_scenario(P.envTypes{e}, P, P.baseSeed + seed);

reg = planner_registry();
pick = [1 2 6 7];                                % Dijkstra, A*, RRT, RRT*
res = cell(1, 4);
for k = 1:4
    set_seed(P.baseSeed + pick(k));
    res{k} = reg(pick(k)).fn(infl, s, g, P);
end

f = new_fig(P, 'How planners search', [1100 1000]);
hPts = zeros(1, 4);  hEdge = zeros(1, 4);  ax = zeros(1, 4);
for k = 1:4
    ax(k) = subplot(2, 2, k);
    plot_map(ax(k), raw, infl, s, g);
    c = reg(pick(k)).color;
    hEdge(k) = plot(ax(k), NaN, NaN, '-', 'Color', lighten(c, 0.5), 'LineWidth', 0.6);
    hPts(k)  = plot(ax(k), NaN, NaN, '.', 'Color', lighten(c, 0.25), 'MarkerSize', 6);
    title(ax(k), reg(pick(k)).name, 'FontSize', 12);
end

perFrame = 35;
total = max(cellfun(@(r) size(r.explored, 1), res));
nFrames = ceil(total / perFrame);
done = false(1, 4);
for fr = 1:nFrames
    upto = fr * perFrame;
    for k = 1:4
        E = res{k}.explored;
        m = min(upto, size(E, 1));
        set(hPts(k), 'XData', E(1:m, 1), 'YData', E(1:m, 2));
        if ~isempty(res{k}.edges) && m >= 2
            Ed = res{k}.edges(1:m - 1, :);
            X = [Ed(:, 1) Ed(:, 3) nan(m - 1, 1)]';
            Y = [Ed(:, 2) Ed(:, 4) nan(m - 1, 1)]';
            set(hEdge(k), 'XData', X(:), 'YData', Y(:));
        end
        if ~done(k) && m == size(E, 1)
            done(k) = true;
            if res{k}.success
                plot(ax(k), res{k}.path(:, 1), res{k}.path(:, 2), '-', ...
                    'Color', reg(pick(k)).color, 'LineWidth', 2.5);
                title(ax(k), sprintf('%s - done after %d nodes', ...
                    reg(pick(k)).name, size(E, 1)), 'FontSize', 12);
            else
                title(ax(k), sprintf('%s - FAILED', reg(pick(k)).name), 'FontSize', 12);
            end
        end
    end
    drawnow;
    if ~isempty(gifFile)
        try
            fm = getframe(f);
            [im, map] = rgb2ind(frame2im(fm), 128);
            if fr == 1
                imwrite(im, map, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 0.04);
            else
                imwrite(im, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.04);
            end
        catch err
            warning('GIF export failed: %s', err.message);
            gifFile = '';
        end
    end
    pause(0.01);
end
end
