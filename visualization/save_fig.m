function save_fig(f, P, fname)
%SAVE_FIG  Save a figure as a 150-dpi PNG into P.figDir.
if ~isfield(P, 'saveFigures') || ~P.saveFigures, return; end
if ~exist(P.figDir, 'dir'), mkdir(P.figDir); end
set(f, 'PaperPositionMode', 'auto');
print(f, fullfile(P.figDir, fname), '-dpng', '-r150');
end
