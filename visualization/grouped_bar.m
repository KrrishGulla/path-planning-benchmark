function grouped_bar(ax, Y, colors, groupNames, ylab)
%GROUPED_BAR  Grouped bar chart: rows of Y = groups, columns = planners.
b = bar(ax, Y, 'grouped');
for k = 1:numel(b)
    set(b(k), 'FaceColor', colors(k, :), 'EdgeColor', 'none');
end
set(ax, 'XTick', 1:size(Y, 1), 'XTickLabel', groupNames);
try, set(ax, 'XTickLabelRotation', 15); catch, end
ylabel(ax, ylab);
grid(ax, 'on');  box(ax, 'on');
end
