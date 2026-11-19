function add_suptitle(f, txt)
%ADD_SUPTITLE  Figure-level title (sgtitle when available, else annotation).
figure(f);
if exist('sgtitle', 'file') || exist('sgtitle', 'builtin')
    sgtitle(txt, 'FontWeight', 'bold', 'FontSize', 13);
else
    annotation(f, 'textbox', [0 0.955 1 0.045], 'String', txt, ...
        'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
        'FontWeight', 'bold', 'FontSize', 13);
end
end
