function write_csv(fname, header, rows)
%WRITE_CSV  Toolbox-free CSV writer (works in MATLAB and Octave).
%   header : 1xC cell of column names
%   rows   : RxC cell (char or numeric entries)
fid = fopen(fname, 'w');
if fid < 0, error('write_csv: cannot open %s', fname); end
fprintf(fid, '%s\n', strjoin(header, ','));
for r = 1:size(rows, 1)
    parts = cell(1, size(rows, 2));
    for c = 1:size(rows, 2)
        v = rows{r, c};
        if ischar(v)
            parts{c} = v;
        elseif islogical(v)
            parts{c} = sprintf('%d', v);
        elseif isnan(v)
            parts{c} = 'NaN';
        else
            parts{c} = sprintf('%.6g', v);
        end
    end
    fprintf(fid, '%s\n', strjoin(parts, ','));
end
fclose(fid);
end
