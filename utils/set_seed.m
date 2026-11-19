function set_seed(s)
%SET_SEED  Reproducible random seed (works in MATLAB and GNU Octave).
s = mod(round(s), 2^31 - 1);
try
    rng(s, 'twister');
catch
    rand('twister', s); %#ok<RAND>
    randn('state', s);  %#ok<RAND>
end
end
