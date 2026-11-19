function setup_paths()
%SETUP_PATHS  Add all project sub-folders to the MATLAB path.
root = fileparts(mfilename('fullpath'));
subs = {'planners', 'environment', 'metrics', 'visualization', 'utils'};
for k = 1:numel(subs)
    addpath(fullfile(root, subs{k}));
end
addpath(root);
end
