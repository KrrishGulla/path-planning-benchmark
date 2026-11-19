function [off, stepCost, cA, cB] = grid_moves(H)
%GRID_MOVES  8-connected neighbourhood for a column-major grid of height H.
%   off      : linear-index offsets [N S W E NW NE SW SE]
%   stepCost : 1 for straight moves, sqrt(2) for diagonals
%   cA, cB   : the two orthogonal cells that must be free for each diagonal
%              move (prevents squeezing between diagonal obstacles)
dr  = [-1;  1;  0; 0; -1; -1;  1; 1];
dc  = [ 0;  0; -1; 1; -1;  1; -1; 1];
off = dr + dc * H;
stepCost = [1; 1; 1; 1; sqrt(2) * ones(4, 1)];
cA = off([1; 1; 2; 2]);
cB = off([3; 4; 3; 4]);
end
