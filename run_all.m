%% RUN_ALL  One-click entry point for the whole project.
%
%  Comparative Evaluation of Global Path Planning Algorithms for Mobile Robots
%  (Dijkstra, A*, Weighted A*, Theta*, PRM, RRT, RRT*, APF)
%
%  1. Live demo on one indoor map (2x4 planner panels + overlay)
%  2. Monte-Carlo benchmark over 5 environment classes
%  3. Statistics, 10 figures, CSV tables, TOPSIS ranking, key findings
%
%  Everything is written to ./results  (figures in ./results/figures)
%  Requires: base MATLAB R2018b or newer, NO toolboxes. (Also runs in GNU Octave 7+.)

clc; close all;
setup_paths();

P = default_params();
P.trialsPerEnv = 20;      % 20 -> ~3-6 min in MATLAB.  Use 5 for a quick 1-min run.

%% 1) Single-scenario demo
main_demo('rooms');

%% 2) Benchmark
R = run_benchmark(P);

%% 3) Analysis + figures + tables + key findings
S = analyze_results(R, P);
