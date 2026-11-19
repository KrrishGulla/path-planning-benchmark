# Comparative Evaluation of Global Path Planning Algorithms for Mobile Robots

**Krrish Gulla · Reg. No. 230929146 · Mobile Robots and Path Planning – Mini Project**

A toolbox-free MATLAB benchmark that compares **8 global path planners from 4 algorithm
families** on **5 procedurally generated environment classes**, using Monte-Carlo trials,
6 performance criteria, and a **TOPSIS multi-criteria decision layer** that answers the
practical question: *"which planner should I use for my application?"*

---

## 1. Quick start (MATLAB R2018b or newer, no toolboxes)

1. Unzip, open MATLAB, and `cd` into the `GlobalPathPlanningBenchmark` folder.
2. Run **one** of these in the Command Window:

| Command | What it does | Time |
|---|---|---|
| `run_all` | Demo + full benchmark (20 trials/env) + all figures & tables | 3–6 min |
| `main_demo` | All 8 planners on one indoor map, side-by-side | ~5 s |
| `main_demo('maze')` | Same on another map: `random` `forest` `maze` `rooms` `narrow` | ~5 s |
| `main_demo('all')` | One demo for every environment class | ~30 s |
| `interactive_demo('rooms')` | **Click your own start and goal**, then all planners run | ~5 s |
| `animate_search('maze')` | Animation: Dijkstra vs A* vs RRT vs RRT* searching | ~20 s |
| `animate_search('maze',7,'search.gif')` | Same, and save an animated GIF | ~40 s |

For a fast benchmark (≈1 min):

```matlab
P = default_params();  P.trialsPerEnv = 5;
R = run_benchmark(P);
analyze_results(R, P);
```

All outputs go to `results/` (figures in `results/figures/`). The benchmark is fully
reproducible: the same seed always produces the same maps, starts, goals and samples.

> The code also runs in GNU Octave 7+ (used to verify it).
> `sample_results_octave/` contains a complete pre-computed run as a backup.
> Absolute times there come from Octave and are slower than MATLAB; rerun in MATLAB for the final numbers.

---

## 2. Planners compared

| Family | Planner | Key idea | Guarantee |
|---|---|---|---|
| Graph search | **Dijkstra** | Uniform-cost expansion, no heuristic | Optimal on grid, complete |
| Graph search | **A\*** | f = g + h, octile heuristic | Optimal on grid, complete |
| Graph search | **Weighted A\*** | f = g + w·h (w = 2) | ≤ w × optimal, faster |
| Any-angle search | **Theta\*** | A\* + line-of-sight parent shortcuts | Near-optimal, not 45°-restricted |
| Sampling-based | **PRM** | Random roadmap + k-NN + graph search | Probabilistically complete |
| Sampling-based | **RRT** | Random tree, stops at first solution | Probabilistically complete |
| Sampling-based | **RRT\*** | RRT + choose-parent + rewiring | Asymptotically optimal |
| Potential field | **APF** | Gradient descent on attractive + repulsive potential | None (local minima) |

Dijkstra, A\* and Weighted A\* share **one search engine** (`grid_search_core.m`) and differ
only in the heuristic, so their comparison is perfectly fair. RRT and RRT\* get the **same
sampling budget** (4000 iterations).

---

## 3. Environment classes

| Class | Models | What it stresses |
|---|---|---|
| Random Blocks | Warehouse clutter | Baseline |
| Cluttered Forest | Orchard / pillars / crowd | Many small gaps, smoothness |
| Maze | Corridors with dead ends and loops | Local minima, heuristic misguidance |
| Indoor Rooms | 3×3 office with doors and furniture | Typical indoor AMR task |
| Narrow Passage | Walls with 2-cell gaps | The narrow-passage problem of sampling planners |

Maps are 100 × 100 cells. Obstacles are **inflated by the robot radius** (configuration
space) so every planner treats the robot as a point; safety is then measured against the
**true** (non-inflated) obstacles.

---

## 4. Evaluation metrics

| Criterion | Metric | Better |
|---|---|---|
| Reliability | Success rate (%) | Higher |
| Computation | Median planning time (ms) | Lower |
| Optimality | Path length ÷ shortest path found in that trial | Lower (1.00 = best) |
| Smoothness | Total heading change per 100 cells (rad) | Lower |
| Safety | Mean clearance to true obstacles along the path (cells) | Higher |
| Search effort | Expanded cells / roadmap or tree nodes / APF steps | Lower |

Also recorded: number of turning vertices, sharpest turn, minimum clearance.

---

## 5. What is novel in this study

1. **Unified, fair benchmark across four planner families** – same maps, same start/goal,
   same C-space, same collision checker, same metrics, shared search engine.
2. **Quantified environment difficulty** – every scenario gets an obstacle density,
   a *narrowness index* (share of free space within 3 cells of an obstacle) and a
   *tortuosity* (near-optimal path length ÷ straight-line distance) and a
   **bottleneck width** — the widest-path clearance between start and goal, i.e. the
   tightest gap the robot cannot avoid. Global measures like density miss narrow-passage
   difficulty entirely (a narrow-passage map is *sparse*); the bottleneck metric captures
   it. The four combine into a difficulty index used to draw **robustness curves**: how
   each planner degrades as scenarios get harder.
3. **Survivorship-bias correction** – unreliable planners look good if their path quality
   is averaged only over the easy cases they solved. For ranking, every failed query is
   scored with the worst value observed in that environment (PAR-style penalty).
4. **Application-aware ranking with TOPSIS** – instead of "who is best", the study asks
   "best *for what*": four weight profiles (Balanced, Real-time, Safety-critical,
   Path-quality) give four different rankings.
5. **Weight-sensitivity (rank-reversal) analysis** – shows at what weight on speed or on
   safety the recommended planner changes.

---

## 6. Output files

`results/figures/`

| File | Content |
|---|---|
| `fig01_environments.png` | The 5 environment classes + complexity statistics |
| `fig02_path_overlays.png` | Paths of all planners on one map of each class |
| `fig03_success_rate.png` | Success rate per environment |
| `fig04_performance_metrics.png` | Time, path optimality, smoothness, clearance |
| `fig05_tradeoff_time_vs_length.png` | Speed vs path-quality trade-off |
| `fig06_radar_profiles.png` | Normalised capability profile of each planner |
| `fig07_topsis_heatmap.png` | TOPSIS score and rank per environment |
| `fig08_topsis_profiles.png` | Rankings under four application profiles |
| `fig09_topsis_sensitivity.png` | Rank reversal vs criterion weight |
| `fig10_robustness_vs_difficulty.png` | Success & optimality vs scenario difficulty |
| `demo_*_panels.png`, `demo_*_overlay.png` | Single-scenario demo figures |

`results/`

| File | Content |
|---|---|
| `summary_by_environment.csv` | Every metric, per planner per environment |
| `summary_overall.csv` | Every metric, per planner, pooled |
| `topsis_rankings.csv` | Closeness and rank for each application profile |
| `complexity_by_environment.csv` | Density, narrowness, tortuosity, bottleneck width per class |
| `all_trials.csv` | One row per planning query (for your own plots) |
| `key_findings.txt` | Auto-generated summary of the main conclusions |
| `benchmark_results.mat` | Raw result struct `R` |

---

## 7. Project structure

```
GlobalPathPlanningBenchmark/
├── run_all.m               one-click: demo + benchmark + analysis
├── main_demo.m             single-scenario demo
├── interactive_demo.m      click start/goal
├── animate_search.m        search animation (+ optional GIF)
├── run_benchmark.m         Monte-Carlo benchmark
├── analyze_results.m       statistics, TOPSIS, figures, CSV, findings
├── default_params.m        ALL tunable parameters
├── setup_paths.m
├── planners/               grid_search_core, dijkstra, astar, weighted_astar,
│                           thetastar, prm, rrt, rrtstar, apf, grid_moves
├── environment/            generate_map, inflate_map, clearance_map,
│                           sample_start_goal, make_scenario, map_complexity,
│                           reachable_mask
├── metrics/                path_metrics, topsis_rank
├── visualization/          plotting helpers
└── utils/                  registry, line_of_sight, NaN-safe stats, CSV writer
```

## 8. Changing the experiment

Everything is in `default_params.m`, e.g. `P.trialsPerEnv`, `P.mapSize`,
`P.inflateRadius`, `P.wastar.w`, `P.prm.nSamples`, `P.rrtstar.maxIter`, `P.apf.kr`.
To add a planner, write `plan_xxx(map, start, goal, P)` returning `empty_result()` fields
and add it to `utils/planner_registry.m`; every figure and table picks it up automatically.

## 9. References

1. E. W. Dijkstra, "A note on two problems in connexion with graphs," *Numerische Mathematik*, 1959.
2. P. E. Hart, N. J. Nilsson, B. Raphael, "A formal basis for the heuristic determination of minimum cost paths," *IEEE Trans. SSC*, 1968.
3. I. Pohl, "Heuristic search viewed as path finding in a graph," *Artificial Intelligence*, 1970.
4. A. Nash, K. Daniel, S. Koenig, A. Felner, "Theta\*: Any-angle path planning on grids," *AAAI*, 2007.
5. L. E. Kavraki, P. Švestka, J.-C. Latombe, M. H. Overmars, "Probabilistic roadmaps for path planning in high-dimensional configuration spaces," *IEEE T-RA*, 1996.
6. S. M. LaValle, "Rapidly-exploring random trees: A new tool for path planning," TR 98-11, Iowa State Univ., 1998.
7. S. Karaman, E. Frazzoli, "Sampling-based algorithms for optimal motion planning," *IJRR*, 2011.
8. O. Khatib, "Real-time obstacle avoidance for manipulators and mobile robots," *IJRR*, 1986.
9. S. S. Ge, Y. J. Cui, "New potential functions for mobile robot path planning," *IEEE T-RA*, 2000.
10. C.-L. Hwang, K. Yoon, *Multiple Attribute Decision Making: Methods and Applications*, Springer, 1981.
