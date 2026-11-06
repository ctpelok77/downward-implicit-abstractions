# Implicit abstractions vs. LM-cut

`implicit-vs-lmcut.py` runs A* with the ported implicit abstractions heuristic
(strategies 0–4: forks, inverted forks, both, semiforks, hourglasses), LM-cut
and blind search on `DEFAULT_OPTIMAL_SUITE` (65 domains), with 30 minutes and
3584 MB per run. All algorithms are built from the commit in `REV`.

Requirements:
- lab (`pip install lab downward`), Python 3, CMake and a C++ compiler.
- `DOWNWARD_BENCHMARKS`: a checkout of https://github.com/aibasel/downward-benchmarks.
- The commit `REV` must exist in the repository this script lives in.

Running:

    export DOWNWARD_BENCHMARKS=...
    python implicit-vs-lmcut.py --test yes --all   # a few tasks, to test the setup
    python implicit-vs-lmcut.py --all              # the full experiment

`IA_PROCESSES` (default 60), `IA_TIME_LIMIT` and `IA_MEMORY_LIMIT` override
the parallelism and limits.

Results in `data/implicit-vs-lmcut-eval/`: the absolute report, one
comparison report per strategy against LM-cut, and scatter plots of
`expansions_until_last_jump` and `initial_h_value`.
