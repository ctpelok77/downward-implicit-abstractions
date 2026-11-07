#!/usr/bin/env python3
"""
Compare the ported implicit abstractions heuristic against the old version
(FD-implicit-lp) on the regression task set: h(s0) and plan cost per task
and strategy.

The old planner must be built with GCC 4.9 (it uses tr1 and ext/ headers);
run-old.sh runs it in the docker.io/library/gcc:4.9 container (amd64) via
podman. Build it once:

    podman run --rm --arch amd64 -v OLD_SRC:/src -w /src/search \
        docker.io/library/gcc:4.9 make release DOWNWARD_BITWIDTH=64
    podman run --rm --arch amd64 -v OLD_SRC:/src -w /src/preprocess \
        docker.io/library/gcc:4.9 make DOWNWARD_BITWIDTH=64

where OLD_SRC is a copy of FD-implicit-lp/src. Then:

    python compare-old.py OLD_SRC new-values.json [-j 2]

new-values.json is a file written by ../regression/check-values.py record.
Expansions are not compared: the A* tie-breaking differs between the two
Fast Downward versions.
"""

import argparse
import json
import multiprocessing
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BENCHMARKS = os.environ.get(
    "DOWNWARD_BENCHMARKS", os.path.expanduser("~/software/downward-benchmarks"))


def run_old(args):
    old_src, key, time_limit = args
    task, strategy = key.split(" s=")
    out = subprocess.run(
        [os.path.join(HERE, "run-old.sh"), old_src, BENCHMARKS, task,
         strategy, str(time_limit)],
        capture_output=True, text=True).stdout.split()
    h0, cost, exit_code = (out + ["None"] * 3)[:3]
    return key, {"h0": None if h0 == "None" else h0.rstrip("."),
                 "cost": None if cost == "None" else cost,
                 "exit": exit_code}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("old_src")
    parser.add_argument("new_values")
    parser.add_argument("-j", type=int, default=2)
    parser.add_argument("--time-limit", type=int, default=60)
    parser.add_argument("--save", help="write old results to this JSON file")
    args = parser.parse_args()

    with open(args.new_values) as f:
        new = json.load(f)
    work = [(os.path.abspath(args.old_src), key, args.time_limit)
            for key in sorted(new)]
    with multiprocessing.Pool(args.j) as pool:
        old = dict(pool.map(run_old, work))
    if args.save:
        with open(args.save, "w") as f:
            json.dump(old, f, indent=1, sort_keys=True)

    counts = {"same": 0, "higher": 0, "lower": 0, "only-new": 0,
              "only-old": 0, "neither": 0, "cost-diff": 0}
    lines = []
    for key in sorted(new):
        n, o = new[key], old[key]
        nh = float(n["h0"]) if n["h0"] is not None else None
        oh = float(o["h0"]) if o["h0"] is not None else None
        if nh is None and oh is None:
            kind = "neither"
        elif oh is None:
            kind = "only-new"
        elif nh is None:
            kind = "only-old"
        elif nh == oh:
            kind = "same"
        else:
            kind = "higher" if nh > oh else "lower"
        counts[kind] += 1
        cost_diff = (n["cost"] is not None and o["cost"] is not None
                     and n["cost"] != o["cost"])
        if cost_diff:
            counts["cost-diff"] += 1
        if kind != "same" or cost_diff:
            lines.append("%-40s %-9s new h0=%s cost=%s exit=%s | old h0=%s cost=%s exit=%s"
                         % (key, kind, n["h0"], n["cost"], n["exit"],
                            o["h0"], o["cost"], o["exit"]))
    for line in lines:
        print(line)
    print("h0 vs old: %(same)d same, %(higher)d higher, %(lower)d lower, "
          "%(only-new)d only new, %(only-old)d only old, %(neither)d neither; "
          "%(cost-diff)d plan cost differences" % counts)
    sys.exit(1 if counts["lower"] or counts["cost-diff"] else 0)


if __name__ == "__main__":
    main()
