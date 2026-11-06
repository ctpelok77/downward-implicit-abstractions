#!/usr/bin/env python3
"""
Regression check for the implicit abstractions heuristic.

Runs A* with implicit_abstractions(strategy=0..4) on a fixed set of small
tasks, records h(s0), plan cost, expansions until the last jump and the exit
code, and either writes them to a baseline file or compares against one.

    python check-values.py record baseline.json [--build release]
    python check-values.py compare baseline.json [--build debug]

Runs in parallel (-j, default: number of CPUs). Each run has a time limit
(--time-limit, default 60s); runs that time out are recorded as such and
compared like any other result.
"""

import argparse
import json
import multiprocessing
import os
import re
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BENCHMARKS = os.environ.get(
    "DOWNWARD_BENCHMARKS", os.path.expanduser("~/software/downward-benchmarks"))

TASKS = [
    "blocks/probBLOCKS-4-0", "blocks/probBLOCKS-5-0", "blocks/probBLOCKS-6-0",
    "logistics00/probLOGISTICS-4-0", "logistics00/probLOGISTICS-5-0",
    "driverlog/p01", "driverlog/p02", "driverlog/p03",
    "depot/p01", "depot/p02",
    "satellite/p01-pfile1", "satellite/p02-pfile2",
    "rovers/p01", "rovers/p02", "rovers/p03",
    "zenotravel/p01", "zenotravel/p02", "zenotravel/p03",
    "tpp/p01", "tpp/p02", "tpp/p03",
    "storage/p01", "storage/p02", "storage/p03",
    "miconic/s1-0", "miconic/s2-0", "miconic/s3-0",
    "gripper/prob01", "gripper/prob02",
    "freecell/p01", "freecell/p02",
    "elevators-opt08-strips/p01",
    "mprime/prob01",
    "psr-small/p01-s2-n1-l2-f50", "psr-small/p02-s5-n1-l3-f30",
]
STRATEGIES = [0, 1, 2, 3, 4]

PATTERNS = {
    "h0": r"Initial heuristic value for \w+: (\S+)",
    "cost": r"Plan cost: (\d+)",
    "expansions": r"Expanded until last jump: (\d+) state",
    "exit": r"search exit code: (-?\d+)",
}


def domain_file(task):
    dom, prob = task.split("/")
    for name in ["domain.pddl", "domain_" + prob + ".pddl",
                 prob.split("-")[0] + "-domain.pddl"]:
        path = os.path.join(BENCHMARKS, dom, name)
        if os.path.exists(path):
            return path
    sys.exit("no domain file for " + task)


def run(args):
    task, strategy, build, time_limit = args
    problem = os.path.join(BENCHMARKS, task + ".pddl")
    cmd = [sys.executable, os.path.join(REPO, "fast-downward.py"),
           "--build", build, "--search-time-limit", "%ds" % time_limit,
           domain_file(task), problem,
           "--search", "astar(implicit_abstractions(strategy=%d))" % strategy]
    with tempfile.TemporaryDirectory() as tmp:
        try:
            out = subprocess.run(cmd, cwd=tmp, capture_output=True, text=True,
                                 timeout=time_limit + 120).stdout
        except subprocess.TimeoutExpired:
            out = ""
    result = {}
    for key, pattern in PATTERNS.items():
        m = re.search(pattern, out)
        result[key] = m.group(1) if m else None
    if result["exit"] is None:
        m = re.search(r"translate exit code: (-?\d+)", out)
        result["exit"] = ("translate-" + m.group(1)) if m and m.group(1) != "0" else "timeout"
    return "%s s=%d" % (task, strategy), result


def run_all(build, jobs, time_limit):
    work = [(t, s, build, time_limit) for t in TASKS for s in STRATEGIES]
    with multiprocessing.Pool(jobs) as pool:
        return dict(pool.map(run, work))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["record", "compare"])
    parser.add_argument("baseline")
    parser.add_argument("--build", default="release")
    parser.add_argument("-j", type=int, default=multiprocessing.cpu_count())
    parser.add_argument("--time-limit", type=int, default=60)
    args = parser.parse_args()

    results = run_all(args.build, args.j, args.time_limit)

    if args.mode == "record":
        with open(args.baseline, "w") as f:
            json.dump(results, f, indent=1, sort_keys=True)
        exits = {}
        for r in results.values():
            exits[r["exit"]] = exits.get(r["exit"], 0) + 1
        print("recorded %d runs, exit codes: %s" % (len(results), exits))
        return

    with open(args.baseline) as f:
        baseline = json.load(f)
    diffs = [(k, baseline.get(k), results[k]) for k in sorted(results)
             if baseline.get(k) != results[k]]
    for key, old, new in diffs:
        print("DIFF %s\n  baseline: %s\n  now:      %s" % (key, old, new))
    print("%d runs, %d differences" % (len(results), len(diffs)))
    sys.exit(1 if diffs else 0)


if __name__ == "__main__":
    main()
