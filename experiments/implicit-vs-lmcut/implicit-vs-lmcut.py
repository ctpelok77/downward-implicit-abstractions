#!/usr/bin/env python3
"""
Experiment: the ported implicit abstractions heuristic against LM-cut.

All algorithms are A* with the release build of REV. The implicit
abstractions configurations differ in the decomposition strategy:

    ia-forks        implicit_abstractions(strategy=0)  forks
    ia-iforks       implicit_abstractions(strategy=1)  inverted forks
    ia-both         implicit_abstractions(strategy=2)  forks and inverted forks
    ia-semiforks    implicit_abstractions(strategy=3)  semiforks
    ia-hourglasses  implicit_abstractions(strategy=4)  hourglasses

all with the default uniform cost partitioning. lmcut is the baseline and
blind a sanity check. The LP-based configurations are not included (the
build has no LP solver).

Requirements:
    pip install lab downward
    export DOWNWARD_BENCHMARKS=/path/to/downward-benchmarks
    The planner is built from the current HEAD (override with IA_REV).

Usage:
    python implicit-vs-lmcut.py --test yes --all   # a few tasks, to test the setup
    python implicit-vs-lmcut.py --all              # build, start, parse, fetch, reports
    python implicit-vs-lmcut.py 1 2 3              # or single steps (see --help)

Environment overrides: IA_PROCESSES (default 60), IA_TIME_LIMIT (default 30m),
IA_MEMORY_LIMIT (default 3584M), IA_REV (default HEAD).
"""

import os
import subprocess

from lab.environments import LocalEnvironment
from downward.reports.absolute import AbsoluteReport
from downward.reports.compare import ComparativeReport
from downward.reports.scatter import ScatterPlotReport

import common_setup
from common_setup import IssueConfig, IssueExperiment

BENCHMARKS_DIR = os.environ["DOWNWARD_BENCHMARKS"]
BUILD = "release"

# The commit all algorithms are built from (also used in the algorithm names):
# IA_REV if set, otherwise the current HEAD of this repository.
REV = os.environ.get("IA_REV") or subprocess.check_output(
    ["git", "rev-parse", "--short=9", "HEAD"],
    cwd=os.path.dirname(os.path.abspath(__file__)), text=True).strip()

# ---------------------------------------------------------------------------
# Configurations
# ---------------------------------------------------------------------------

STRATEGIES = [
    ("ia-forks", 0),
    ("ia-iforks", 1),
    ("ia-both", 2),
    ("ia-semiforks", 3),
    ("ia-hourglasses", 4),
]

CONFIGS = [
    ("lmcut", ["--search", "astar(lmcut())"]),
    ("blind", ["--search", "astar(blind())"]),
] + [
    (nick, ["--search", "astar(implicit_abstractions(strategy=%d))" % strategy])
    for nick, strategy in STRATEGIES
]

# ---------------------------------------------------------------------------
# Suite
# ---------------------------------------------------------------------------

SUITE = common_setup.DEFAULT_OPTIMAL_SUITE
if common_setup.is_test_run():
    SUITE = ["gripper:prob01.pddl", "gripper:prob02.pddl",
             "miconic:s1-0.pddl", "miconic:s2-0.pddl", "depot:p01.pddl"]

# ---------------------------------------------------------------------------
# Experiment
# ---------------------------------------------------------------------------

ENVIRONMENT = LocalEnvironment(
    processes=int(os.environ.get("IA_PROCESSES", "60")))

exp = IssueExperiment(
    revisions=[REV],
    configs=[
        IssueConfig(
            config_nick,
            component_options,
            build_options=[BUILD],
            driver_options=["--build", BUILD])
        for config_nick, component_options in CONFIGS
    ],
    environment=ENVIRONMENT,
    time_limit=os.environ.get("IA_TIME_LIMIT", "30m"),
    memory_limit=os.environ.get("IA_MEMORY_LIMIT", "3584M"),
)
exp.add_suite(BENCHMARKS_DIR, SUITE)

exp.add_parser(exp.EXITCODE_PARSER)
exp.add_parser(exp.TRANSLATOR_PARSER)
exp.add_parser(exp.SINGLE_SEARCH_PARSER)
exp.add_parser(exp.PLANNER_PARSER)

exp.add_step("build", exp.build)
exp.add_step("start", exp.start_runs)
exp.add_step("parse", exp.parse)
exp.add_fetcher(name="fetch")

ATTRIBUTES = exp.DEFAULT_TABLE_ATTRIBUTES + ["initial_h_value"]

# Not add_absolute_report_step: its "publish" step fails where publish is not
# installed, which would stop the remaining reports.
exp.add_report(
    AbsoluteReport(attributes=ATTRIBUTES),
    outfile=os.path.join(exp.eval_dir, "implicit-vs-lmcut.html"))

LMCUT = common_setup.get_algo_nick(REV, "lmcut")
for nick, _ in STRATEGIES:
    other = common_setup.get_algo_nick(REV, nick)
    exp.add_report(
        ComparativeReport([(LMCUT, other, "Diff")], attributes=ATTRIBUTES),
        name="compare-lmcut-vs-%s" % nick,
        outfile=os.path.join(exp.eval_dir, "compare-lmcut-vs-%s.html" % nick))

for attribute in ["expansions_until_last_jump", "initial_h_value"]:
    for nick, _ in STRATEGIES:
        other = common_setup.get_algo_nick(REV, nick)
        exp.add_report(
            ScatterPlotReport(
                attributes=[attribute],
                filter_algorithm=[LMCUT, other],
                get_category=lambda run1, run2: run1["domain"],
                format="png"),
            name="scatter-%s-lmcut-vs-%s" % (attribute, nick),
            outfile=os.path.join(
                exp.eval_dir, "scatter-%s-lmcut-vs-%s.png" % (attribute, nick)))

exp.run_steps()
