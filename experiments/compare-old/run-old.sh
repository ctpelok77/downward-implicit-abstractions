#!/bin/bash
# Runs the old planner (FD-implicit-lp, built with GCC 4.9 in a container)
# on one task and strategy; prints "h0 cost exit".
# Usage: run-old.sh <old-src-dir> <benchmarks-dir> <domain/problem> <strategy> <time-limit-s>
OLD=$1; BENCH=$2; TASK=$3; S=$4; TL=$5
DOM=$(dirname $TASK); PROB=$(basename $TASK)
podman run --rm --arch amd64 -v $OLD:/src:ro -v $BENCH:/bench:ro docker.io/library/gcc:4.9 bash -c "
cd /tmp
D=/bench/$DOM/domain.pddl
[ -f \$D ] || D=/bench/$DOM/domain_$PROB.pddl
[ -f \$D ] || D=/bench/$DOM/\$(echo $PROB | cut -d- -f1)-domain.pddl
python2 /src/translate/translate.py \$D /bench/$TASK.pddl >/dev/null 2>&1 || { echo 'None None translate-fail'; exit; }
/src/preprocess/preprocess < output.sas >/dev/null 2>&1 || { echo 'None None preprocess-fail'; exit; }
timeout $TL /src/search/downward-release --search 'astar(implicit(strategy=$S))' < output > out.txt 2>&1
e=\$?
h=\$(grep -o 'Initial state h value: [0-9.]*' out.txt | grep -o '[0-9.]*\$' | head -1)
c=\$(grep -o 'Plan cost: [0-9]*' out.txt | grep -o '[0-9]*\$' | head -1)
[ \$e = 124 ] && e=timeout
echo \"\${h:-None} \${c:-None} \$e\"
"
