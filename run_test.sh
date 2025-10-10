#!/usr/bin/env bash
set -euo pipefail

for i in {1..5}; do
  echo "Running test case $i"
  ~/tool/julia/julia bus2_opt.jl "case/tc_${i}.json" > /dev/null
  python test/test_case.py --case "solution/tc_${i}.json" result.json 
done