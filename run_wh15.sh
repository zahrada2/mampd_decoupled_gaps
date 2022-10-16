#!/bin/bash
yell() { echo "$0: $*" >&2; }
die() { yell "$*"; exit 111; }
try() { "$@" || die "cannot $*"; }

# map type
mapType="warehouse15"
# map instance
map=0
# how many agents of the map to use
agents=120
# how many subgoals of the map to select
goals=480
# offset from automatically calculated capacity
capoffset=0
# number of iterations
numIters=10000
# number of consecutive runs
numRuns=1

outdir=./results
mapname=$mapType"/map"$map"_"$goals

mkdir -p $outdir/$mapType 

tagvar=$mapname"_num_goals_"$goals"_vehicles_"$agents

./cvrp --ttd --method simplified_vns --dir-out $outdir --file-out $tagvar --mapf-out $tagvar --wait-time 0 --mapf-map $mapname -p $mapname -I $numIters --num-run $numRuns --goals $goals --vehicles $agents --pbs --capacity-offset $capoffset
