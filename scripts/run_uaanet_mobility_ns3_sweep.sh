#!/usr/bin/env bash
set -euo pipefail

# Pokreni iz root foldera ns-3 instalacije.
# SIM mora biti naziv scratch programa BEZ .cc ekstenzije.
SIM=${SIM:-scratch/uaanet_routing_mobility_ns3}
CSV_FILE=${CSV_FILE:-results/uaanet_mobility/uaanet_mobility_results.csv}

# Fiksiramo broj UAV-ova, a mijenjamo mobility model.
N_UAVS=${N_UAVS:-10}
N_GCS=${N_GCS:-1}
SIM_TIME=${SIM_TIME:-100}
NODE_SPEED=${NODE_SPEED:-20}
NODE_MIN_SPEED=${NODE_MIN_SPEED:-10}
NODE_PAUSE=${NODE_PAUSE:-2}
TX_RANGE=${TX_RANGE:-300}

# Dodatni mobility parametri.
RANDOM_WALK_TIME=${RANDOM_WALK_TIME:-1}
GAUSS_MARKOV_ALPHA=${GAUSS_MARKOV_ALPHA:-0.85}
GAUSS_MARKOV_TIMESTEP=${GAUSS_MARKOV_TIMESTEP:-1}
GROUP_RADIUS=${GROUP_RADIUS:-50}

# Možeš pregaziti iz terminala, npr:
# PROTOCOLS="AODV_ETX QSPU" RUNS="1 2 3" ./run_uaanet_mobility_ns3_sweep.sh
PROTOCOLS=${PROTOCOLS:-"AODV OLSR DSDV AODV_ETX QSPU"}
MOBILITY_MODELS=${MOBILITY_MODELS:-"RWP RWALK2D GAUSS_MARKOV GROUP"}
RUNS=${RUNS:-"1 2 3 4 5"}

mkdir -p "$(dirname "${CSV_FILE}")"
rm -f "${CSV_FILE}"

echo "CSV_FILE=${CSV_FILE}"
echo "N_UAVS=${N_UAVS}, NODE_SPEED=${NODE_SPEED}, NODE_MIN_SPEED=${NODE_MIN_SPEED}"
echo "PROTOCOLS=${PROTOCOLS}"
echo "MOBILITY_MODELS=${MOBILITY_MODELS}"
echo "RUNS=${RUNS}"

for PROTO in ${PROTOCOLS}; do
  for MOB in ${MOBILITY_MODELS}; do
    for run in ${RUNS}; do
      echo "Pokrecem: protocol=${PROTO}, mobilityModel=${MOB}, nUavs=${N_UAVS}, run=${run}"

      ./ns3 run "${SIM} \
        --routingProtocol=${PROTO} \
        --mobilityModel=${MOB} \
        --nGcs=${N_GCS} \
        --nUavs=${N_UAVS} \
        --run=${run} \
        --simTime=${SIM_TIME} \
        --nodeSpeed=${NODE_SPEED} \
        --nodeMinSpeed=${NODE_MIN_SPEED} \
        --fixedNodeSpeed=0 \
        --nodePause=${NODE_PAUSE} \
        --txRange=${TX_RANGE} \
        --randomWalkTime=${RANDOM_WALK_TIME} \
        --gaussMarkovAlpha=${GAUSS_MARKOV_ALPHA} \
        --gaussMarkovTimeStep=${GAUSS_MARKOV_TIMESTEP} \
        --groupRadius=${GROUP_RADIUS} \
        --csvFile=${CSV_FILE}"
    done
  done
done

echo "Gotovo. CSV rezultati su u: ${CSV_FILE}"
echo "Za grafike pokreni:"
echo "python3 analyze_uaanet_mobility_ns3_sweep.py --csv ${CSV_FILE} --out-dir results/uaanet_mobility/analysis"
