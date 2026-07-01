#!/usr/bin/env bash
set -euo pipefail

# Pokreni iz root foldera ns-3 instalacije.
# SIM mora biti naziv scratch programa BEZ .cc ekstenzije.
SIM=${SIM:-scratch/uaanet_routing}
CSV_FILE=${CSV_FILE:-results/aodv_scaling/aodv_scaling_results.csv}


mkdir -p "$(dirname "${CSV_FILE}")"
rm -f "${CSV_FILE}"

UAV_COUNTS="5 8 10 12 15"
RUNS="1 2 3 4 5"
PROTOCOLS=("AODV" "OLSR" "DSDV" "AODV_ETX" "QSPU")
for PROTO in "${PROTOCOLS[@]}"; do
  for n in ${UAV_COUNTS}; do
    for run in ${RUNS}; do
      echo "Pokrecem: ${PROTO}, nUavs=${n}, run=${run}"

      ./ns3 run "${SIM} \
        --routingProtocol=${PROTO} \
        --nUavs=${n} \
        --run=${run} \
        --csvFile=${CSV_FILE}"
    done
  done
done

echo "Gotovo. CSV rezultati su u: ${CSV_FILE}"
