#!/usr/bin/env bash
set -euo pipefail

# Pokreni iz root foldera ns-3 instalacije.
# Ako koristiš fajl scratch/uaanet_routing_speed.cc, ostavi SIM ovako.
# Ako zamijeniš postojeći scratch/uaanet_routing.cc, pokreni sa:
#   SIM=scratch/uaanet_routing ./run_uaanet_speed_sweep.sh
SIM=${SIM:-scratch/uaanet_routing}
CSV_FILE=${CSV_FILE:-results/uaanet_speed/uaanet_speed_results.csv}
ANALYZER=${ANALYZER:-analyze_uaanet_speed_sweep.py}
OUT_DIR=${OUT_DIR:-results/uaanet_speed/analysis}

# Fiksiramo broj UAV-ova, a mijenjamo brzinu.
# Za kraće izvršavanje smanji RUNS ili PROTOCOLS.
N_UAVS=${N_UAVS:-10}
SIM_TIME=${SIM_TIME:-100}
NODE_PAUSE=${NODE_PAUSE:-2.0}
TX_RANGE=${TX_RANGE:-300}

# Brzine u m/s. 5 m/s = 18 km/h, 30 m/s = 108 km/h.
NODE_SPEEDS=${NODE_SPEEDS:-"5 10 20 30"}
RUNS=${RUNS:-"1 2 3 4 5"}

# Preporuka za finalni rad: AODV, AODV_ETX i QSPU.
# OLSR/DSDV uključi samo ako baš želiš dodatno poređenje, jer produžavaju vrijeme.
PROTOCOLS=(${PROTOCOLS:-"AODV OLSR DSDV AODV_ETX QSPU"})

mkdir -p "$(dirname "${CSV_FILE}")" "${OUT_DIR}"
rm -f "${CSV_FILE}"

for PROTO in "${PROTOCOLS[@]}"; do
  for SPEED in ${NODE_SPEEDS}; do
    for RUN in ${RUNS}; do
      echo "Pokrecem: protocol=${PROTO}, nUavs=${N_UAVS}, speed=${SPEED} m/s, run=${RUN}"

      ./ns3 run "${SIM} \
        --routingProtocol=${PROTO} \
        --nUavs=${N_UAVS} \
        --nodeSpeed=${SPEED} \
        --nodeMinSpeed=${SPEED} \
        --fixedNodeSpeed=true \
        --nodePause=${NODE_PAUSE} \
        --txRange=${TX_RANGE} \
        --simTime=${SIM_TIME} \
        --run=${RUN} \
        --csvFile=${CSV_FILE}"
    done
  done
done

echo "Gotovo. CSV rezultati su u: ${CSV_FILE}"

if [[ -f "${ANALYZER}" ]]; then
  echo "Pokrecem analizu: ${ANALYZER}"
  python3 "${ANALYZER}" --csv "${CSV_FILE}" --out-dir "${OUT_DIR}"
  echo "Grafici su u: ${OUT_DIR}"
else
  echo "Analyzer ${ANALYZER} nije pronadjen u trenutnom folderu. Pokreni ga rucno ako treba."
fi
