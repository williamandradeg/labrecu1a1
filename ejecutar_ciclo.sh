#!/usr/bin/env bash
set -eo pipefail
cd "$HOME/lrrecupera"
source entorno.sh
exec 9>logs/cycle.lock
flock -n 9 || { echo 'Ya hay un ciclo en ejecucion'; exit 1; }
python3 src/celda_robotica/scripts/cycle.py 2>&1 | tee logs/cycle.log
