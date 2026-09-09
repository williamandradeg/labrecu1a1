#!/usr/bin/env bash
set -e
cd "$HOME/lrrecupera"
./detener.sh
source entorno.sh
export DISPLAY="${DISPLAY:-:0}"
if [ -z "${XAUTHORITY:-}" ]; then
 export XAUTHORITY=$(find /run/user/$(id -u) -maxdepth 1 -name '.mutter-Xwaylandauth.*' | head -1)
fi
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1
export LP_NUM_THREADS=2
export QT_X11_NO_MITSHM=1
python3 - <<'PY'
from pathlib import Path
from datetime import datetime
import shutil,xml.etree.ElementTree as E
r=Path.home()/'lrrecupera'
if (r/'evidencias/resultado.json').exists():
 dst=r/'runs'/datetime.now().strftime('%Y%m%d_%H%M%S')
 dst.mkdir(parents=True,exist_ok=True)
 for p in (r/'evidencias').iterdir():
  if p.is_file():shutil.copy2(p,dst/p.name)
tree=E.parse(r/'src/celda_robotica/worlds/celda.sdf')
w=tree.getroot().find('world')
camera=w.find("model[@name='evidence_camera']")
if camera is not None:w.remove(camera)
for plug in list(w.findall('plugin')):
 if plug.get('name')=='gz::sim::systems::Sensors':w.remove(plug)
tree.write(r/'logs/celda_video.sdf',encoding='unicode')
PY
unset CELDA_WORLD
if [ "${1:-}" = "--video" ]; then
 export CELDA_WORLD="$HOME/lrrecupera/logs/celda_video.sdf"
fi
printf '00 ESCENA Lista para iniciar\n' > evidencias/estado.txt
nohup ros2 launch celda_robotica celda.launch.py > logs/server.log 2>&1 < /dev/null &
echo $! > logs/launch.pid
echo 'Celda iniciada. Para ejecutar el proceso: ./ejecutar_ciclo.sh'