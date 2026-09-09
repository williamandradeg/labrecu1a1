#!/usr/bin/env bash
python3 - <<'PY'
import os,signal,time
from pathlib import Path
anc={os.getpid()};pid=os.getppid()
while pid>1:
 anc.add(pid)
 try:pid=int(Path(f'/proc/{pid}/stat').read_text().split(')')[1].split()[1])
 except:break
owned=[]
for d in Path('/proc').iterdir():
 if not d.name.isdigit() or int(d.name) in anc:continue
 try:
  env=(d/'environ').read_bytes().split(b'\0')
  if b'GZ_PARTITION=lrrecupera_celda' in env:owned.append(int(d.name))
 except (PermissionError,FileNotFoundError,ProcessLookupError):pass
for pid in owned:
 try:os.kill(pid,signal.SIGTERM)
 except ProcessLookupError:pass
time.sleep(2)
for pid in owned:
 try:os.kill(pid,signal.SIGKILL)
 except ProcessLookupError:pass
print('Procesos de la celda detenidos:',len(owned))
PY
