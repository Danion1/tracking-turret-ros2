#!/bin/bash
# Os dois GIFs do README do rastreio (2026-11-11): Gazebo sem janela, drone em vaivém, os nós do Daniel
# (detectar_alvo, seguir_alvo, comando_torreta) e o roteiro de filmagem (video_rastreio.py).
# Saída: midia/rastreio_webcam.{mp4,gif} (visão limpa da webcam) e midia/rastreio_tiro.{mp4,gif} (de fora).
# Uso: bash ros2/gravar_rastreio.sh     (vários minutos: a simulação espera cada quadro)
# Cerca de RAM: o script roda num cgroup (systemd-run); teto = livre − 600 MB, no máximo 4 500 MB.
if [ -z "$CERCA_RAM" ]; then
    LIVRE=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
    TETO=$(( LIVRE - 600 < 4500 ? LIVRE - 600 : 4500 ))
    [ "$TETO" -lt 1800 ] && { echo "só $LIVRE MB livres (precisa de 1800 + 600 de folga): feche algo e rode de novo"; exit 1; }
    echo "== cerca de RAM: teto $TETO MB ($LIVRE MB livres)"
    exec env CERCA_RAM=1 systemd-run --user --scope -q -p MemoryMax=${TETO}M -p MemorySwapMax=0 bash "$0" "$@"
fi
PROJ="$(cd "$(dirname "$0")" && pwd)"
MIDIA="$PROJ/../midia"; mkdir -p "$MIDIA"
W=$(mktemp -d); mkdir $W/src $W/cams
ln -s "$PROJ/torreta_description" "$PROJ/torreta_nos" $W/src/
export ROS_DOMAIN_ID=77 GZ_PARTITION=rastreio_video_$$
echo "== já rodando antes (não mexer):"; pgrep -a -x "gz|ruby|robot_state_pub" || echo "  nada"
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install > build.log 2>&1 || { tail -20 build.log; exit 1; }
source install/setup.bash
setsid ros2 launch torreta_description rastreio.launch.py gui:=false modo:=vaivem > launch.log 2>&1 &
PID=$!
for i in $(seq 1 60); do sleep 4; N=$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c " active"); [ "$N" -ge 4 ] && break; done
timeout 15 ros2 control list_controllers | awk '{print "  "$1, $3}'
LIB=install/torreta_nos/lib/torreta_nos
setsid $LIB/detectar_alvo --ros-args -p use_sim_time:=true > no1.log 2>&1 & N1=$!
setsid $LIB/seguir_alvo --ros-args -p use_sim_time:=true > no2.log 2>&1 & N2=$!
setsid $LIB/comando_torreta --ros-args -p use_sim_time:=true > no3.log 2>&1 & N3=$!
python3 "$PROJ/video_rastreio.py" gravar $W/cams 3000 > gravar.log 2>&1 & GV=$!
python3 "$PROJ/gravar_dardo.py" gravar $W/dardo.jsonl 3000 & GD=$!
sleep 2
time timeout 600 python3 "$PROJ/video_rastreio.py" roteiro $W/cams
touch $W/cams/PARAR; wait $GV; cat gravar.log
kill $GD 2>/dev/null; sleep 1
echo "== nós do Daniel"; for f in no1.log no2.log no3.log; do head -6 $f | cut -c1-160; done
echo "== montagem"
python3 "$PROJ/video_rastreio.py" montar $W/cams $W/dardo.jsonl "$MIDIA"
for n in $N1 $N2 $N3; do kill -INT -- -$n 2>/dev/null; done; kill -TERM -- -$PID; sleep 10
for n in $N1 $N2 $N3 $PID; do kill -KILL -- -$n 2>/dev/null; done
pgrep -a -x "gz|ruby|robot_state_pub|parameter_bri" || echo "== nada sobrou"
CG=/sys/fs/cgroup$(cut -d: -f3 /proc/self/cgroup)
echo "== pico de memória (cgroup): $(( $(cat $CG/memory.peak 2>/dev/null || echo 0) / 1048576 )) MB"
echo "workspace: $W"
