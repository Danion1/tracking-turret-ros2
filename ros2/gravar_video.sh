#!/bin/bash
# Vídeo dos mecanismos da torreta (Gazebo sem janela, câmeras simuladas), comandado pelo nó
# comando_torreta do Daniel. Saída: midia/torreta.mp4 e midia/torreta.gif.
# Uso: bash ros2/gravar_video.sh     (de qualquer pasta; vários minutos: a simulação espera cada quadro)
# Cerca de RAM (2026-11-10/11): o script roda num cgroup (systemd-run); passou do teto, o kernel mata só ele.
# O teto tem de ficar ABAIXO da memória livre (senão o WSL inteiro esgota antes de o teto agir):
# teto = livre − 600 MB de folga para o resto do sistema, no máximo 4 500 MB.
if [ -z "$CERCA_RAM" ]; then
    LIVRE=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
    TETO=$(( LIVRE - 600 < 4500 ? LIVRE - 600 : 4500 ))
    [ "$TETO" -lt 3500 ] && { echo "só $LIVRE MB livres (o teste precisa de 3500 + 600 de folga): feche algo e rode de novo"; exit 1; }
    echo "== cerca de RAM: teto $TETO MB ($LIVRE MB livres)"
    exec env CERCA_RAM=1 systemd-run --user --scope -q -p MemoryMax=${TETO}M -p MemorySwapMax=0 bash "$0" "$@"
fi
PROJ="$(cd "$(dirname "$0")" && pwd)"
MIDIA="$PROJ/../midia"; mkdir -p "$MIDIA"
W=$(mktemp -d); mkdir $W/src $W/cams
ln -s "$PROJ/torreta_description" "$PROJ/torreta_nos" $W/src/
export ROS_DOMAIN_ID=77 GZ_PARTITION=video_teste_$$
echo "== já rodando antes (não mexer):"; pgrep -a -x "gz|ruby|robot_state_pub" || echo "  nada"
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install > build.log 2>&1 || { tail -20 build.log; exit 1; }
source install/setup.bash
SHARE=install/torreta_description/share/torreta_description
python3 "$PROJ/video_torreta.py" mundo $SHARE/worlds/torreta.sdf $W/mundo_video.sdf
setsid ros2 launch torreta_description gazebo.launch.py gui:=false mundo:=$W/mundo_video.sdf urdf:=torreta_video.urdf > launch.log 2>&1 &
PID=$!
for i in $(seq 1 60); do sleep 4; N=$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c " active"); [ "$N" -ge 4 ] && break; done
timeout 15 ros2 control list_controllers | awk '{print "  "$1, $3}'
setsid ros2 run torreta_nos comando_torreta --ros-args -p use_sim_time:=true > no.log 2>&1 &
NO=$!
python3 "$PROJ/video_torreta.py" gravar $W/cams 3000 > gravar.log 2>&1 &
GV=$!
python3 "$PROJ/gravar_dardo.py" gravar $W/dardo.jsonl 3000 &
GD=$!
sleep 2
time python3 "$PROJ/video_torreta.py" roteiro
touch $W/cams/PARAR; wait $GV; cat gravar.log
kill $GD 2>/dev/null
echo "== nó do Daniel"; grep -v Traceback no.log | head -12
echo "== simulador dos volantes"; grep -E "LANÇADO|chão" launch.log | sed 's/.*simular_volantes\]: //'
echo "== montagem"
python3 "$PROJ/video_torreta.py" montar $W/cams $W/dardo.jsonl "$MIDIA/torreta.mp4" "$MIDIA/torreta.gif"
kill -INT -- -$NO 2>/dev/null; kill -TERM -- -$PID; sleep 10; kill -KILL -- -$NO -$PID 2>/dev/null
pgrep -a -x "gz|ruby|robot_state_pub|parameter_bri" || echo "== nada sobrou"
echo "workspace: $W"
