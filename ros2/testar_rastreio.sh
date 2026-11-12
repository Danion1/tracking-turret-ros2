#!/bin/bash
# Conferência da webcam simulada + alvo (2026-11-10): Gazebo sem janela, rastreio.launch.py, o nó
# comando_torreta do Daniel (para a mira do meio do teste) e o conferir_camera.py.
# Uso: bash ros2/testar_rastreio.sh [modo] [segundos_simulados] [nos] [distancia_m]   (padrão: vaivem 10 mira 3.0)
#   nos = mira      comando_torreta + mira de 15°/8° no meio (confere a câmera presa no berço)
#         detectar  + detectar_alvo do Daniel, torreta parada (erro da detecção contra a verdade)
#         seguir    detectar_alvo + seguir_alvo do Daniel (rastreio em malha fechada; sem comando_torreta)
#         disparo   seguir + comando_torreta; dispara em DUR − 3 s e mede a falta do dardo contra o drone
# Saída: números no terminal e midia/rastreio_<nos>_<modo>.mp4 (o que a webcam vê, com a previsão em verde).
# Cerca de RAM (2026-11-10/11): o script roda num cgroup (systemd-run); passou do teto, o kernel mata só ele.
# O teto tem de ficar ABAIXO da memória livre (senão o WSL inteiro esgota antes de o teto agir):
# teto = livre − 600 MB de folga para o resto do sistema, no máximo 4 500 MB. Pico medido do teste: 1 069 MB (2026-11-11).
if [ -z "$CERCA_RAM" ]; then
    LIVRE=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
    TETO=$(( LIVRE - 600 < 4500 ? LIVRE - 600 : 4500 ))
    [ "$TETO" -lt 1500 ] && { echo "só $LIVRE MB livres (o teste precisa de 1500 + 600 de folga): feche algo e rode de novo"; exit 1; }
    echo "== cerca de RAM: teto $TETO MB ($LIVRE MB livres)"
    exec env CERCA_RAM=1 systemd-run --user --scope -q -p MemoryMax=${TETO}M -p MemorySwapMax=0 bash "$0" "$@"
fi
MODO=${1:-vaivem}; DUR=${2:-10}; NOS=${3:-mira}; DIST=${4:-3.0}
PROJ="$(cd "$(dirname "$0")" && pwd)"
MIDIA="$PROJ/../midia"; mkdir -p "$MIDIA"
W=$(mktemp -d); mkdir $W/src
ln -s "$PROJ/torreta_description" "$PROJ/torreta_nos" $W/src/
export ROS_DOMAIN_ID=77 GZ_PARTITION=rastreio_teste_$$
echo "== já rodando antes (não mexer):"; pgrep -a -x "gz|ruby|robot_state_pub" || echo "  nada"
echo "== o mundo do rastreio = torreta.sdf + sistema de sensores?"
diff <(grep -v -e "gz-sim-sensors-system" "$PROJ/torreta_description/worlds/torreta_rastreio.sdf" | sed '1,/-->/d') \
     <(sed '1,/-->/d' "$PROJ/torreta_description/worlds/torreta.sdf") > /dev/null && echo "  sim" || echo "  NÃO: os dois mundos divergiram"
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install > build.log 2>&1 || { tail -20 build.log; exit 1; }
source install/setup.bash
setsid ros2 launch torreta_description rastreio.launch.py gui:=false modo:=$MODO distancia:=$DIST > launch.log 2>&1 &
PID=$!
for i in $(seq 1 60); do sleep 4; N=$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c " active"); [ "$N" -ge 4 ] && break; done
timeout 15 ros2 control list_controllers | awk '{print "  "$1, $3}'
# a conferência sobe ANTES dos nós do Daniel: grava a partida também (2026-11-11: a 4 m a torreta subiu
# para o céu nos 3 s antes de a conferência ligar, sem nenhum dado)
case $NOS in mira) ARGS="15 8";; detectar) ARGS="--detectar";; seguir) ARGS="--detectar --seguir";; disparo) ARGS="--detectar --seguir --disparar $(( DUR - 3 )) --dardo $W/dardo.jsonl";; esac
( timeout 900 python3 "$PROJ/conferir_camera.py" $DUR "$MIDIA/rastreio_${NOS}_${MODO}_${DIST}m.mp4" $ARGS --salvar-quadro "$W/quadro60.png" ) &
CF=$!
sleep 2
LIB=install/torreta_nos/lib/torreta_nos; NO=; NO2=; NO3=; GD=
case $NOS in
  mira)     setsid $LIB/comando_torreta --ros-args -p use_sim_time:=true > no.log 2>&1 & NO=$!; ARGS="15 8" ;;
  detectar) setsid $LIB/detectar_alvo --ros-args -p use_sim_time:=true > no.log 2>&1 & NO=$!; ARGS="--detectar" ;;
  seguir)   setsid $LIB/detectar_alvo --ros-args -p use_sim_time:=true > no.log 2>&1 & NO=$!
            setsid $LIB/seguir_alvo --ros-args -p use_sim_time:=true > no2.log 2>&1 & NO2=$!; ARGS="--detectar --seguir" ;;
  disparo)  setsid $LIB/detectar_alvo --ros-args -p use_sim_time:=true > no.log 2>&1 & NO=$!
            setsid $LIB/seguir_alvo --ros-args -p use_sim_time:=true > no2.log 2>&1 & NO2=$!
            setsid $LIB/comando_torreta --ros-args -p use_sim_time:=true > no3.log 2>&1 & NO3=$!
            python3 "$PROJ/gravar_dardo.py" gravar $W/dardo.jsonl 1800 & GD=$!
            ARGS="--detectar --seguir --disparar $(( DUR - 3 )) --dardo $W/dardo.jsonl" ;;
esac
wait $CF
[ -f "$W/quadro60.png" ] && cp "$W/quadro60.png" "$MIDIA"/quadro_${NOS}_${MODO}_${DIST}m.png
echo "== alvo e ponte"; grep -E "mover_alvo|alvo:|parameter_bridge.*(webcam|Creating)" launch.log | sed 's/^\[[^]]*\] //' | head -6
echo "== nós do Daniel (log)"; for f in no.log no2.log no3.log; do [ -f $f ] && { echo "  $f:"; head -8 $f | cut -c1-200; }; done
kill $GD 2>/dev/null; for n in $NO $NO2 $NO3; do kill -INT -- -$n 2>/dev/null; done; kill -TERM -- -$PID; sleep 10; for n in $NO $NO2 $NO3 $PID; do kill -KILL -- -$n 2>/dev/null; done
pgrep -a -x "gz|ruby|robot_state_pub|parameter_bri" || echo "== nada sobrou"
CG=/sys/fs/cgroup$(cut -d: -f3 /proc/self/cgroup)
echo "== pico de memória do teste (cgroup): $(( $(cat $CG/memory.peak 2>/dev/null || echo 0) / 1048576 )) MB"
echo "workspace: $W"
