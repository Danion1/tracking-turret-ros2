#!/bin/bash
# Teste do dardo solto no Gazebo sem janela, disparado pelo nó do Daniel (comando_torreta).
# Mede com gravar_dardo.py: onde a ponta para, se o dardo fica na calha, a saída e o voo.
# Uso:  bash ros2/testar_dardo.sh [pan_graus tilt_graus]     (padrão 0 0)
PROJ="$(cd "$(dirname "$0")" && pwd)"
PAN=${1:-0}; TILT=${2:-0}
W=$(mktemp -d); mkdir $W/src
ln -s "$PROJ/torreta_description" "$PROJ/torreta_nos" $W/src/
export ROS_DOMAIN_ID=77 GZ_PARTITION=teste_teste_$$
echo "== já rodando antes do teste (não mexer):"; pgrep -a -x "gz|ruby|robot_state_pub" || echo "  nada"
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install > build.log 2>&1 || { tail -20 build.log; exit 1; }
source install/setup.bash
CFG=install/torreta_description/share/torreta_description/config/dardo.json
setsid ros2 launch torreta_description gazebo.launch.py gui:=false > launch.log 2>&1 &
PID=$!
for i in $(seq 1 30); do sleep 4; N=$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c " active"); [ "$N" -ge 4 ] && break; done
timeout 15 ros2 control list_controllers
setsid ros2 run torreta_nos comando_torreta --ros-args -p use_sim_time:=true > no.log 2>&1 &
NO=$!
sleep 2
python3 "$PROJ/gravar_dardo.py" gravar $W/dardo.jsonl 20 &
REC=$!
sleep 2
if [ "$PAN" != "0" ] || [ "$TILT" != "0" ]; then
  echo "== mira: pan $PAN°, tilt $TILT°"
  timeout 20 ros2 topic pub -w 1 --once /torreta/alvo std_msgs/msg/Float64MultiArray "{data: [$PAN, $TILT]}" > /dev/null
  sleep 4
fi
echo "== disparo"
timeout 20 ros2 topic pub -w 1 --once /torreta/disparar std_msgs/msg/Empty "{}" > /dev/null
wait $REC
echo "== nó do Daniel"; cat no.log | grep -v Traceback | head -12
echo "== simulador dos volantes"; grep -E "simular_volantes|LANÇADO|não lança" launch.log | grep -v "^\s*$" | head -6
echo "== medida"; python3 "$PROJ/gravar_dardo.py" analisar $W/dardo.jsonl $CFG; RES=$?
echo "== erros no log"; grep -i -E "error|fail" launch.log | grep -v -E "not yet initialized|slower than" | head -10
kill -INT -- -$NO 2>/dev/null; kill -TERM -- -$PID; sleep 10; kill -KILL -- -$NO -$PID 2>/dev/null
pgrep -a -x "gz|ruby|robot_state_pub|parameter_bri" || echo "== nada sobrou"
echo "workspace: $W"
exit $RES
