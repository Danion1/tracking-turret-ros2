#!/bin/bash
# Teste do disparo do comando_torreta (nó do Daniel) no Gazebo sem janela.
PROJ="$(cd "$(dirname "$0")" && pwd)"
W=$(mktemp -d); mkdir $W/src
ln -s "$PROJ/torreta_description" "$PROJ/torreta_nos" $W/src/
export ROS_DOMAIN_ID=77 GZ_PARTITION=teste_teste_$$
echo "== já rodando antes:"; pgrep -a -x "gz|ruby|robot_state_pub" || echo "  nada"
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install > build.log 2>&1 || { tail -20 build.log; exit 1; }
source install/setup.bash
setsid ros2 launch torreta_description gazebo.launch.py gui:=false > launch.log 2>&1 &
PID=$!
for i in $(seq 1 30); do sleep 4; N=$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c " active"); [ "$N" -ge 4 ] && break; done
timeout 15 ros2 control list_controllers
setsid ros2 run torreta_nos comando_torreta --ros-args -p use_sim_time:=true > no.log 2>&1 &
NO=$!
python3 "$PROJ/medir_mecanismos.py" gravar $W/juntas.jsonl 45 &
REC=$!
sleep 4
echo "== TESTE 1: disparo normal"
timeout 20 ros2 topic pub -w 1 --once /torreta/disparar std_msgs/msg/Empty "{}" > /dev/null
sleep 8
echo "== TESTE 1b: disparo duplo (o segundo deve ser ignorado)"
timeout 20 ros2 topic pub -w 1 --once /torreta/disparar std_msgs/msg/Empty "{}" > /dev/null
timeout 20 ros2 topic pub -w 1 --once /torreta/disparar std_msgs/msg/Empty "{}" > /dev/null
sleep 8
echo "== TESTE 2: timeout forçado (gatilho_controller desativado)"
timeout 20 ros2 control set_controller_state gatilho_controller inactive
timeout 20 ros2 topic pub -w 1 --once /torreta/disparar std_msgs/msg/Empty "{}" > /dev/null
sleep 8
timeout 20 ros2 control set_controller_state gatilho_controller active
wait $REC
echo "== log do nó"; cat no.log
python3 - $W/juntas.jsonl <<'PY'
import json,sys
L=[json.loads(l) for l in open(sys.argv[1])]
t0=L[0]["t"]
print(f"amostras {len(L)}, {L[-1]['t']-t0:.1f} s de sim")
prev=None
for d in L:
    vm=d["v"].get("volante_motor",0); s=d["p"].get("sg90",0); c=d["p"].get("cremalheira",0)
    st=(round(vm/100), round(s,1))
    if st!=prev: print(f"t={d['t']-t0:6.2f}  vol_motor={vm:8.1f}  vol_esp={d['v'].get('volante_espelho',0):8.1f}  sg90={s:5.3f}  crem={c*1000:5.2f} mm"); prev=st
print("crem máx mm:", max(d['p'].get('cremalheira',0) for d in L)*1000)
print("vol_motor máx:", max(d['v'].get('volante_motor',0) for d in L))
print("erro máx |esp + motor| vel:", max(abs(d['v'].get('volante_espelho',0)+d['v'].get('volante_motor',0)) for d in L))
PY
python3 "$PROJ/medir_mecanismos.py" analisar $W/juntas.jsonl 0 2>&1 | grep -v -i "tilt\|esforço"
kill -TERM -- -$NO -$PID; sleep 10; kill -KILL -- -$NO -$PID 2>/dev/null
pgrep -a -x "gz|ruby|robot_state_pub|parameter_bri" || echo "== nada sobrou"
echo "workspace: $W"
