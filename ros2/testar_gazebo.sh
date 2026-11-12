#!/bin/bash
# Conferência da torreta no Gazebo sem janela (workspace temporário, rede própria).
# Os movimentos são `ros2 topic pub` de linha de comando: o nó de comando é do Daniel.
# Mede com medir_mecanismos.py: razão e sinal de cada transmissão amostra a amostra,
# salto, e o esforço do tilt contra a gravidade das massas do URDF.
# Uso:  bash ros2/testar_gazebo.sh      (da pasta do projeto ou de qualquer lugar)

PROJ="$(cd "$(dirname "$0")" && pwd)"
W=$(mktemp -d); mkdir $W/src
ln -s "$PROJ/torreta_description" $W/src/
export ROS_DOMAIN_ID=77 GZ_PARTITION=teste_teste_$$      # não cruza com a sessão do Daniel (ros2-wsl.md, 8)
echo "== já rodando antes do teste (não mexer):"; pgrep -a -x "gz|ruby|robot_state_pub" || echo "  nada"
cd $W && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select torreta_description > build.log 2>&1 || { tail build.log; exit 1; }
source install/setup.bash
URDF=install/torreta_description/share/torreta_description/urdf/torreta.urdf

# torque de gravidade no tilt, pose zero: g·Σ m·x do que gira no tilt (frame do berço)
TAU=$(python3 - "$URDF" <<'EOF'
import sys, xml.etree.ElementTree as ET
r = ET.parse(sys.argv[1]).getroot()
orig = {j.find("child").get("link"): float(j.find("origin").get("xyz").split()[0]) for j in r.findall("joint")}
sub = {"berco": 0.0, "rotor_mais_y": orig["rotor_mais_y"], "rotor_menos_y": orig["rotor_menos_y"],
       "pinhao_sg90": orig["pinhao_sg90"], "empurrador": orig["empurrador"]}
t = 0.0
for l in r.findall("link"):
    if l.get("name") in sub:
        i = l.find("inertial")
        t += float(i.find("mass").get("value")) * (sub[l.get("name")] + float(i.find("origin").get("xyz").split()[0]))
print(f"{9.80665 * t:.5f}")
EOF
)
echo "== torque de gravidade calculado no tilt (pose zero): $TAU N·m"

setsid ros2 launch torreta_description gazebo.launch.py gui:=false > launch.log 2>&1 &
PID=$!
for i in $(seq 1 30); do
  sleep 4
  N=$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c " active")
  [ "$N" -ge 4 ] && break
done
echo "== controladores"; timeout 15 ros2 control list_controllers

python3 "$PROJ/medir_mecanismos.py" gravar $W/juntas.jsonl 75 &
REC=$!
sleep 6                                                     # parado na pose zero (esforço do tilt)
echo "== pan_motor -1,0 rad (torre +28,6°) e tilt 0,5 rad em 2 s"
timeout 20 ros2 topic pub -w 1 --once /pan_tilt_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [pan_motor, tilt], points: [{positions: [-1.0, 0.5], time_from_start: {sec: 2}}]}" > /dev/null
sleep 12
echo "== gatilho: sg90 1,2 rad em 1 s (cremalheira 12 mm)"
timeout 20 ros2 topic pub -w 1 --once /gatilho_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [sg90], points: [{positions: [1.2], time_from_start: {sec: 1}}]}" > /dev/null
sleep 8
echo "== volantes: 1200 rad/s (o FA-130RA-18100 faz 1 288 sem carga)"
timeout 20 ros2 topic pub -w 1 --once /volantes_controller/commands std_msgs/msg/Float64MultiArray "{data: [1200.0]}" > /dev/null
sleep 10
echo "== pan_motor +2,0 rad (torre −57,3°), gatilho de volta"
timeout 20 ros2 topic pub -w 1 --once /pan_tilt_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [pan_motor, tilt], points: [{positions: [2.0, -0.2], time_from_start: {sec: 2}}]}" > /dev/null
timeout 20 ros2 topic pub -w 1 --once /gatilho_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [sg90], points: [{positions: [0.0], time_from_start: {sec: 1}}]}" > /dev/null
wait $REC
echo "== medida"
python3 "$PROJ/medir_mecanismos.py" analisar $W/juntas.jsonl $TAU; RES=$?
echo "== gravação: $W/juntas.jsonl (fora do Git)"
echo "== erros/avisos no log"; grep -i -E "error|warn|fail|mimic" launch.log | grep -v -E "not yet initialized|Executor is not available|slower than the gazebo" | head -20
kill -TERM -- -$PID; sleep 10; kill -KILL -- -$PID 2>/dev/null
pgrep -a -x "gz|ruby|robot_state_pub|parameter_bri" || echo "== nada sobrou"
echo "workspace: $W"
exit $RES
