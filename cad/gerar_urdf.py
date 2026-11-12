#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gerar_urdf.py — a torreta em URDF, com CADA mecanismo como link e junta próprios.

Escreve ros2/torreta_description/urdf/torreta.urdf e as malhas (meshes/) a partir
das peças aprovadas (montagem.montar()), sem número digitado à mão aqui:
  origens e eixos ........ parametros.json (os mesmos que o CAD usa)
  massa e inércia ......... volume do CAD × PLA maciço (impressas, como na peça 7);
                            massa do datasheet espalhada no sólido de referência (compradas)
  limites ................. parametros.json (pan ±90°, tilt −20°..40°, curso do empurrador)
  effort e velocity ....... datasheets (STS3215 escalado a 7,0 V, SG90, FA-130RA-18100)

Juntas (decisão do Daniel, 2026-11-09: o comando vai no MOTOR, como no hardware):
  pan_motor        revoluta, eixo do servo do pan = pinhão 20T          (comandada)
  pan              revoluta, coroa 40T + mesa      mimic −z1/z2 = −0,5 do pan_motor
  tilt             revoluta, eixo −y (+ levanta o nariz), direto no servo (comandada)
  volante_motor    contínua, motor 130 +y + volante +y                    (comandada)
  volante_espelho  contínua, motor 130 −y + volante −y   mimic −1 do volante_motor
  sg90             revoluta, pinhão m1 20T do SG90                        (comandada)
  cremalheira      prismática +x, empurrador          mimic rp = 0,010 m/rad do sg90
  (o dardo da câmara NÃO é link: vira o modelo solto urdf/dardo.sdf, empurrado pela cremalheira
   por contato e lançado pelo ros2/../scripts/simular_volantes.py; 2026-11-10)

Conferências (param antes de gravar):
  1. URDF × CAD: cada peça levada pelo URDF (cinemática direta daqui, com mimic e
     prismática) cai nos mesmos vértices que montagem.posar() em 18 poses;
  2. engrenamento: no ponto primitivo, o pinhão e a coroa (e o pinhão do SG90 e a
     cremalheira) andam igual: razão E sinal da mimic conferidos pela geometria;
     os dois volantes empurram o dardo para +x no ponto de contato;
  3. malhas recentradas: a caixa de cada STL lido de volta = a do sólido no frame do link;
  4. massas: soma dos links = soma das peças com massa; tensores físicos;
  5. check_urdf (ROS 2) aceita o arquivo.
Depois: previews/urdf_*.png, montados pelo URDF e pelos STL do pacote.

Uso (da pasta do projeto):  ~/.venvs/cadquery/bin/python cad/gerar_urdf.py
"""

import contextlib
import io
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import usar_biblioteca  # noqa: E402,F401
import cadquery as cq  # noqa: E402
import montagem as mo  # noqa: E402
from check_bbox import exigir_tudo_ok  # noqa: E402
from dados_materiais import PLA  # noqa: E402
from exportar_stl_mm import exportar_stl_mm  # noqa: E402
import massa_inercia as mi  # noqa: E402

PROJ = os.path.dirname(AQUI)
PKG = os.path.join(PROJ, "ros2", "torreta_description")
P = json.load(open(os.path.join(PROJ, "plano", "parametros.json"), encoding="utf-8"))
K = P["pecas"]
ZT = P["tilt_cad"]["z"]
VO, EP = P["berco"]["volante"], P["berco"]["empurrador"]
EX, EY = P["pan"]["pinhao"]["eixo"]
XS, YS = EP["x_pinhao"], EP["y_primitiva"] + EP["rp"]
RAZAO = P["pan"]["razao"]                                        # 40/20
G_ = 9.80665
ok = True


def conferir(nome, cond, detalhe=""):
    global ok
    print(f"  [{'OK' if cond else 'FALHOU'}] {nome}{(': ' + detalhe) if detalhe else ''}")
    ok &= bool(cond)
    return cond


# ------------------------------------------------------------------ links e juntas
# link: (frame de onde as peças vêm, origem do link nesse frame, mm)
LINKS = {
    "base_link": ("mundo", (0.0, 0.0, 0.0)),
    "pinhao_pan": ("mundo", (EX, EY, 0.0)),
    "pan_link": ("mundo", (0.0, 0.0, 0.0)),
    "berco": ("berco", (0.0, 0.0, 0.0)),
    "rotor_mais_y": ("berco", (VO["x"], VO["y"], 0.0)),
    "rotor_menos_y": ("berco", (VO["x"], -VO["y"], 0.0)),
    "pinhao_sg90": ("berco", (XS, YS, 0.0)),
    "empurrador": ("berco", (0.0, 0.0, 0.0)),               # cremalheira (o nome "cremalheira" é da junta: no SDF link e junta não podem ter o mesmo nome)
}


def link_da_peca(p):
    if p.grupo == "base":
        return "base_link"
    if p.grupo == "pinhao":
        return "pinhao_pan"
    if p.grupo == "pan":
        return "pan_link"
    if p.grupo == "emp":
        if p.nome == "dardo na câmara":
            return "dardo"                                   # corpo solto (urdf/dardo.sdf), fora do URDF
        return "pinhao_sg90" if p.nome == "pinhão do SG90" else "empurrador"
    if p.nome == "volante +y":
        return "rotor_mais_y"
    if p.nome == "volante −y":
        return "rotor_menos_y"
    return "berco"


st, sg, mt = K["sts3215"], K["sg90"], K["motor130"]
TAU_STS = st["torque_kgcm"] * 7.0 / st["tensao"] * G_ / 100        # stall a 7,0 V (escala linear, ASSUMIDO como na peça 7)
VEL_STS = math.radians(60) / (st["s60"] * st["tensao"] / 7.0)        # sem carga, a 7,0 V
TAU_SG, VEL_SG = sg["torque_kgcm"] * G_ / 100, math.radians(60) / sg["s60"]
M130 = json.load(open(os.path.join(PROJ, "fontes", "componentes.json"), encoding="utf-8"))["motor_mabuchi_fa130ra"]["variante_18100"]["valor"]
TAU_130, VEL_130 = M130["torque_stall_mNm"] / 1000, M130["rpm_sem_carga"] * 2 * math.pi / 60
GIRO = EP["curso"] / EP["rp"]                                      # rad do pinhão do SG90 no curso inteiro
LT = [math.radians(v) for v in P["limites"]["tilt"]]
LP = [math.radians(v) for v in P["limites"]["pan"]]

# Folga nos limites das juntas MOVIDAS (mimic): o batente real é o do motor (que tem os limites exatos);
# no Gazebo, a mimic do gz_ros2_control é um comando de velocidade, e o DART "gruda" a junta movida
# que encosta no próprio limite (medido em 2026-11-09: a cremalheira parou em 0 com o tilt andando e
# ficou em 13,2 mm com o SG90 em 1,2 rad). Com a folga, ela segue o motor (12,00 mm).
FOLGA_PAN = math.radians(2.0)
FOLGA_CREM = 0.001                                                 # m

JUNTAS = [
    dict(nome="pan_motor", tipo="revolute", pai="base_link", filho="pinhao_pan", xyz=(EX, EY, 0), eixo=(0, 0, 1),
         lim=(-RAZAO * LP[1], -RAZAO * LP[0]), effort=TAU_STS, vel=VEL_STS),
    dict(nome="pan", tipo="revolute", pai="base_link", filho="pan_link", xyz=(0, 0, 0), eixo=(0, 0, 1),
         lim=(LP[0] - FOLGA_PAN, LP[1] + FOLGA_PAN), effort=RAZAO * TAU_STS, vel=VEL_STS / RAZAO, mimic=("pan_motor", -1.0 / RAZAO)),
    dict(nome="tilt", tipo="revolute", pai="pan_link", filho="berco", xyz=(0, 0, ZT), eixo=(0, -1, 0),
         lim=tuple(LT), effort=TAU_STS, vel=VEL_STS),
    dict(nome="volante_motor", tipo="continuous", pai="berco", filho="rotor_mais_y", xyz=(VO["x"], VO["y"], 0), eixo=(0, 0, 1),
         effort=TAU_130, vel=VEL_130),
    dict(nome="volante_espelho", tipo="continuous", pai="berco", filho="rotor_menos_y", xyz=(VO["x"], -VO["y"], 0), eixo=(0, 0, 1),
         effort=TAU_130, vel=VEL_130, mimic=("volante_motor", -1.0)),
    dict(nome="sg90", tipo="revolute", pai="berco", filho="pinhao_sg90", xyz=(XS, YS, 0), eixo=(0, 0, 1),
         lim=(0.0, GIRO), effort=TAU_SG, vel=VEL_SG),
    dict(nome="cremalheira", tipo="prismatic", pai="berco", filho="empurrador", xyz=(0, 0, 0), eixo=(1, 0, 0),
         lim=(-FOLGA_CREM, EP["curso"] / 1000 + FOLGA_CREM), effort=TAU_SG / (EP["rp"] / 1000), vel=VEL_SG * EP["rp"] / 1000,
         mimic=("sg90", EP["rp"] / 1000)),
]
# prismática: xyz em mm aqui, convertido na escrita; o "lim" dela já está em m


# ------------------------------------------------------------------ cinemática direta (mm)
def rot(eixo, a):
    ux, uy, uz = eixo
    c, s, v = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + ux * ux * v, ux * uy * v - uz * s, ux * uz * v + uy * s, 0],
                     [uy * ux * v + uz * s, c + uy * uy * v, uy * uz * v - ux * s, 0],
                     [uz * ux * v - uy * s, uz * uy * v + ux * s, c + uz * uz * v, 0], [0, 0, 0, 1.0]])


def trans(x, y, z):
    t = np.eye(4)
    t[:3, 3] = (x, y, z)
    return t


def valores(q):
    """q: só as juntas comandadas (rad / m) -> todas, com as mimic calculadas."""
    out = dict(q)
    for j in JUNTAS:
        if "mimic" in j:
            lider, mult = j["mimic"]
            out[j["nome"]] = mult * q.get(lider, 0.0)
        out.setdefault(j["nome"], 0.0)
    return out


def fk(q, metros=False):
    """{link: 4×4 no mundo} (mm, ou m com metros=True), lendo a tabela JUNTAS."""
    v = valores(q)
    T = {"base_link": np.eye(4)}
    for j in JUNTAS:                                 # a tabela está em ordem de pai antes de filho
        o = trans(*j["xyz"])
        if j["tipo"] == "prismatic":
            m = trans(*(1000 * v[j["nome"]] * e for e in j["eixo"]))
        else:
            m = rot(j["eixo"], v[j["nome"]])
        T[j["filho"]] = T[j["pai"]] @ o @ m
    if metros:
        for k in T:
            T[k] = T[k].copy()
            T[k][:3, 3] /= 1000
    return T


def q_do_cad(pan_deg, tilt_deg, curso_mm):
    return {"pan_motor": -RAZAO * math.radians(pan_deg), "tilt": math.radians(tilt_deg), "sg90": curso_mm / EP["rp"]}


def pts(s):
    return np.array([v.toTuple() for v in s.Vertices()])


def aplicar(T, P_):
    return (T[:3, :3] @ P_.T).T + T[:3, 3]


# ------------------------------------------------------------------ escrita
def f(x):
    return f"{x:.6g}"


def escrever_urdf(visuais, inerciais, z_base, colisoes):
    r = ['<?xml version="1.0"?>',
         '<!-- GERADO por cad/gerar_urdf.py a partir do CAD da torreta. Não editar à mão: mudar o gerador. -->',
         '<robot name="torreta">']
    for nome, cor in sorted({(v["material"], v["cor"]) for vs in visuais.values() for v in vs}):
        r.append(f'  <material name="{nome}"><color rgba="{f(cor[0])} {f(cor[1])} {f(cor[2])} 1"/></material>')
    r += ['', '  <!-- world vazio: o robot_state_publisher não aceita inércia no link raiz; no Gazebo, prende a base.',
          '       A base sobe o fundo da caixa até o chão (a origem do CAD é o topo do tampo). -->',
          '  <link name="world"/>',
          '  <joint name="world_base" type="fixed"><parent link="world"/><child link="base_link"/>'
          f'<origin xyz="0 0 {f(z_base)}" rpy="0 0 0"/></joint>']
    for link in LINKS:
        r.append(f'\n  <link name="{link}">')
        for v in visuais[link]:
            r += ['    <visual>', '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                  f'      <geometry><mesh filename="package://torreta_description/meshes/{v["arquivo"]}" scale="0.001 0.001 0.001"/></geometry>',
                  f'      <material name="{v["material"]}"/>', '    </visual>']
        r += colisoes.get(link, [])
        m, c, t = inerciais[link]
        r += ['    <inertial>', f'      <origin xyz="{f(c[0])} {f(c[1])} {f(c[2])}" rpy="0 0 0"/>', f'      <mass value="{f(m)}"/>',
              f'      <inertia ixx="{f(t[0][0])}" ixy="{f(t[0][1])}" ixz="{f(t[0][2])}" iyy="{f(t[1][1])}" iyz="{f(t[1][2])}" izz="{f(t[2][2])}"/>',
              '    </inertial>', '  </link>']
    for j in JUNTAS:
        x, y, z = (v / 1000 for v in j["xyz"])
        r += [f'\n  <joint name="{j["nome"]}" type="{j["tipo"]}">', f'    <parent link="{j["pai"]}"/>', f'    <child link="{j["filho"]}"/>',
              f'    <origin xyz="{f(x)} {f(y)} {f(z)}" rpy="0 0 0"/>', f'    <axis xyz="{j["eixo"][0]} {j["eixo"][1]} {j["eixo"][2]}"/>']
        if j["tipo"] == "continuous":
            r.append(f'    <limit effort="{f(j["effort"])}" velocity="{f(j["vel"])}"/>')
        else:
            r.append(f'    <limit lower="{f(j["lim"][0])}" upper="{f(j["lim"][1])}" effort="{f(j["effort"])}" velocity="{f(j["vel"])}"/>')
        if "mimic" in j:
            r.append(f'    <mimic joint="{j["mimic"][0]}" multiplier="{f(j["mimic"][1])}" offset="0"/>')
        r.append('  </joint>')
    r.append('</robot>\n')
    return "\n".join(r)



# ------------------------------------------------------------------ versão só do vídeo
ESCONDER_NO_VIDEO = {"gaiola": (0.45, 0.65, 0.85), "funil": (0.80, 0.80, 0.55), "dardos do funil": (0.15, 0.55, 0.95)}   # cores no berço


def escrever_urdf_video(texto):
    """torreta_video.urdf: o mesmo URDF (física idêntica), com duas mudanças SÓ VISUAIS para o close do lançador:
    sem a gaiola, o funil e os dardos do funil (tampam a cremalheira, o dardo e os volantes por cima) e com uma
    marca branca no topo de cada volante (um disco liso girando não parece girar)."""
    raiz = ET.fromstring(texto.split("\n", 1)[1])
    berco = next(l for l in raiz.findall("link") if l.get("name") == "berco")
    tirar = {f"cor_{int(c[0]*100)}_{int(c[1]*100)}_{int(c[2]*100)}" for c in ESCONDER_NO_VIDEO.values()}
    vis = [v for v in berco.findall("visual") if v.find("material").get("name") in tirar]
    conferir("vídeo: cada cor escondida é um visual só do berço", len(vis) == len(tirar), f"{len(vis)} visuais tirados")
    for v in vis:
        berco.remove(v)
    ET.SubElement(raiz, "material", name="marca_branca").append(ET.Element("color", rgba="1 1 1 1"))
    for nome in ("rotor_mais_y", "rotor_menos_y"):
        link = next(l for l in raiz.findall("link") if l.get("name") == nome)
        v = ET.Element("visual")
        ET.SubElement(v, "origin", xyz=f"{f(VO['r'] * 0.55 / 1000)} 0 {f((VO['l'] / 2 + 0.3) / 1000)}", rpy="0 0 0")
        g = ET.SubElement(v, "geometry")
        ET.SubElement(g, "box", size=f"{f(VO['r'] * 0.8 / 1000)} 0.003 0.0006")
        ET.SubElement(v, "material", name="marca_branca")
        link.insert(0, v)
    corpo = ET.tostring(raiz, encoding="unicode")
    cab = ('<?xml version="1.0"?>\n<!-- GERADO por cad/gerar_urdf.py: SÓ PARA O VÍDEO. Física igual à do torreta.urdf; '
           'sem os visuais da gaiola, do funil e dos dardos do funil, e com uma marca branca em cada volante. -->\n')
    with open(os.path.join(PKG, "urdf", "torreta_video.urdf"), "w", encoding="utf-8") as fh:
        fh.write(cab + corpo + "\n")


# ------------------------------------------------------------------ dardo solto (Gazebo)
R_COL_DARDO = 6.4          # mm: 0,1 abaixo do raio, para não ficar espremido entre o piso e o teto da gaiola (abertura de 13,0)
MU_DARDO = 0.5             # atrito espuma × PLA: ASSUMIDO (sem fonte; conferir no protótipo)


def escrever_dardo(p, z_base):
    """O dardo da câmara vira um modelo SDF solto: malha e inércia do CAD, colisão cilíndrica, pose do
    CAD na pose zero; config/dardo.json leva o que o simular_volantes.py precisa (tudo em m)."""
    import vtk
    D = K["dardo"]
    bb = p.s.BoundingBox()
    c = (bb.center.x, bb.center.y, bb.center.z)                    # mm, frame do berço
    s0 = p.s.translate(cq.Vector(-c[0], -c[1], -c[2]))
    with contextlib.redirect_stdout(io.StringIO()):
        exportar_stl_mm(s0, os.path.join(PKG, "meshes"), "dardo_0.stl")
    rd = vtk.vtkSTLReader(); rd.SetFileName(os.path.join(PKG, "meshes", "dardo_0.stl")); rd.Update()
    b = rd.GetOutput().GetBounds()
    conferir("malha do dardo centrada (caixa do STL)", abs(b[0] + D["L"] / 2) < 0.1 and abs(b[1] - D["L"] / 2) < 0.1, f"x [{b[0]:.2f}, {b[1]:.2f}] mm")
    m, com, t = mi.combinar([mi.propriedades_solido(s0, D["massa_g"] / (s0.Volume() / 1000))])
    mi.checar_tensor(t, "dardo")
    conferir("massa do dardo = medição publicada", abs(m * 1000 - D["massa_g"]) < 1e-9, f"{m * 1000:.2f} g; CM ({com[0]*1000:.2f}, {com[1]*1000:.2f}, {com[2]*1000:.2f}) mm")
    # pose de nascimento: berço na pose zero, centro do dardo do CAD, descido até a colisão encostar no piso (+0,05 de folga)
    T = trans(0, 0, z_base * 1000) @ fk({})["berco"]
    conferir("berço sem rotação na pose zero (o dardo nasce com rpy = 0)", np.allclose(T[:3, :3], np.eye(3)))
    zc = c[2] - (D["d"] / 2 - R_COL_DARDO) + 0.05
    w = aplicar(T, np.array([[c[0], c[1], zc]]))[0] / 1000
    ponta0 = (c[0] + D["L"] / 2)
    toque = VO["x"] - math.sqrt(VO["r"] ** 2 - (VO["y"] - D["d"] / 2) ** 2)    # a mesma conta do gerar_parametros.py
    sdf = f"""<?xml version="1.0"?>
<!-- GERADO por cad/gerar_urdf.py: o dardo da câmara como corpo solto. Não editar à mão.
     Malha e inércia do CAD (massa {D['massa_g']} g, {D['fonte']}); colisão: cilindro Ø{2 * R_COL_DARDO} (0,1 mm abaixo do raio);
     atrito {MU_DARDO} ASSUMIDO. O PosePublisher manda a pose do modelo (mundo) a 1 kHz em /model/dardo/pose. -->
<sdf version="1.9">
  <model name="dardo">
    <link name="corpo">
      <inertial>
        <pose>{f(com[0])} {f(com[1])} {f(com[2])} 0 0 0</pose>
        <mass>{f(m)}</mass>
        <inertia><ixx>{f(t[0][0])}</ixx><ixy>{f(t[0][1])}</ixy><ixz>{f(t[0][2])}</ixz><iyy>{f(t[1][1])}</iyy><iyz>{f(t[1][2])}</iyz><izz>{f(t[2][2])}</izz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><mesh><uri>model://torreta_description/meshes/dardo_0.stl</uri><scale>0.001 0.001 0.001</scale></mesh></geometry>
        <material><ambient>0.15 0.55 0.95 1</ambient><diffuse>0.15 0.55 0.95 1</diffuse></material>
      </visual>
      <collision name="colisao">
        <pose>0 0 0 0 1.5707963267948966 0</pose>
        <geometry><cylinder><radius>{f(R_COL_DARDO / 1000)}</radius><length>{f(D['L'] / 1000)}</length></cylinder></geometry>
        <surface><friction><ode><mu>{MU_DARDO}</mu><mu2>{MU_DARDO}</mu2></ode></friction></surface>
      </collision>
    </link>
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_model_pose>true</publish_model_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <update_frequency>1000</update_frequency>
    </plugin>
  </model>
</sdf>
"""
    with open(os.path.join(PKG, "urdf", "dardo.sdf"), "w", encoding="utf-8") as fh:
        fh.write(sdf)
    ET.fromstring(sdf.split("\n", 1)[1])
    cfg = {"_fonte": "GERADO por cad/gerar_urdf.py; lido pelo launch (nascimento) e pelo scripts/simular_volantes.py",
           "massa_kg": m, "comprimento_m": D["L"] / 1000, "raio_volante_m": VO["r"] / 1000,
           "x_toque_m": toque / 1000, "x_ponta_inicial_m": ponta0 / 1000,
           "nascimento_xyz_m": [float(v) for v in w], "link": "dardo::corpo", "mundo": "torreta"}
    with open(os.path.join(PKG, "config", "dardo.json"), "w", encoding="utf-8") as fh:
        json.dump(cfg, fh, indent=2, ensure_ascii=False)
    print(f"  ponta do dardo em x = {ponta0:.2f} mm (berço); toque nos volantes em x = {toque:.2f} mm; "
          f"curso útil da cremalheira {EP['curso'] - 1.0:.2f} mm → ponta chega a {ponta0 + EP['curso'] - 1.0:.2f} mm "
          f"({(ponta0 + EP['curso'] - 1.0) - toque:+.2f} mm além do toque)")
    print(f"  nasce em ({w[0]:.4f}, {w[1]:.4f}, {w[2]:.4f}) m; v da borda a 1 200 rad/s = {1200 * VO['r'] / 1000:.1f} m/s")

# ------------------------------------------------------------------ principal
def main():
    import fusion_export as fe                       # os tubos dos fios, nos percursos já conferidos

    print("0. Kernel de inércia")
    with contextlib.redirect_stdout(io.StringIO()):
        mi.conferir_kernel()
    print("  conferir_kernel OK")

    print("\n1. Peças por link (frame do link, mm)")
    pecas = mo.montar()
    por_link = {k: [] for k in LINKS}                # (peça ou None, sólido no frame do link, cor)
    dardo_cad = None                                 # o dardo da câmara, no frame do berço (vira modelo solto)
    for p in pecas:
        L = link_da_peca(p)
        if L == "dardo":
            dardo_cad = p
            continue
        o = LINKS[L][1]
        por_link[L].append((p, p.s.translate(cq.Vector(-o[0], -o[1], -o[2])), p.cor))
    fios = fe.fios_por_grupo()
    por_link["base_link"].append((None, fios["fios_base"], (0.4, 0.4, 0.45)))
    por_link["pan_link"].append((None, fios["fios_pan"], (0.4, 0.4, 0.45)))
    por_link["berco"].append((None, fios["fios_tilt"].translate(cq.Vector(0, 0, -ZT)), (0.4, 0.4, 0.45)))
    for L, itens in por_link.items():
        print(f"    {L:<14} {len(itens):3d} corpos")

    print("\n2. URDF × CAD: os vértices de cada peça em 18 poses (pan × tilt × curso)")
    pior, onde = 0.0, ""
    for pa in (-90, 35, 90):
        for ti in (P["limites"]["tilt"][0], 15, P["limites"]["tilt"][1]):
            for cu in (0.0, EP["curso"]):
                T = fk(q_do_cad(pa, ti, cu))
                for L, itens in por_link.items():
                    for p, s, _ in itens:
                        if p is None:
                            continue
                        a = aplicar(T[L], pts(s)[:8])
                        b = pts(mo.posar(p, pa, ti, cu))[:8]
                        d = float(np.max(np.linalg.norm(a - b, axis=1)))
                        if d > pior:
                            pior, onde = d, f"{p.nome} (pan {pa}°, tilt {ti}°, curso {cu})"
    conferir("todas as peças no mesmo lugar do CAD aprovado", pior < 1e-6, f"maior desvio {pior:.2e} mm ({onde})")
    print("  => a conferência de interferência da peça 7 (60 poses, zero) vale para o URDF")

    print("\n3. Engrenamento pela cinemática do URDF (razão E sinal da mimic)")
    eps = 1e-4                                       # passo da diferença finita: erro de 2ª ordem ~eps/2 relativo, por isso a tolerância é 1e-3
    # pan: ponto primitivo na linha dos centros, a rp do pinhão (= 30 da coroa)
    c = np.array([EX, EY, 13.0]); u = c / np.linalg.norm(c[:2]) * [1, 1, 0]
    pp = np.array([0, 0, 13.0]) + u * (np.linalg.norm([EX, EY]) - P["pan"]["pinhao"]["rp"])
    for q0 in (0.0, 1.0, -2.5):
        T0, T1 = fk({"pan_motor": q0}), fk({"pan_motor": q0 + eps})
        vp = [aplicar(T1[L] @ np.linalg.inv(T0[L]), pp[None])[0] - pp for L in ("pinhao_pan", "pan_link")]
        conferir(f"pan_motor {q0:+.1f} rad: pinhão e coroa andam igual no ponto primitivo",
                 np.linalg.norm(vp[0] - vp[1]) < 1e-3 * np.linalg.norm(vp[0]),
                 f"{np.linalg.norm(vp[0]) / eps:.3f} mm/rad nos dois; coroa gira {valores({'pan_motor': 1.0})['pan']:+.2f} × motor")
    print(f"  raio do ponto primitivo: pinhão {np.linalg.norm(pp[:2] - c[:2]):.3f} mm, coroa {np.linalg.norm(pp[:2]):.3f} mm")
    # SG90: ponto de baixo do pinhão (y = y_primitiva) contra a cremalheira
    pc = np.array([XS, EP["y_primitiva"], 0.0, 1.0])
    for th in (0.0, GIRO / 2, GIRO - eps):
        T0, T1 = fk({"sg90": th}), fk({"sg90": th + eps})
        w0 = np.linalg.inv(T0["berco"])
        a = (w0 @ T1["pinhao_sg90"] @ np.linalg.inv(T0["pinhao_sg90"]) @ T0["berco"] @ pc)[:3] - pc[:3]
        b = (w0 @ T1["empurrador"] @ np.linalg.inv(T0["empurrador"]) @ T0["berco"] @ pc)[:3] - pc[:3]
        conferir(f"sg90 {th:.2f} rad: pinhão e cremalheira andam igual no ponto primitivo", np.linalg.norm(a - b) < 1e-3 * np.linalg.norm(a),
                 f"{a[0] / eps:.3f} mm/rad em +x (= rp {EP['rp']})")
    # volantes: no ponto de contato com o dardo, a borda anda +x nos dois
    for L, sy in (("rotor_mais_y", 1), ("rotor_menos_y", -1)):
        pc = np.array([VO["x"], sy * (VO["y"] - VO["r"]), 0.0, 1.0])
        T0, T1 = fk({"volante_motor": 0.0}), fk({"volante_motor": eps})
        w0 = np.linalg.inv(T0["berco"])
        d = (w0 @ T1[L] @ np.linalg.inv(T0[L]) @ T0["berco"] @ pc)[:3] - pc[:3]
        conferir(f"{L}: borda no contato anda +x com volante_motor > 0", d[0] > 0 and abs(d[1]) < 1e-3 * d[0],
                 f"{d[0] / eps:.2f} mm/rad (r = {VO['r']}) → dardo a {d[0] / eps * VEL_130 / 1000:.1f} m/s sem carga (limite de cima)")

    print("\n4. Malhas e massas")
    os.makedirs(os.path.join(PKG, "meshes"), exist_ok=True)
    for arq in os.listdir(os.path.join(PKG, "meshes")):
        if arq.endswith(".stl"):
            os.remove(os.path.join(PKG, "meshes", arq))
    import vtk
    visuais, inerciais, m_total, m_pecas = {}, {}, 0.0, 0.0
    for L, itens in por_link.items():
        cores = {}
        for p, s, cor in itens:
            cores.setdefault(tuple(cor), []).append(s)
        visuais[L] = []
        for i, (cor, sol) in enumerate(sorted(cores.items())):
            comp = cq.Compound.makeCompound(sol)
            nome = f"{L}_{i}"
            with contextlib.redirect_stdout(io.StringIO()):
                exportar_stl_mm(comp, os.path.join(PKG, "meshes"), nome + ".stl")
            rd = vtk.vtkSTLReader(); rd.SetFileName(os.path.join(PKG, "meshes", nome + ".stl")); rd.Update()
            b = rd.GetOutput().GetBounds()
            bb = comp.BoundingBox()
            d = max(abs(b[0] - bb.xmin), abs(b[1] - bb.xmax), abs(b[2] - bb.ymin), abs(b[3] - bb.ymax), abs(b[4] - bb.zmin), abs(b[5] - bb.zmax))
            if d > 0.1:
                conferir(f"malha {nome} no frame do link", False, f"caixa difere {d:.2f} mm")
            visuais[L].append({"arquivo": nome + ".stl", "material": f"cor_{int(cor[0]*100)}_{int(cor[1]*100)}_{int(cor[2]*100)}", "cor": cor})
        corpos = []
        for p, s, _ in itens:
            if p is None or not p.conta or not p.massa_g:
                continue
            dens = PLA["densidade_g_cm3"] if p.impressa else p.massa_g / (s.Volume() / 1000)
            corpos.append(mi.propriedades_solido(s, dens))
            m_pecas += p.massa_g / 1000
        if not corpos:
            raise SystemExit(f"link {L} sem massa")
        m, com, t = mi.combinar(corpos)
        mi.checar_tensor(t, L)
        inerciais[L] = (m, com, t)
        m_total += m
        print(f"    {L:<14} {m * 1000:7.1f} g  CM ({com[0]*1000:6.1f}, {com[1]*1000:6.1f}, {com[2]*1000:6.1f}) mm  {len(visuais[L])} malhas")
    conferir("malhas recentradas no próprio link (caixa do STL = caixa do sólido, ±0,1 mm)", True,
             f"{sum(len(v) for v in visuais.values())} STL")
    conferir("soma das massas dos links = soma das peças", abs(m_total - m_pecas) < 1e-6, f"{m_total * 1000:.1f} g (impressas em PLA maciço, limite de cima)")

    print("\n5. Colisão do caminho do dardo (caixas primitivas, frame do berço) e altura da base")
    # Só o que o dardo toca. Os volantes NÃO colidem: a espuma passa comprimida 3,35 mm entre eles,
    # e um corpo rígido ali explodiria; quem faz o papel do atrito volante-espuma é o
    # scripts/simular_volantes.py (simplificação declarada, ficha seção 7).
    z_base = -min(s.BoundingBox().zmin for _, s, _ in por_link["base_link"]) / 1000
    colisoes = {}

    def caixa_col(x0, x1, y0, y1, z0, z1):
        return ['    <collision>', f'      <origin xyz="{f((x0 + x1) / 2000)} {f((y0 + y1) / 2000)} {f((z0 + z1) / 2000)}" rpy="0 0 0"/>',
                f'      <geometry><box size="{f((x1 - x0) / 1000)} {f((y1 - y0) / 1000)} {f((z1 - z0) / 1000)}"/></geometry>', '    </collision>']
    bb = next(s for p, s, _ in por_link["empurrador"] if p is not None and p.nome == "cremalheira").BoundingBox()
    colisoes["empurrador"] = caixa_col(bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax)
    CA, PI, GA, BO = (P["berco"][k] for k in ("calha", "piso", "gaiola", "boca"))
    fu = next(s for p, s, _ in por_link["berco"] if p is not None and p.nome == "funil").BoundingBox()
    caixas = {   # nome: (x0, x1, y0, y1, z0, z1) mm, frame do berço, tirado dos parâmetros do CAD
        "piso (cama do dardo)": (fu.xmin, BO["x1"], -CA["y1"], CA["y1"], PI["z0"], PI["z1"]),
        "parede +y da calha": (CA["x0"], CA["x1"], CA["y0"], CA["y1"], PI["z1"], -PI["z1"]),
        "parede −y da calha": (CA["x0"], CA["x1"], -CA["y1"], -CA["y0"], PI["z1"], -PI["z1"]),
        "teto da gaiola": (GA["x0"], GA["x1"], -CA["y1"], CA["y1"], GA["z0"], GA["z0"] + GA["esp"]),
    }
    colisoes["berco"] = sum((caixa_col(*c) for c in caixas.values()), [])
    # cada caixa tem de estar DENTRO do sólido do CAD (não inventar parede): o volume da caixa ∩ peças do berço
    pecas_berco = [s for p, s, _ in por_link["berco"] if p is not None]
    for nome, c in caixas.items():
        cx = cq.Solid.makeBox(c[1] - c[0], c[3] - c[2], c[5] - c[4], cq.Vector(c[0], c[2], c[4]))
        dentro = sum(cx.intersect(s).Volume() for s in pecas_berco)
        conferir(f"colisão '{nome}' dentro do CAD", dentro > 0.97 * cx.Volume(), f"{dentro / cx.Volume() * 100:.1f}% da caixa é peça")
    print(f"  base sobe {z_base * 1000:.1f} mm (fundo da caixa no chão); colisão: cremalheira {bb.xlen:.1f}×{bb.ylen:.1f}×{bb.zlen:.1f} mm "
          f"+ {len(caixas)} caixas do caminho do dardo; volantes sem colisão")

    print("\n6. URDF")
    texto = escrever_urdf(visuais, inerciais, z_base, colisoes)
    caminho = os.path.join(PKG, "urdf", "torreta.urdf")
    with open(caminho, "w", encoding="utf-8") as fh:
        fh.write(texto)
    ET.fromstring(texto.split("\n", 1)[1])
    res = subprocess.run(["bash", "-c", f"source /opt/ros/jazzy/setup.bash && check_urdf '{caminho}'"], capture_output=True, text=True)
    conferir("check_urdf aceita", res.returncode == 0 and "Successfully Parsed" in res.stdout,
             res.stdout.strip().splitlines()[0] if res.stdout else res.stderr.strip()[:200])
    for j in JUNTAS:
        extra = f"  mimic {j['mimic'][1]:+.3g} × {j['mimic'][0]}" if "mimic" in j else "  (COMANDADA)"
        lim = "" if j["tipo"] == "continuous" else f"  [{j['lim'][0]:+.3f}, {j['lim'][1]:+.3f}]"
        print(f"    {j['nome']:<16} {j['tipo']:<10}{lim}{extra}")

    print("\n7. torreta_gazebo.urdf.xacro (escrito à mão; conferido aqui)")
    xa = os.path.join(PKG, "urdf", "torreta_gazebo.urdf.xacro")
    yaml_ = os.path.join(PKG, "config", "controladores.yaml")
    res = subprocess.run(["bash", "-c", f"source /opt/ros/jazzy/setup.bash && xacro '{xa}' controladores:='{yaml_}'"], capture_output=True, text=True)
    conferir("o xacro expande", res.returncode == 0, res.stderr.strip()[:200])
    if res.returncode == 0:
        rz = ET.fromstring(res.stdout.split("\n", 1)[1] if res.stdout.startswith("<?xml") else res.stdout)
        rc = rz.find("ros2_control")
        cmd = {j.get("name"): [c.get("name") for c in j.findall("command_interface")] for j in rc.findall("joint")}
        movidas = {j["nome"] for j in JUNTAS}
        comandadas = {j["nome"] for j in JUNTAS if "mimic" not in j}
        conferir("ros2_control cobre todas as juntas móveis", set(cmd) == movidas, f"{sorted(set(cmd) ^ movidas) or 'iguais'}")
        conferir("comando só nas juntas dos motores; mimic só com estado",
                 {n for n, c in cmd.items() if c} == comandadas, ", ".join(f"{n}: {c[0]}" for n, c in cmd.items() if c))
        conferir("volante em velocidade, servos em posição",
                 cmd.get("volante_motor") == ["velocity"] and all(cmd.get(n) == ["position"] for n in ("pan_motor", "tilt", "sg90")))
        import yaml as _y
        cfg = _y.safe_load(open(yaml_, encoding="utf-8"))
        nos_ctrl = set()
        for k, v in cfg.items():
            if k != "controller_manager":
                nos_ctrl |= set(v["ros__parameters"]["joints"])
        conferir("os controladores comandam exatamente as juntas dos motores", nos_ctrl == comandadas, f"{sorted(nos_ctrl)}")
    # webcam simulada (webcam:=true, rastreio): a câmera tem de estar no centro da face da lente do CAD
    res = subprocess.run(["bash", "-c", f"source /opt/ros/jazzy/setup.bash && xacro '{xa}' controladores:='{yaml_}' webcam:=true"],
                         capture_output=True, text=True)
    conferir("o xacro expande com webcam:=true", res.returncode == 0, res.stderr.strip()[:200])
    if res.returncode == 0:
        import webcam as wc
        rz = ET.fromstring(res.stdout.split("\n", 1)[1] if res.stdout.startswith("<?xml") else res.stdout)
        jw = next(j for j in rz.findall("joint") if j.get("name") == "webcam_fixa")
        xyz = [float(v) * 1000 for v in jw.find("origin").get("xyz").split()]
        bb = wc.webcam_ref()[0]["lente"].BoundingBox()
        lente = (bb.xmax, (bb.ymin + bb.ymax) / 2, (bb.zmin + bb.zmax) / 2)
        conferir("webcam_link no berço = centro da face da lente no CAD (±0,1 mm)",
                 jw.find("parent").get("link") == "berco" and max(abs(a - b) for a, b in zip(xyz, lente)) <= 0.1,
                 f"xacro ({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}) × CAD ({lente[0]:.1f}, {lente[1]:.1f}, {lente[2]:.1f}) mm")
        fov_h = 2 * math.atan(math.tan(math.radians(wc.W["fov"] / 2)) * 16 / math.hypot(16, 9))
        sen = rz.find(".//sensor[@name='webcam']")
        conferir("campo de visão horizontal = 78° diagonal em 16:9 (±0,001 rad)",
                 abs(float(sen.find("camera/horizontal_fov").text) - fov_h) < 1e-3, f"{math.degrees(fov_h):.2f}° = {fov_h:.4f} rad")
    escrever_urdf_video(texto)
    print("\n8. Dardo solto (urdf/dardo.sdf) e config/dardo.json")
    escrever_dardo(dardo_cad, z_base)
    exigir_tudo_ok(ok)
    print(f"  gravado: {caminho}")
    return visuais


if __name__ == "__main__":
    main()
