# Fontes: componentes da torreta

Pesquisa de 2026-11-05. Os mesmos números, estruturados, estão em `componentes.json` (cada valor com fonte, item e tipo). O número que não está publicado fica como **null**; o deduzido vem marcado como **dedução**; o que vem de loja aparece marcado como **loja**.

Tipos: **datasheet** (documento do fabricante), **loja**, **medição publicada**, **dedução** e **referência cruzada** (modelo CAD de terceiros).

---

## 1. Feetech STS3215 (C001, 7,4 V)

**Fonte principal:** a PRODUCT SPECIFICATION da Feetech, edição A/0 de 2020-04-10, na tradução automática do Google hospedada pela Core Electronics: <https://core-electronics.com.au/attachments/uploads/sts3215-smart-servo-datasheet-translated.pdf>. O layout da tradução embaralha a tabela, então os valores foram lidos na **imagem** das páginas 3, 4 e 6. O site da Feetech (feetechrc.com) não mostrou a página do STS3215 ao robô: **conferência manual**.
**Fonte secundária:** a "Evaluation" da Feetech (Mantech): <https://www.mantech.co.za/datasheets/products/sts3215-200620a-1.pdf>. Ela traz 45,2×24,7×35, 19,5 kg.cm a 7,4 V, 52 rpm a 7,4 V, redução 1:345 e resolução de 0,088°.

| Grandeza | Valor | Item do documento |
|---|---|---|
| Caixa | 45,23 × 24,73 × 35 mm | item 9 (desenho); 6-1 "45.2X24.7X35" |
| Altura do corpo no lado do eixo | 29 mm | item 9, vista lateral "29" |
| Degraus da tampa traseira | 32 e 35 mm | item 9 (a superfície de cada cota não está clara) |
| Eixo de saída ao centro da caixa | 12,5 mm (ao longo do comprimento), centrado na largura | item 9 "12.5"; confirmado no STEP do SO-ARM100 |
| Eixo de saída à face frontal | 10,1 mm | **dedução** (45,23/2 − 12,5) |
| Estria acima da face de cima | 3,4 mm | item 9 "3.4" |
| Eixo passivo abaixo da face de baixo | 4,1 mm, Ø6 | item 9 "4.1", "Ø6" |
| Altura total, da estria ao eixo passivo | 36,5 mm | item 9 "36.5" (3,4 + 29 + 4,1 = 36,5 fecha) |
| Estria | 25 dentes, diâmetro externo 5,9 mm | 6-9 "25T/OD5.9mm" |
| Parafuso do horn | M3×6, de máquina | 6-13 "M3X6"; desenho "M3X6 机牙螺丝" |
| Parafusos da caixa | B-PA2.0 autoatarraxante (cantos), PA3.0×5 (fundo) | item 9 |
| Abas de fixação | **não tem**: fixa por furos nas faces de cima e de baixo | item 9 |
| Furos de fixação: rosca e posição | **null** no desenho da Feetech | conferência manual |
| Massa | 55 ± 1 g | 6-8 "55±1g" |
| Tensão | 4 a 7,4 V | 5-7 |
| Torque de stall | 19,5 kg.cm (1,912 N·m) a 7,4 V; 16,5 kg.cm (1,618 N·m) a 6 V | 5-4 |
| Torque nominal | 5 kg.cm (0,490 N·m) a 7,4 V; 4 kg.cm (0,392 N·m) a 6 V | 5-8 |
| Velocidade sem carga | 0,192 s/60° (52 rpm) a 7,4 V; 0,238 s/60° (42 rpm) a 6 V | 5-2 |
| Corrente de stall / nominal / sem carga | 2,5 / 0,65 / 0,15 A a 7,4 V; 2 / 0,5 / 0,13 A a 6 V | 5-5, 5-9, 5-3 |
| Kt; resistência do motor | 8 kg.cm/A; 2,5 Ω | 5-11, 5-10 |
| Faixa | 360° (0 a 4096), sem batente | 7-7, 6-2 |
| Resolução | 0,088° (360°/4096) | 7-8 |
| Encoder | magnético, 12 bits, absoluto | 6-6 |
| Folga | ≤ 0,5° (máximo) | 6-11 (o "ÿ" da tradução é o "≦", como no SCS0009) |
| Redução | 1/345 | 6-10 |
| Engrenagens; caixa | cobre (metal); PA+GF | 6-4, 6-3 |
| Rolamento | **sim**, de esferas | 6-5 "Ball bearings" |
| Motor | com núcleo ("Core Motor") | 6-14 |
| Carga radial e axial | **null** | não publicadas |

**Divergência resolvida (0,19 contra 0,24 s/60°):** os dois números estão na tabela, cada um numa tensão: 0,192 s/60° é a 7,4 V e 0,238 s/60° é a 6 V. A conta fecha: 60°/0,192 s = 312,5 °/s = 52 rpm, e 60°/0,238 s = 42 rpm.

**Horn padrão:** a especificação diz "10 Accessories: No Accessories" e não cota o horn. Para isso: **conferência manual**. Como referência cruzada, o STEP do SO-ARM100 modela um horn de Ø20 × 2,5 mm com 4 furos de Ø2,5 em (±4,95; ±4,95) do eixo, ou seja, num círculo de 14,0 mm (**dedução**: 2·√2·4,95). O horn passivo tem Ø20 × 2,1 mm.

**Referência cruzada de geometria (SO-ARM100, Apache-2.0):**
- STEP: <https://github.com/TheRobotStudio/SO-ARM100/blob/main/STEP/SO100/STS3215_03a.step>, cópia em `STS3215_03a_SO-ARM100.step`.
- STL: `Simulation/SO101/assets/sts3215_03a_v1.stl` e `sts3215_03a_no_horn_v1.stl`, no mesmo repositório.
- Medido no STEP com o CadQuery: caixa de 45,4 × 24,8 mm (bate com o desenho); eixo em x = +12,5 mm a partir do centro; furos-piloto de Ø1,5 com rebaixo de Ø4,2 em y = ±10,25 e x = +4,2 / −16,5 (em cima) e +4,2 / −20,3 (embaixo).
- É modelo de terceiros: serve para conferir, não substitui o desenho.

**12 V (C018):** existe. Loja: <https://openelab.io/da/products/feetech-sts3215-c018-servo-12v>, "STS3215-C018 12V 30kg.cm", 1/345 (lido no resultado de busca). Não há datasheet oficial: **conferência manual**.

## 2. Rolamentos 6808-2RS e 608-2Z

**Fonte:** o catálogo SKF *Rolling bearings* (PUB BU/P1 17000/1 EN), tabela 1.1, em <https://www.skf.com/binaries/pub12/Images/0901d196802809de-Rolling-bearings---17000_1-EN_tcm_12-121486.pdf>. O catálogo inteiro tem 23 MB (SHA-256 `65e0919bee24975b9563ca0d9adf721a9e7efcf6d0fa32c18b8d119d37b190b2`); foi guardado só o extrato das p. 264-265, 270-271 e 278-279. As páginas de produto do skf.com montam os dados por JavaScript e não abrem para o robô.

| | 61808-2RS1 (= 6808-2RS) | 608-2Z |
|---|---|---|
| d × D × B | 40 × 52 × 7 mm | 8 × 22 × 7 mm |
| r1,2 min | 0,3 mm | 0,3 mm |
| d1 ou d2 / D2 (≈) | d2 42,1 / D2 49,3 | d1 12,1 / D2 19,2 |
| C (dinâmica) | 4,49 kN | 3,45 kN |
| C0 (estática) | 3,75 kN | 1,37 kN |
| Pu (limite de fadiga) | 0,16 kN | 0,057 kN |
| Rotação de referência | – (vedação de contato) | 75 000 r/min |
| Rotação limite | 7 500 r/min | 38 000 r/min |
| Massa | 0,034 kg | 0,013 kg |
| Número e Ø das esferas | **null** | **null** |

- 6808 = 61808 na nomenclatura da SKF (série de dimensões 18): **dedução** pela norma ISO de dimensões.
- A linha do 61808-2RS1 saiu impressa na p. 270, dentro da seção de d 15-17 mm, mas os valores são de d = 40: o da min é 42 e o Da max é 50, iguais aos do 61808-2RZ da p. 278. É erro de diagramação do catálogo.

## 3. Motor tamanho 130

**Mabuchi FA-130RA:** o PDF da Mabuchi (2005) está numa cópia hospedada pela Pololu: <https://www.pololu.com/file/0J11/fa_130ra.pdf>. No mabuchi-motor.com o PDF antigo não existe mais: **conferência manual**.

| Grandeza | Valor | Item |
|---|---|---|
| Carcaça | Ø20,1 mm, com faces planas a 15,1 mm uma da outra | desenho "ϕ20.1", "15.1" |
| Lata; tampa traseira; total | 25,0; 2,3; 38,0 mm | desenho |
| Eixo | Ø2,0 mm, saindo 9,4 mm da face da lata | "ϕ2.0", "9.4" |
| Ressalto do mancal | Ø6,15 × 1,7 mm | desenho |
| Furos de fixação na face | **null**: o desenho não mostra nenhum | prender por berço; ver nota abaixo |
| Massa | 17 g (aprox.) | "WEIGHT : 17g (APPROX)" |
| FA-130RA-18100 (3 V nominal, faixa 1,5-3,0 V) | 12 300 rpm e 0,15 A sem carga; stall de 3,53 mN·m e 2,10 A; máxima eficiência a 9 710 rpm e 0,56 A | linha da tabela |
| FA-130RA-2270 (1,5 V) | 9 100 rpm; stall de 2,55 mN·m e 2,20 A | linha da tabela |
| FA-130RA-14150 (3 V, faixa 1,5-4,5 V) | 8 300 rpm; stall de 2,11 mN·m e 0,90 A | linha da tabela |

**Motor de lançador (loja):** OOD Kraken 130 3S: 35 000 rpm, stall de 740 g·cm e 26 A, 11,1 V ideal, formato 130. A página <https://outofdarts.com/products/ood-kraken-130-3s-neodymium-high-performance-motor> respondeu HTTP 429, então esses números vieram do resultado de busca: **conferência manual**. Não achei datasheet real de motor 130 de lançador.

**Nota:** motor 130 de lançador costuma vir com 2 furos M2 na face. O Gryphon pede "M2 screws: 4-5mm length for motor mounting". A distância entre os furos não foi achada em datasheet: **null**, conferência manual.

## 4. Micro servo SG90 (TowerPro)

**Fonte:** as páginas da TowerPro, <https://www.towerpro.com.tw/product/sg90-analog/> e <https://www.towerpro.com.tw/product/sg90-7/> (Digital). A TowerPro **não publica desenho cotado**.

| Grandeza | Valor | Item |
|---|---|---|
| Caixa | 23 × 12,2 × 29 mm | "Dimension: 23×12.2x29mm" |
| Massa | 9 g | "Weight: 9g" |
| Torque de stall | 1,8 kg.cm (0,177 N·m) a 4,8 V | "Stall torque: 1.8kg/cm(4.8v)" |
| Velocidade | 0,12 s/60° (Analog); 0,10 s/60° (Digital), a 4,8 V | páginas |
| Tensão | 4,8 V ("4.8V – 6V are fine") | página |
| Furos das abas, posição do eixo, estria | **null** | conferência manual (medir) |

Há uma compilação sem autor, <https://www.friendlywire.com/projects/ne555-servo-safe/SG90-datasheet.pdf>, com A 32, B 23, C 28,5, D 12, E 32 (comprimento com as abas) e F 19,5 (da base até a aba). Marquei como **loja**, porque a mesma folha diz 2,5 kg.cm e 14,7 g, o que contradiz a TowerPro.

## 5. Dardo de espuma tipo Elite e lançador de volante

**Dardo:**
| Grandeza | Valor | Fonte | Tipo |
|---|---|---|---|
| Massa | 1,02 g | BlasterHub (2018-12-20), tabela "Nerf Elite \| 1.02g \| 2.89″"; balança de 0,01 g; <https://blasterhub.com/?p=15312> | medição publicada |
| Comprimento | 73,4 mm (2,89″) | idem | medição publicada |
| Comprimento | 70 mm ("2.75 inches / 7 cm") | BlasterCentral, <https://www.blastercentral.com/nerf-darts/> | medição publicada (arredondada) |
| Diâmetro | 13 mm ("0.5 inches / 1.3 cm", ou seja, 12,7 mm) | BlasterCentral | medição publicada (arredondada) |
| Diâmetro | 12 mm ("7.2cm x 1.2cm") | listagem de loja (resultado de busca) | loja |
| Ponta: diâmetro e forma | **null** | conferência manual (paquímetro) | — |

**Volante:**
- **Espaçamento original entre os motores, 43,5 mm:** Silver Fox / OFP diz "43.5 mm spacing is the same spacing as the stock cages" (resultado de busca). Tipo: loja.
- **Projeto aberto Gryphon (CC0), README:** <https://github.com/Flygonial/The-Gryphon-Foam-Dart-Blaster>
  - motores 130 a 180 (20,4 mm), escovados;
  - volantes de Ø33 a 38 mm, para eixo de 2 mm;
  - espaçamento dos motores de 38,5 a 43,5 mm;
  - cano de 14, 14,5 ou 15 mm;
  - cabe numa mesa de impressão de 120 mm.
  - É o modelo aberto mais próximo do nosso.
- **Volante original do Stryfe:** Ø33,7 mm, com folga de 10 mm. Fonte: fórum BritNerf (resultado de busca; o site respondeu 403: **conferência manual**). Tipo: medição publicada.
- **Definição de folga:** espaçamento dos motores menos o diâmetro de raiz do volante. Fonte: wiki r/Nerf, que cita raiz de 33,5 mm no Stryfe. O Reddit bloqueia robô: **conferência manual**.
- **MIT HTMAA:** "the gap between the flywheels is 9mm" (não diz se mediu), <https://fab.cba.mit.edu/classes/863.23/EECS/people/Sam/week13.html>.
- **Compressão usual:** dardo de 12,7 a 13 mm numa folga de 9 a 10 mm dá ≈ 2,7 a 4 mm no total (20 a 30% do diâmetro). Tipo: **dedução**.

**Velocidade:**
- Stryfe original, motores 130 com 4 pilhas AA: "just under 72 feet per second" (<https://blasterpiece.com/reviews/nerf-stryfe-blaster/>). Tipo: medição publicada.
- Volante original Dart Zone / Adventure Force Light Command: 93,7 fps (BlasterHub). Tipo: medição publicada.
- Motor 130 em 3S: **null** (não achei medição confiável).

## 6. Flashforge Inventor (a original, com dupla extrusão)

**Fonte:** a tabela "FLashforge 3d printers parameter" da Flashforge (2019), hospedada pela Farnell: <https://www.farnell.com/datasheets/2829239.pdf>, coluna Inventor. O site flashforge.com não lista mais a Inventor: **conferência manual** no manual impresso.

- Volume de impressão: **230 × 150 × 160 mm**. Algumas lojas dizem 228 × 150 × 160.
- Extrusoras: 2.
- Bico: 0,4 mm.
- Camada: 0,05 a 0,4 mm.
- Precisão de impressão: ±0,1 mm.
- Precisão de posição: XY 0,011 mm, Z 0,0025 mm.
- Filamento: 1,75 mm (±0,07).
- Temperatura máxima do bico: 240 °C.
- Área útil com as duas extrusoras: **null**. O bico 2 costuma tirar largura útil; conferir no FlashPrint.

## 7. Webcam Logitech C920

**Fonte:** <https://www.logitech.com/en-za/products/webcams/c920-pro-hd-webcam.html>. As especificações estão no JSON da página. A página da C920 hoje é a da **C920s**.

- Com o clipe fixo: altura 43,3 mm, largura 94 mm, profundidade 71 mm.
- Massa: 162 g.
- Cabo: 1,5 m.
- Vídeo: 1080p/30 fps e 720p/30 fps; 3 MP.
- Campo de visão diagonal: 78°, fixo.
- Foco: autofoco.
- "Tripod-ready universal mounting clip" (o tripé não vem junto).
- Rosca 1/4"-20: **dedução** (é o padrão de tripé, mas a Logitech não escreve a rosca).

## 8. Inserto de latão M3 (Ruthex RX-M3x5.7)

**Fonte:** a tabela da 3DJake, <https://www.3djake.ch/de-CH/ruthex/gewindeeinsatz-m3-100-stueck> (**loja**; código GE-M3x57-001). O ruthex.de e o cnckitchen.store responderam HTTP 429: **conferência manual**.

| Grandeza | Valor |
|---|---|
| D1, diâmetro externo do recartilhado (**dedução** pelo desenho padrão da Ruthex) | 4,6 mm |
| D2, guia de baixo (**dedução**) | 3,9 mm |
| D3, furo recomendado (**dedução**: a página não define D3) | 4,0 mm |
| Comprimento L | 5,7 mm |
| Parede mínima W | 1,6 mm |

**CNC Kitchen**, <https://www.cnckitchen.com/blog/tipps-amp-tricks-fr-gewindeeinstze-im-3d-druck-3awey>:
- furo cego ≈ 1 mm mais fundo que o inserto;
- peça de teste com furos de 4,0 a 4,4 mm (é a faixa testada, não uma recomendação);
- ferro 10 a 20 °C acima da temperatura de impressão.

O pôster de dimensões da CNC Kitchen é imagem: **conferência manual**.

## 9. Engrenagem de módulo 1,5

- **Mínimo de dentes sem rebaixo, com 20°: 17.** Fonte: KHK, <https://khkgears.net/new/gear_knowledge/gear_technical_reference/involute_gear_profile.html>, "the minimum number of teeth free of undercutting is 17". A conta é z = 2/sin²α = 17,1.
- Com 16 dentes ou menos ainda dá para usar, se a resistência e a razão de contato aceitarem. Para z < 17, a KHK recomenda correção de perfil positiva.
- **Referência de folga e largura de face em plástico:** catálogo KHK PS1.5 (nylon MC901, m1,5, JIS N9), <https://mx.c.misumi-ec.com/en/book/KHK1_USA_M01/pdf/0144.pdf>.
  - Folga normal teórica de 0 a 0,38 mm (z 15-32), 0 a 0,40 mm (z 35-65) e 0 a 0,42 mm (z 70-100).
  - Largura de face de 15 mm (= 10 × módulo).
  - A menor engrenagem do catálogo tem 15 dentes.
  - Aviso da KHK: temperatura e umidade mudam o diâmetro e a folga; par plástico com plástico sem lubrificação esquenta e dilata.
- Folga para PLA impresso: **null**, sem fonte técnica. Os números da KHK são de nylon usinado.

## 10. ESP32-S3-DevKitC-1

**Fonte:** o desenho da Espressif, rev 1.0 de 2021-03-12: <https://dl.espressif.com/dl/DXF_ESP32-S3-DevKitC-1_V1_20210312CB.pdf> (e o `.dxf`). O guia do usuário é <https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.0.html>.

- Placa: **62,74 × 25,40 mm**.
- Espessura: **null**.
- Massa: **null**.
- Furos de fixação: **nenhum**.
- USB: 2 conectores na borda curta, do lado oposto à antena: J2 "UART" (ponte USB-UART) e J4 "USB" (USB nativo). Na v1.0 são Micro-USB.
- Pinos: passo de 2,54 mm, com centro a 1,27 mm da borda longa. O último pino fica a 8,00 mm da borda dos USB (leitura do desenho).
- A antena do módulo passa da borda de cima; a saliência não está cotada.
- Conferir a revisão da placa comprada, porque o desenho é da v1.

## 11. Adaptador serial para os Feetech

**Waveshare Bus Servo Adapter (A):** <https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)>

- Placa: 42 × 33 mm.
- Furos: Ø2,5 mm. Posições: **null**, conferência manual.
- Alimentação: P4 5,5 × 2,1 mm.
- **Divergência:** o wiki diz "Input voltage: 9~12.6V (should match servo voltage)", mas o STS3215-C001 é de 4 a 7,4 V. Conferir antes de usar com o C001.
- Furação (página do produto <https://www.waveshare.com/bus-servo-adapter-a.htm>): "Mounting hole spacing: 37.00 × 28.00mm".
- **Esquema elétrico** (`Waveshare_Bus_Servo_Adapter_A_esquema.pdf`, lido em 2026-11-06), o que resolve a divergência: o borne KF350 passa pelo P-MOSFET AO4407 (proteção contra inversão) e vira a rede DC_IN0. O jack DC005 vai direto para DC_IN0. DC_IN0 vai DIRETO ao pino 2 dos conectores dos servos (H3/H5, "MAX:6A"), sem regulador no caminho. Por isso o fabricante diz que a entrada tem de ser igual à tensão do servo: os 9–12,6 V são a faixa dos ST de 12 V que ele vende. Há um TVS SMAJ15CA (15 V) em DC_IN0. A lógica é 3,3 V (AMS1117), alimentada por 5 V da USB-C (VDDUSB) OU por um buck U5 a partir de DC_IN0 (FB 0,8 V, 56k/10k → ~5,3 V), juntos por diodos B5819. O chip do U5 não está identificado: com 7,0 V na entrada, a lógica sem USB depende dele (**a conferir no primeiro teste**; com a USB-C ligada, a lógica funciona de qualquer jeito). O meio-duplex usa SN74LVC1G125/126 a 3,3 V, então serve direto para o ESP32 pelo H2 (GND, RXD, TXD) com o jumper H4 no modo UART. O conversor USB é o CH343P.
- Régua: o SO-ARM100 (README) usa o STS3215-C001 com fonte de **5 V** ("16.5kg.cm at 6V (and likely slightly less for a 5V power supply)") e uma "Motor Control Board" cujo link da Amazon é da Waveshare (dedução pelo endereço do link; o README não nomeia a placa).

**FE-URT-1 (Feetech):** não é mais necessário, pelo esquema acima. O feetechrc.com não resolve DNS para o robô (2026-11-06).

---

## 12. Eletrônica da caixa (2026-11-06)

**Buck XL4015 (módulo), 3 unidades** (7,0 V servos, 3,0 V motores, 5 V SG90): ficha da Handsontec <https://www.handsontec.com/dataspecs/module/XL4015-5A-PS.pdf>. 'Size: 51x26.3x14 (LxWxH) (mm)', entrada 8–36 V, saída 1,25–32 V, até 5 A. A furação não é cotada, e as lojas divergem (54 × 23, 62 × 26): a caixa usa berço com cerca + fita dupla-face. **Medir a placa comprada.** O MP1584 ficou de fora: o site da MPS bloqueia o robô, e três módulos iguais simplificam a compra.

**MOSFET AO3400A** (motores do volante): <https://www.aosmd.com/res/datasheets/AO3400A.pdf>, Rev 3.1. 30 V, 5,7 A, RDS(on) < 32 mΩ a 4,5 V e < 48 mΩ a 2,5 V. Com o gate a 3,3 V do ESP32, vale o pior caso de 48 mΩ: a partida de 4,2 A dissipa ~0,85 W por instantes, e a corrida a ~1,1 A, ~0,06 W.

**Diodo 1N5819** (roda-livre dos motores): Vishay <https://www.vishay.com/docs/88525/1n5817.pdf>. 40 V, 1 A médio, 25 A de pico, DO-41.

**Jack DC PJ-002AH** (Same Sky, ex-CUI): <https://www.sameskydevices.com/product/resource/pj-002ah.pdf>. 5 A, 24 V, pino Ø2,0 (plugue 5,5 × 2,1), corpo 9,0 × 11,0 × 14,4, boca Ø6,5 com centro a 6,5 da base, entrada do plugue 9,5 mm. É de placa (ângulo reto): vai num berço impresso com a boca rente à face de FORA da parede. Os jacks de painel achados com ficha (Lumberg NEB/J 21 C) são de 0,5 A.

---

## Arquivos baixados (nesta pasta)

| Arquivo | Origem | SHA-256 |
|---|---|---|
| `STS3215_especificacao_feetech_traduzida.pdf` | core-electronics.com.au (spec Feetech A/0, traduzida) | `6647638660ae4d5a5e8fc7ab02c2b1a8890613b1666590908881c636f6bc2ace` |
| `STS3215_desenho.jpg` | imagem da p. 6 do PDF acima (desenho cotado) | `51b17fd984512083216c7cd3e82165820b507a4c34c16ff93220b48cf7603f00` |
| `STS3215_avaliacao_feetech_mantech.pdf` | mantech.co.za | `99f2ca7ba0c00819934396e9f50f6beb22ae8ac2132ddd969c6bb7f2f3306c45` |
| `STS3215_03a_SO-ARM100.step` | github TheRobotStudio/SO-ARM100 (Apache-2.0) | `cacd717c2f22bec856fdace2c3bc9fa58084d54b6e70378eae90562f34df93ef` |
| `SKF_17000_extrato_608_61808.pdf` | catálogo SKF 17000 (p. 264-265, 270-271, 278-279) | `1a8cac6a4c9dce816eee13d0625271bb2c97a6627c0b57cce4a80c441d5d7416` |
| `Mabuchi_FA-130RA.pdf` | pololu.com (PDF da Mabuchi) | `bbc1ab250545cc379ed3965d4a706a4f9636a544be11d4f6488b30b89dc171a6` |
| `SG90_compilacao_nao_oficial.pdf` | friendlywire.com (compilação, não é da TowerPro) | `10c9ef2831f9d1a00fa9d5a436372e19a41c80be36e17938af204341f07accc3` |
| `Flashforge_parametros_farnell.pdf` | farnell.com (tabela da Flashforge) | `6e01d0fd70ede966cfcbec5d0da3b2dc4291d0825b9e16ca22492806f53f1cf1` |
| `KHK_PS1.5_nylon_m1.5.pdf` | misumi-ec.com (catálogo KHK, p. 142) | `ba71ba69f44bc0cb201aaef861692de89a5863c94f2deb35b874fb20cbc860b3` |
| `ESP32-S3-DevKitC-1_dimensoes.pdf` | dl.espressif.com | `5cae10183739d8221ad51a12332585b94b3079ad3fe2231a48c07aa1f51eeef3` |
| `XL4015_modulo_handsontec.pdf` | handsontec.com | `7015280612acdf4a808984f39005d937a22068f0c0967582a6e27a55ff7e1de6` |
| `AO3400A_aosmd.pdf` | aosmd.com (Rev 3.1) | `9c60d0b6c1ddc7609a4b788a91181468d8ffe6c3e9ba425c3d8ffc5ba88c5033` |
| `1N5817-1N5819_vishay.pdf` | vishay.com (doc 88525) | `4465c825da3be36cc8a180c252d5d80e7651d9dc237cd3d09822a759f39c970b` |
| `PJ-002AH_sameskydevices.pdf` | sameskydevices.com | `1ff7866e134086429bccde7ae2556139ed12dd5b851b8b3f0ea911bd77924145` |
| `Waveshare_Bus_Servo_Adapter_A_esquema.pdf` | files.waveshare.com (link do wiki: Bus Servo Adapter (A)-Sch.pdf) | `702645126730724854f8b15509cee5db562c8b292db41306211800d90d54d649` |
| `ESP32-S3-DevKitC-1_dimensoes.dxf` | dl.espressif.com | `0a700d9d1cd4a8f3d98a434b120d685524a9962916320b3bb98d74486789fda1` |

## Conferência manual (o robô não conseguiu)

- feetechrc.com: as páginas do STS3215 (C001 e C018), as cotas do horn padrão, a rosca dos furos de fixação e o FE-URT-1.
- skf.com: as páginas de produto, que montam os dados por JavaScript. O catálogo PDF cobre os mesmos dados.
- outofdarts.com (HTTP 429), ruthex.de e cnckitchen.store (HTTP 429), reddit.com (bloqueio a robô) e tapatalk BritNerf (HTTP 403).
- O PDF da FA-130RA no site da Mabuchi.
- O manual da Flashforge Inventor.
- Medir com paquímetro: a ponta do dardo, os furos das abas e a estria do SG90, os furos do motor 130 de lançador e a posição dos furos do adaptador Waveshare.
