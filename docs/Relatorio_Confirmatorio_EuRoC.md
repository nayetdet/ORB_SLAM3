# Relatório final — rodada confirmatória EuRoC (ORB-SLAM3 + reconstrução densa)

Gerado a partir de `~/orb_work2/results_pilot/euroc_confirm` e `euroc_confirm_stats`. Desenho e critérios: `evaluation/PRE_REGISTRATION_confirmatory.md` (escrito antes de rodar).

## 1. Desenho
- 4 braços: **base** (baseline, shutdown sincronizado), **c1** (baseline + ajuste global final), **faithful** (denso fiel à tese), **imp** (fila 3, baixa prioridade, ajuste final, filtro de voxel corrigido).
- 8 sequências EuRoC, 12 execuções por braço, entrelaçadas (quadrado latino, semente 11), 1000 features, CPU limitada a 2 núcleos físicos / 4 threads, `--ulimit nice=40:40`.
- Teste: permutação exata sobre a diferença de medianas; Holm sobre as 24 comparações braço×sequência; IC95% bootstrap da razão de medianas.

## 2. Execução
384 execuções, 4 falhas, todas isoladas (crash numérico do ORB-SLAM3 `SO3::exp nan`, já visto no baseline): base/MH04/run01, base/MH05/run05, faithful/V103/run01, imp/MH05/run05. Falhas são descartadas, não imputadas.

## 3. ATE RMSE (m, SE(3), trajetória de quadros) — mediana ± desvio, e variação % contra o baseline (p bruto / p Holm)

| Seq | base | c1 | faithful | imp |
|---|---|---|---|---|
| MH01 | 0.0410 ± 0.0079 | 0.0334 ± 0.0041<br>-18.4% (0.006 / 0.13) * | 0.0343 ± 0.0064<br>-16.2% (0.106 / 1.00) | 0.0312 ± 0.0069<br>-23.9% (0.021 / 0.41) * |
| MH02 | 0.0243 ± 0.0033 | 0.0163 ± 0.0015<br>-32.8% (0.000 / 0.00) ** | 0.0248 ± 0.0037<br>+1.8% (0.739 / 1.00) | 0.0153 ± 0.0011<br>-37.0% (0.000 / 0.00) ** |
| MH03 | 0.0353 ± 0.0052 | 0.0312 ± 0.0061<br>-11.4% (0.244 / 1.00) | 0.0309 ± 0.0069<br>-12.5% (0.236 / 1.00) | 0.0333 ± 0.0086<br>-5.5% (0.424 / 1.00) |
| MH04 | 0.0725 ± 0.0201 | 0.0599 ± 0.0171<br>-17.5% (0.267 / 1.00) | 0.0736 ± 0.0225<br>+1.4% (0.961 / 1.00) | 0.0532 ± 0.0120<br>-26.7% (0.012 / 0.25) * |
| MH05 | 0.0626 ± 0.0196 | 0.0577 ± 0.0181<br>-7.7% (0.764 / 1.00) | 0.0542 ± 0.0259<br>-13.4% (0.333 / 1.00) | 0.0554 ± 0.0165<br>-11.5% (0.541 / 1.00) |
| V101 | 0.0365 ± 0.0010 | 0.0371 ± 0.0009<br>+1.8% (0.296 / 1.00) | 0.0363 ± 0.0009<br>-0.4% (0.747 / 1.00) | 0.0363 ± 0.0009<br>-0.4% (0.913 / 1.00) |
| V102 | 0.0436 ± 0.0271 | 0.0313 ± 0.0110<br>-28.1% (0.225 / 1.00) | 0.0327 ± 0.0109<br>-25.0% (0.275 / 1.00) | 0.0301 ± 0.0058<br>-30.9% (0.012 / 0.25) * |
| V103 | 0.1046 ± 0.0544 | 0.1140 ± 0.0350<br>+9.0% (0.747 / 1.00) | 0.0915 ± 0.0727<br>-12.6% (0.450 / 1.00) | 0.1123 ± 0.0605<br>+7.4% (0.981 / 1.00) |

`**` = significativo após Holm (família de 24); `*` = só p bruto < 0,05.

## 4. Isolando cada efeito (variação % da mediana, p bruto)

| Seq | ajuste final (c1 vs base) | desacoplamento (imp vs c1) | tese (faithful vs base) | imp vs faithful |
|---|---|---|---|---|
| MH01 | -18.4% (0.006) | -6.7% (0.524) | -16.2% (0.106) | -9.3% (0.453) |
| MH02 | -32.8% (0.000) | -6.2% (0.201) | +1.8% (0.739) | -38.1% (0.000) |
| MH03 | -11.4% (0.244) | +6.7% (0.534) | -12.5% (0.236) | +8.0% (0.505) |
| MH04 | -17.5% (0.267) | -11.1% (0.418) | +1.4% (0.961) | -27.7% (0.028) |
| MH05 | -7.7% (0.764) | -4.1% (0.824) | -13.4% (0.333) | +2.2% (0.797) |
| V101 | +1.8% (0.296) | -2.1% (0.258) | -0.4% (0.747) | -0.0% (0.998) |
| V102 | -28.1% (0.225) | -4.0% (0.684) | -25.0% (0.275) | -8.0% (0.373) |
| V103 | +9.0% (0.747) | -1.5% (0.929) | -12.6% (0.450) | +22.8% (0.383) |

## 5. Contagens contra o baseline (8 comparações por braço)
| Braço | p bruto<0,05 | Holm<0,05 | piores com p<0,05 | IC95% superior da razão > 1,10 |
|---|---|---|---|---|
| c1 | 2 | 1 | 0 | 4 |
| faithful | 0 | 0 | 0 | 5 |
| imp | 4 | 1 | 0 | 3 |

Menor p bruto de imp vs c1 em todas as sequências: 0.201.

## 6. Comparação com a tese (ATE em m; a tese não declara alinhamento nem tipo de trajetória)
Colunas nossas: mediana de 12 e média das 3 primeiras execuções (como a tese), em SE(3)-quadros; entre colchetes: SE(3)-keyframes e Sim(3)-quadros.

| Seq | tese (Proposed) | ORB-SLAM3 publicado | base | faithful | imp |
|---|---|---|---|---|---|
| MH01 | 0.039 | 0.029 | 0.0410 / 0.0456 [0.0400, 0.0218] | 0.0343 / 0.0326 [0.0337, 0.0191] | 0.0312 / 0.0283 [0.0295, 0.0168] |
| MH02 | 0.028 | 0.019 | 0.0243 / 0.0209 [0.0202, 0.0224] | 0.0248 / 0.0255 [0.0208, 0.0245] | 0.0153 / 0.0161 [0.0139, 0.0148] |
| MH03 | 0.032 | 0.024 | 0.0353 / 0.0303 [0.0329, 0.0265] | 0.0309 / 0.0345 [0.0302, 0.0260] | 0.0333 / 0.0409 [0.0308, 0.0279] |
| MH04 | 0.112 | 0.085 | 0.0725 / 0.0880 [0.0780, 0.0657] | 0.0736 / 0.0658 [0.0795, 0.0672] | 0.0532 / 0.0612 [0.0656, 0.0429] |
| MH05 | 0.061 | 0.052 | 0.0626 / 0.0678 [0.0708, 0.0544] | 0.0542 / 0.0544 [0.0610, 0.0455] | 0.0554 / 0.0807 [0.0641, 0.0489] |
| V101 | 0.042 | 0.035 | 0.0365 / 0.0363 [0.0329, 0.0348] | 0.0363 / 0.0364 [0.0321, 0.0350] | 0.0363 / 0.0365 [0.0322, 0.0343] |
| V102 | 0.036 | 0.025 | 0.0436 / 0.0462 [0.0368, 0.0408] | 0.0327 / 0.0472 [0.0240, 0.0299] | 0.0301 / 0.0292 [0.0231, 0.0249] |
| V103 | 0.107 | 0.061 | 0.1046 / 0.1170 [0.1019, 0.1004] | 0.0915 / 0.0990 [0.0846, 0.0883] | 0.1123 / 0.0899 [0.1087, 0.1116] |

Ressalvas: hardware diferente (Ryzen 5 4600G limitado a 2 núcleos contra i7-7500U), tese = média de 3 execuções, convenção de ATE da tese desconhecida. Diferenças menores que o desvio entre execuções não são vitória nem derrota.

## 7. Métricas por construção e custo
| Seq | tempo de parede (s) base / c1 / faithful / imp | espera no shutdown (s) imp | ajuste final (s) imp | filtro de voxel imp: chamadas que estourariam / tratadas | nuvem run00 (MB) faithful / imp |
|---|---|---|---|---|---|
| MH01 | 210 / 211 / 210 / 212 | 0.0 | — | 0.0 / 0.0 | 181 / 175 |
| MH02 | 176 / 177 / 175 / 177 | 0.0 | — | 0.0 / 0.0 | 159 / 155 |
| MH03 | 154 / 155 / 154 / 155 | 0.0 | — | 8.0 / 8.0 | 374 / 251 |
| MH04 | 116 / 121 / 146 / 122 | 0.0 | — | 11.5 / 11.5 | 766 / 357 |
| MH05 | 129 / 133 / 144 / 135 | 0.0 | — | 11.0 / 11.0 | 749 / 370 |
| V101 | 167 / 167 / 167 / 167 | 0.0 | — | 0.0 / 0.0 | 35 / 31 |
| V102 | 100 / 101 / 100 / 100 | 0.0 | — | 0.0 / 0.0 | 51 / 49 |
| V103 | 122 / 123 / 121 / 122 | 0.0 | — | 0.0 / 0.0 | 106 / 101 |

(Nuvens só existem na run00: as demais foram apagadas com `--prune-dense`; n=1 por célula.)

## 8. Conclusões frente às premissas

1. **Comparável com a tese:** mesmas 8 sequências do EuRoC, 1000 features, média das 3 primeiras execuções reportada; alinhamento da tese é desconhecido, então as comparações numéricas com a tabela dela são indicativas.
2. **Alterações novas:** o ganho de ATE vem do **ajuste global final (C1)**: imp e c1 não diferem de forma significativa em nenhuma sequência (menor p bruto = 0.201). O desacoplamento da thread densa (fila limitada, baixa prioridade) **não** mostrou efeito mensurável no ATE nem com 2 núcleos; o denso fiel à tese não difere do baseline.
3. **Nada pior / melhorar ao menos uma métrica:** imp melhora o ATE do baseline com significância após Holm em 1 sequência(s) (MH02) e com p bruto em 4; **não houve piora significativa** em nenhuma sequência (0 com p<0,05), mas o IC95% superior da razão passa de 1,10 em 3 sequências, então a não-inferioridade não está estabelecida em todas com n=12.
4. **O que é por construção:** o filtro de voxel corrigido e o ajuste final são deterministas; o custo é tempo de parede (seção 7).
5. **Limites:** só EuRoC; TUM e KITTI não entram nesta rodada; 4 falhas isoladas; sem alinhamento da tese; nada commitado.

## 9. Tabela resumo: denso melhorado contra baseline

Gráfico comparativo: `docs/comparativo_metricas_euroc.png` (precisão por sequência com IC95% e tamanho da nuvem).

| Seq | Baseline (m) | Denso melhorado (m) | Variação (p bruto) | Melhorou? | Nuvem |
|---|---|---|---|---|---|
| MH01 | 0,0410 | 0,0312 | −24% (0,021) | 🟢 Sim (p bruto) | −3% |
| MH02 | 0,0243 | 0,0153 | −37% (<0,001) | ✅ Sim, sólido (Holm) | −3% |
| MH03 | 0,0353 | 0,0333 | −5,5% (0,42) | ➖ Igual (tendência) | −33% |
| MH04 | 0,0725 | 0,0532 | −27% (0,012) | 🟢 Sim (p bruto) | −53% |
| MH05 | 0,0626 | 0,0554 | −12% (0,54) | ➖ Igual (tendência) | −51% |
| V101 | 0,0365 | 0,0363 | −0,4% (0,91) | ➖ Igual | −12% |
| V102 | 0,0436 | 0,0301 | −31% (0,012) | 🟢 Sim (p bruto) | −4% |
| V103 | 0,1046 | 0,1123 | +7% (0,98) | ⚠️ Igual, sem garantia (IC95% da razão até 1,60) | −5% |

✅ passa também na correção de Holm (24 comparações); 🟢 p bruto < 0,05 sem passar no Holm (sinal, não prova); ➖ dentro do ruído; ⚠️ sem piora significativa, mas sem garantia. O ganho de ATE é atribuível ao ajuste global final (C1), não ao desacoplamento da thread densa; a redução da nuvem vem do filtro de voxel corrigido e é determinista.

## 10. Cobertura das métricas da tese (Tabelas VIII, IX, X e FPS)

Esta seção só usa dados que já existiam (nenhuma execução nova do SLAM). Tamanhos de arquivo foram medidos de novo nas nuvens/octomaps `run00` com duas ferramentas pequenas e reprodutíveis (`evaluation/harness/size_report.cc` e `leaf_sweep.cc`; saídas em `~/orb_work2/results_pilot/thesis_tables/`). MB = 10^6 bytes. Nuvem `run00` do `euroc_confirm` (braços faithful e imp, 1000 features, 2 núcleos) para EuRoC; nuvem `run00` do benchmark antigo de 39 h para TUM e KITTI (EuRoC antigo foi medido também, mas não é mostrado: 1200 features e baseline sem shutdown sincronizado).

### 10.1 Status por tabela

| Tabela / métrica da tese | Status | O que é comparável | O que não é |
|---|---|---|---|
| IV (TUM, ATE) | feito no benchmark antigo (39 h), não refeito na rodada confirmatória | mesmas sequências em parte | execuções do TUM foram perturbadas por I/O (28/50 faithful, 33/50 imp com iowait >= 10%), alinhamento da tese desconhecido |
| V (EuRoC, ATE) | feito, seção 6 | 8 sequências, 1000 features, média de 3 execuções | alinhamento da tese desconhecido; hardware diferente |
| VI (KITTI, ATE) | feito no benchmark antigo, não na confirmatória | 5 de 11 sequências (00, 03, 05, 07, 09) | alinhamento SE(3) contra Sim(3) muda o resultado em ordem de grandeza (KITTI 03: 1,32 m contra 0,28 m) |
| VII (fusão multi-sequência) | **não feito** | — | existe `Dense.loadCloud` e Atlas nativo, mas nenhuma rodada de fusão foi executada; falta tudo |
| VIII (tamanho x leaf size) | **feito de forma derivada** (10.2) | mesma estrutura (fr1_room, 5 leaf sizes, 3 formatos) | série derivada da nuvem a 0,01 m, não 5 execuções do SLAM; sequência/trajetória e número de keyframes diferem |
| IX (tamanho denso x octomap, razão) | **feito, com formatos comparáveis** (10.3) | razão média ASCII/.ot reproduz a da tese em TUM; razão binário/.ot e .bt é o achado metodológico (D2) | EuRoC/KITTI: a razão depende da razão entre voxel da nuvem e leaf do octomap, que a tese não publica; FR2_no_loop, KITTI 01/02 e EuRoC MH03 antigo não existem no nosso conjunto |
| X (tempos por etapa) | **parcial** (10.4): só o bloco "Dense Reconstruction" | 4 etapas densas + total, medidas pelo mesmo tipo de contador | linhas do ORB-SLAM3 (extração, BA local, laço...) exigem build `REGISTER_TIMES`, que não temos; V2_02 não foi rodada (usamos V102); hardware e profundidade estéreo diferentes |
| FPS (texto da seção 4.5) | **não medido** (10.5) | tempo de rastreamento do baseline em TUM/KITTI | harness segue o ritmo do dataset; tempo de rastreamento do denso sai com 2 casas decimais e EuRoC não imprime |

### 10.2 Tabela VIII: tamanho contra leaf size (TUM fr1_room, derivada)

Origem: nuvem `run00` de `tum_dense_faithful/fr1_room` (4,91 M de pontos a 0,01 m, 1 execução). Para cada leaf L: `VoxelGrid(L)` sobre essa nuvem e um `ColorOcTree(L)` montado como `InsertIntoOctomap` (só pontos ocupados, `prune()`). É uma aproximação de rodar o SLAM com `Dense.resolution = Dense.octomapResolution = L`; não repete o filtro por keyframe nem a fusão. "ASCII" é o tamanho que o `.pcd` teria em texto (extrapolado a partir de uma amostra de até 200 mil pontos escrita pelo PCL, ~43 B/ponto; exato quando N <= 200 mil).

| Leaf (m) | pontos | .pcd binário (nosso) | .pcd ASCII (nosso, est.) | .ot (nosso) | .bt (nosso) | tese: denso | tese: octomap | razão tese | razão nossa binário/.ot | razão nossa ASCII/.ot |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.01 | 4.637 M | 74.2 | 201.3 | 37.6 | 2.37 | 106.4 | 25.2 | 4.2 | 1.97 | 5.35 |
| 0.02 | 1.080 M | 17.3 | 46.8 | 6.51 | 0.40 | 31.6 | 6.4 | 4.9 | 2.65 | 7.20 |
| 0.05 | 0.124 M | 1.98 | 5.37 | 0.83 | 0.05 | 6.6 | 1.2 | 5.5 | 2.38 | 6.44 |
| 0.1 | 0.026 M | 0.41 | 1.11 | 0.20 | 0.01 | 1.2 | 0.23 | 5.2 | 2.06 | 5.57 |
| 0.2 | 0.006 M | 0.09 | 0.25 | 0.05 | 0.00 | 0.317 | 0.062 | 5.1 | 1.92 | 5.19 |

Leitura: (1) a tendência da tese (arquivo cai ~ com o quadrado do leaf, razão aproximadamente constante) é reproduzida. (2) A razão da tese (4,2 a 5,5) é da ordem da razão **ASCII/.ot** nossa (5,2 a 7,2) e não da binária (1,9 a 2,7). (3) Nos leafs grandes a tese tem a nuvem de ~43 B/ponto compatível com ASCII: em 0,05 m, 6,6 MB / 43 B = 153 mil pontos contra 124 mil nossos; em 0,1 m, 28 mil contra 26 mil; se fosse binário (16 B/ponto) a tese teria 412 mil pontos em 0,05 m, 3,3 vezes mais que nós. Isso reforça (não prova) a hipótese de que a tese mede `.pcd` em texto contra `.ot` binário. (4) Em 0,01 e 0,02 m os absolutos da tese são menores que os nossos (a tese teria ~2,5 M de pontos contra 4,6 M, se ASCII): trajetória/keyframes diferentes. (5) O `.bt` (só ocupação, sem cor) é de 14 a 16 vezes menor que o `.ot` colorido.

### 10.3 Tabela IX: tamanho do mapa denso contra o octomap, formatos comparáveis

Colunas: `.pcd` XYZRGB binário (o que o código grava), `.pcd` XYZRGB em texto (estimado), `.ot` ColorOcTree, `.bt` ocupação sem cor; razões com os dois lados no mesmo formato (binário/.ot: ambos binários com cor; XYZ binário/.bt: ambos binários, só geometria) e a razão "misturada" ASCII/.ot, que é a que a tese parece reportar.

**EuRoC (`euroc_confirm`, run00, leaf 0,05 m, voxel da nuvem 0,02 m).** Cada célula: faithful / imp.

| Seq | pontos (M) | .pcd bin. (MB) | .ot (MB) | .bt (MB) | razão bin/.ot (mesmo formato) | razão XYZ-bin/.bt | razão ASCII/.ot (misturada) | tese denso/octomap (razão) |
|---|---|---|---|---|---|---|---|---|
| MH01 | 11.3 / 10.9 | 181.0 / 174.6 | 26.0 / 25.6 | 1.52 / 1.50 | 6.97 / 6.82 | 89 / 87 | 18.3 / 17.9 | 38.2 / 5.5 (6.95) |
| MH02 | 9.9 / 9.7 | 158.6 / 154.9 | 25.2 / 24.4 | 1.56 / 1.50 | 6.28 / 6.36 | 76 / 77 | 16.5 / 16.7 | — |
| MH03 | 23.3 / 15.7 | 373.6 / 250.6 | 37.2 / 35.2 | 2.06 / 1.97 | 10.04 / 7.11 | 136 / 95 | 26.3 / 18.6 | 43.1 / 6.6 (6.53) |
| MH04 | 47.9 / 22.3 | 766.4 / 357.3 | 74.8 / 59.7 | 3.70 / 3.25 | 10.25 / 5.98 | 155 / 82 | 27.1 / 15.8 | — |
| MH05 | 46.8 / 23.1 | 749.5 / 369.5 | 71.9 / 60.9 | 3.63 / 3.31 | 10.42 / 6.06 | 155 / 84 | 27.6 / 16.0 | 71.1 / 11.5 (6.18) |
| V101 | 2.2 / 1.9 | 35.1 / 30.9 | 4.18 / 3.73 | 0.28 / 0.25 | 8.40 / 8.30 | 93 / 92 | 22.5 / 22.2 | 5.8 / 0.94 (6.17) |
| V102 | 3.2 / 3.1 | 50.7 / 49.0 | 5.79 / 5.67 | 0.39 / 0.38 | 8.76 / 8.64 | 98 / 97 | 23.3 / 23.0 | 11.1 / 1.7 (6.53) |
| V103 | 6.6 / 6.3 | 105.7 / 101.1 | 12.5 / 11.7 | 0.92 / 0.83 | 8.47 / 8.62 | 86 / 91 | 22.7 / 23.1 | 16.6 / 2.4 (6.92) |

Razão média (ACR da tese: média das razões nas 6 sequências em comum, tese 6,546): binário/.ot faithful 8.84, imp 7.59; ASCII/.ot faithful 23.4, imp 20.1; XYZ-bin/.bt faithful 110, imp 91.

Em EuRoC a razão não mede compressão de formato: nossa nuvem tem voxel de 0,02 m e o octomap 0,05 m, então a razão é dominada por essa diferença de resolução (6 a 10 mesmo em binário). A tese não publica o `Dense.resolution` de EuRoC; seus 38 MB em MH01 contra nossos 181 MB (binário) sugerem uma nuvem bem mais esparsa. Note a coincidência: a razão binária nossa em EuRoC (6,3 a 10,4; MH01 6,97 contra 6,95 da tese) é da mesma ordem da razão da tese (6,2 a 6,9), então em EuRoC os dados **não discriminam** entre "a tese comparou binário com binário a outra resolução" e "texto contra binário". A hipótese de texto contra binário só é sustentada onde voxel = leaf (TUM e Tabela VIII). As razões absolutas de EuRoC e KITTI não são comparáveis; só TUM é.

**TUM e KITTI (benchmark antigo de 39 h, run00, 10 execuções por célula, só a run00 tem nuvem).**

| Seq | pontos (M) f / i | .pcd bin. (MB) f / i | .ot (MB) f / i | .bt (MB) f / i | bin/.ot f / i | ASCII/.ot f / i | tese denso/octomap (razão) |
|---|---|---|---|---|---|---|---|
| tum_fr1_desk | 1.4 / 1.2 | 21.7 / 19.4 | 13.0 / 11.9 | 0.67 / 0.71 | 1.67 / 1.63 | 4.56 / 4.47 | 36.7 / 7.5 (4.89) |
| tum_fr1_room | 4.9 / 4.7 | 78.6 / 75.8 | 46.7 / 47.4 | 2.37 / 2.45 | 1.68 / 1.60 | 4.56 / 4.33 | 90.4 / 22.6 (4.00) |
| tum_fr2_desk | 2.2 / 2.4 | 35.6 / 39.1 | 18.7 / 19.3 | 0.92 / 0.94 | 1.90 / 2.03 | 5.17 / 5.51 | 67.7 / 17.2 (3.94) |
| tum_fr2_large_with_loop | 40.8 / 31.9 | 652.7 / 510.3 | 239.5 / 224.3 | 10.4 / 10.8 | 2.72 / 2.28 | 7.32 / 6.11 | 31.3 / 4.2 (7.45) [FR2_with_loop; correspondência incerta] |
| tum_fr3_office | 2.8 / 2.8 | 45.5 / 44.8 | 24.7 / 25.2 | 1.18 / 1.17 | 1.84 / 1.78 | 4.98 / 4.82 | 34 / 8.6 (3.95) |
| kitti_00 | 67.2 / 55.2 | 1075.8 / 883.4 | 119.0 / 108.3 | 7.44 / 6.82 | 9.04 / 8.16 | 23.74 / 21.44 | 83.7 / 16.1 (5.20) |
| kitti_03 | 9.6 / 9.4 | 152.9 / 149.7 | 22.4 / 22.4 | 1.42 / 1.42 | 6.82 / 6.67 | 17.77 / 17.40 | — |
| kitti_05 | 37.0 / 30.9 | 592.4 / 493.9 | 81.3 / 72.5 | 5.22 / 4.73 | 7.28 / 6.82 | 19.21 / 18.00 | — |
| kitti_07 | 7.8 / 7.7 | 124.3 / 123.4 | 23.6 / 23.4 | 1.52 / 1.51 | 5.26 / 5.27 | 14.06 / 14.11 | — |
| kitti_09 | 31.0 / 27.7 | 495.9 / 443.7 | 68.2 / 63.6 | 4.30 / 4.05 | 7.28 / 6.98 | 19.08 / 18.30 | — |

ACR TUM (5 sequências; a tese usa 6, inclui FR2_no_loop que não temos; tese 5,138): binário/.ot faithful 1.96, imp 1.86; **ASCII/.ot faithful 5.32, imp 5.05**; XYZ-bin/.bt faithful 30.8, imp 27.9. KITTI 00: binário/.ot 9.04 (faithful), ASCII/.ot 23.7; tese 5,2 (KITTI 00 isolada; ACR da tese 4,771 com 00, 01, 02). Mesma ressalva de resolução de EuRoC: nuvem a 0,1 m, octomap a 0,2 m.

Conclusões para a Tabela IX:

1. **Reproduzido em TUM, com a ressalva do formato.** A razão média ASCII/.ot (5,3 faithful, 5,1 imp) coincide com a da tese (5,14); a razão binário/.ot, que é a de arquivos no mesmo formato, é de apenas 2,0 (faithful) e 1,9 (imp). A "compactação de ~5 vezes" da tese é, em TUM, em grande parte codificação de arquivo (texto contra binário), como já apontado em `docs/Reproduction_Zhang2023.md`; em EuRoC e KITTI isso não se confirma nem se refuta (ver acima).
2. **Melhoria honesta e real:** o `.bt` (D2, só ocupação) tem cerca de 5% do tamanho do `.ot` colorido e 3% a 4% da nuvem XYZ binária em TUM (razão 28 a 31 vezes) e cerca de 1% em EuRoC/KITTI (razão 60 a 155 vezes). Não é comparável à Tabela IX da tese (perde a cor e o `.pcd` correspondente é só XYZ), então vale como achado e não como vitória numérica contra a tese.
3. **Tamanho da nuvem (a coluna "Dense" da tese):** o braço imp gera nuvens menores que o faithful quando o filtro de voxel global estoura o índice int32 do PCL (EuRoC MH03 a MH05: 251 / 357 / 370 MB contra 374 / 766 / 749 MB; KITTI 00: 883 contra 1076 MB), mas parte dessa redução vem de keyframes descartados pela fila limitada (seção 10.4, mediana de 142 de 357 em MH04), ou seja, o mapa imp cobre menos keyframes. Só a parte do filtro de voxel é correção determinista.
4. **O que falta:** `Dense.octomapWriteBt` e `compressionReport` nunca foram ligados nas rodadas; os `.bt` acima foram gerados offline a partir dos `.ot` salvos (`writeBinary` da árvore lida), que é equivalente à saída do código (que também faz `prune` antes). Nenhuma execução nova é necessária para esta tabela. Falta apenas ter FR2_no_loop (TUM), KITTI 01/02 e MH03 antigo para as médias idênticas à da tese, e o `Dense.resolution` de EuRoC/KITTI da tese.

### 10.4 Tabela X: tempos por etapa da reconstrução densa (ms por keyframe)

Fonte: bloco `Dense Reconstruction timing` do `slam.log` (média sobre os keyframes processados de cada execução; aqui a mediana entre as execuções). Contadores equivalentes aos da tese: depth acquisition, voxel filtering, map update, octomap conversion, total. EuRoC: `euroc_confirm` (12 execuções, 1000 features); TUM e KITTI: benchmark antigo (10 execuções; 1000 features em TUM e 2000 em KITTI, como na tese; execuções de TUM com I/O perturbado).

| Etapa (ms) | tese TUM fr3_office | nosso fr3_office faithful / imp | tese EuRoC V2_02 | nosso V102 faithful / imp (todas as 8: faith / imp) | tese KITTI 07 | nosso KITTI 07 faithful / imp |
|---|---|---|---|---|---|---|
| Depth acquisition | 0.02 | 6.9 / 6.7 | 1.57 | 207.5 / 222.3 (194 / 208) | 1.49 | 275.1 / 281.8 |
| Voxel filtering | 91.9 | 24.8 / 22.4 | 82.96 | 15.7 / 16.3 (16 / 16) | 99.68 | 15.5 / 15.8 |
| Map update | 85.35 | 4.6 / 4.6 | 89.6 | 4.9 / 4.9 (10 / 10) | 107.69 | 7.5 / 7.6 |
| Octomap conversion | 153.27 | 15.2 / 14.4 | 149.32 | 9.8 / 10.5 (18 / 20) | 182.21 | 9.7 / 9.9 |
| Total | 385.7 | 51.7 / 47.9 | 378.49 | 237.7 / 253.9 (237 / 255) | 447.11 | 307.8 / 315.7 |

Keyframes processados / descartados pela fila limitada (imp, mediana entre execuções): MH03 193 / 22, MH04 215 / 142, MH05 229 / 112, V103 212 / 25; KITTI 07 235 / 5; TUM fr3_office 284 / 0. O braço faithful processa todos (MH04 356, MH05 343).

Leitura honesta:

- **Total por keyframe:** menor que o da tese nos três casos (51,7 contra 385,7 ms em fr3_office; 238 contra 378 em EuRoC; 308 contra 447 em KITTI 07 no faithful). Isto não é um ganho nosso do algoritmo: o hardware é diferente (Ryzen 5 4600G limitado a 2 núcleos contra i7-7500U) e a composição das etapas é outra.
- **Octomap conversion** muito menor que a da tese (15 contra 153 ms em TUM; 19 contra 149 em EuRoC). O nosso `InsertIntoOctomap` insere só os pontos ocupados, sem traçado de raios (`Dense.octomapRayCast 0`, necessário para reproduzir os tamanhos da Tabela IX; com raios medimos 1979 ms por keyframe em fr1_desk, `docs/Reproduction_Zhang2023.md`). O valor da tese está entre os dois, então não sabemos o que ela fez.
- **Voxel filtering** e **map update**: nossos 25 e 5 ms em fr3_office contra 92 e 85 ms; aplicamos o filtro por keyframe no referencial da câmera; não há como separar hardware de implementação sem rodar o código da tese.
- **Depth acquisition em estéreo é pior que o da tese por duas ordens de grandeza**: 190 a 210 ms (EuRoC) e 275 ms (KITTI 07) contra 1,5 ms. Usamos `cv::StereoSGBM` na imagem retificada inteira (96 disparidades, bloco 9). A tese descreve disparidade calculada numa região de interesse; 1,5 ms para 752x480 não é plausível para SGBM denso em CPU de laptop, então a tese deve usar algo muito mais barato (ROI, bloco simples, ou cronometrar só a conversão disparidade para profundidade). Em RGB-D nosso valor é 6,9 ms contra 0,02 ms (a tese possivelmente cronometra só a leitura; nós incluímos a geração da nuvem a partir de RGB e profundidade).
- **imp contra faithful:** o imp tem `Depth acquisition` ~8% maior (208 contra 192 ms em MH01; compatível com a baixa prioridade cedendo CPU, não testado isoladamente) e processa menos keyframes (fila de 3): ele descarta em mediana 142 de 357 keyframes em MH04 e 112 de 341 em MH05. É o preço do desacoplamento: o mapa denso do imp tem lacunas.
- **Linhas do ORB-SLAM3 da tabela** (extração ORB, rastreamento, BA local, laço, BA completo): **não disponíveis**. Exigem compilar com `REGISTER_TIMES`; os dados do ORB-SLAM3 puro na tese vêm de [19] em V2_02 e de três execuções do autor nas demais. Falta uma build separada (ainda por decidir se vale o custo) e rodar uma sequência de cada dataset (cerca de 3 min cada).
- **V2_02 não foi rodada.** Usamos V102 como a sequência EuRoC mais próxima, mas os tempos dependem do conteúdo; uma execução de V2_02 exige baixar/usar V2_02 (não está no nosso conjunto de 8 sequências).

### 10.5 FPS

**O que a tese afirma.** Não há tabela de FPS. São afirmações em texto: (a) TUM, "estamos perto de reconstrução densa em tempo real" com entrada de 30 Hz, e, entre FR1_room e FR1_desk, a cena menor tem frame rate menor, atribuído ao hardware (discussão dos resultados de TUM); (b) EuRoC: ao reduzir de 2000 para 1000 features, "o frame rate sobe de menos de 10 Hz para cerca de 15 Hz" (discussão dos resultados de EuRoC); (c) seção 4.5 (Runtime Analysis): com a reconstrução densa "cerca de 25 Hz em ambientes pequenos, caindo conforme a sequência cresce, até 10 Hz em KITTI", contra 30 a 40 FPS do ORB-SLAM3 original, que cita de [19]. **A tese não diz como mede**: nem se é a taxa de quadros rastreados por segundo de relógio, nem o inverso do tempo de rastreamento, nem se inclui a thread densa.

**O que os nossos logs respondem.** O harness (`stereo_euroc`, `rgbd_tum`, `stereo_kitti`) segue o ritmo do dataset (`usleep` quando o quadro termina antes do período), então o tempo de parede é a duração da sequência mais um custo fixo, e **o FPS real não foi medido** em nenhum braço. O que existe:

| Indicador | Resultado | Responde ao FPS da tese? |
|---|---|---|
| Tempo médio de rastreamento, baseline, TUM (mediana de 10) | fr3_office 17,1 ms (~58 Hz), fr1_desk 16,3, fr1_room 15,1, fr2_desk 20,6, fr2_large 14,0 | Só do baseline, e só do rastreamento; limite superior de FPS, sem a thread densa |
| Idem, baseline, KITTI | 07: 36,3 ms (~28 Hz), 00: 33,8, 03: 34,3, 05: 36,1, 09: 36,2 | Idem; entrada de KITTI é 10 Hz |
| Idem, braços densos (TUM e KITTI) | impresso com 2 casas decimais (fixed vaza do `PrintTimingSummary`): 0,02 / 0,03 s, resolução de 10 ms | **Não utilizável** para comparar com o baseline nem com a tese (`tracking_time_low_precision`) |
| Tempo de rastreamento em EuRoC | `stereo_euroc` preenche `vTimesTrack` mas não imprime | **Não existe** |
| Tempo de parede menos o do baseline (EuRoC, mediana de 12) | imp: +0 a +6 s em todas; faithful: MH04 +30 s, MH05 +15 s, demais ~0 (duração das sequências 78 a 185 s; custo fixo de carga ~15 a 26 s) | Indireto: hipótese não verificada de que a thread densa do faithful, com ~100 s de trabalho (356 keyframes x ~280 ms) numa sequência de 101 s, termina depois do último quadro; o imp acompanha, ao custo de descartar keyframes |

**Veredito de FPS.** Não dá para afirmar nem contestar "25 Hz / 15 Hz / 10 Hz" da tese com os dados atuais. O único número honesto é que, no baseline, o rastreamento sozinho comporta ~58 Hz em TUM e ~28 Hz em KITTI neste hardware, e que o braço faithful termina 15 a 30 s depois em EuRoC grande (MH04/MH05) enquanto o imp não (causa provável: backlog da thread densa; não verificada). **O que falta:** uma execução sem ritmo do dataset (remover o `usleep` do exemplo ou um flag) com `std::defaultfloat` no tempo de rastreamento, com o tempo de parede por quadro processado, em baseline, faithful e imp numa sequência por dataset (EuRoC MH04, TUM fr3_office, KITTI 07), cerca de 2 a 5 min por execução, e definir FPS como quadros / tempo de parede sem pacing.

### 10.6 Resumo do que falta

1. Tabela VII (fusão multi-sequência): nenhuma execução.
2. Tabela X, linhas do ORB-SLAM3 puro: build `REGISTER_TIMES` e V2_02.
3. FPS real: execução sem pacing e tempo de rastreamento com precisão total no braço denso.
4. Tabelas VIII/IX: nada de execução é necessário; para fechar a comparação exata faltam FR2_no_loop, KITTI 01/02, o `Dense.resolution` de EuRoC/KITTI da tese e, opcionalmente, 5 execuções reais com `Dense.resolution` = leaf para substituir a série derivada da Tabela VIII (cerca de 5 execuções curtas de fr1_room, abaixo de 5 min cada).
