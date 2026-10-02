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
