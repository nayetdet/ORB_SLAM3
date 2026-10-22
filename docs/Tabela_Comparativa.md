# Tabela comparativa dos resultados

Resumo em tabelas do `Relatorio_Final.md`. Os números vêm de lá.

## As versões comparadas

| Nome | O que é |
|---|---|
| **Tese** | Números publicados na dissertação de Zhang (2023), medidos em outro notebook |
| **Baseline** | ORB-SLAM3 puro, sem reconstrução densa |
| **Só ajuste final (c1)** | Baseline mais um ajuste global do mapa no fim, ainda sem parte densa |
| **Denso fiel** | Reconstrução densa como descrita na tese, rodando na nossa máquina |
| **Denso melhorado** | Nossa versão: denso com fila limitada, baixa prioridade, ajuste final e filtro de voxel corrigido |

## Como ler

- **ATE** é o erro médio da trajetória estimada em relação à real. **Quanto menor, melhor.**
- A variação % compara o **denso melhorado com o baseline**.
- Cada número é a mediana de 12 execuções (4 no KITTI). Todas as execuções usaram 2 núcleos de CPU, como no notebook da tese.

| Símbolo | Significado |
|---|---|
| ✅ | Melhorou, e a estatística confirma com folga (passa na correção de Holm) |
| 🟢 | Provavelmente melhorou (p < 0,05), mas sem prova forte |
| ➖ | Igual: a diferença cabe no ruído entre execuções |
| ⚠️ | Não piorou de forma significativa, mas também não dá para garantir |
| 🔴 | Piorou |

---

## 1. Visão geral

| Métrica | Denso melhorado vs baseline | Resultado |
|---|---|---|
| Precisão no EuRoC (drone) | até −37% de erro; nenhuma sequência piorou de forma significativa | ✅ Melhorou |
| Precisão no TUM (câmera na mão) | igual; um sinal de melhora em fr3_office (−11%) | ➖ Igual |
| Precisão no KITTI (carro) | ±5% por sequência, sem tendência | ➖ Igual |
| Tamanho da nuvem de pontos | −33% a −53% nas cenas grandes | ✅ Melhorou |
| Velocidade (FPS) | ~7% mais lento que o baseline (o denso fiel perde ~8%) | 🔴 Não melhorou (o custo vem da parte densa) |
| Mapas encadeados (TUM) | erro ~2× maior que os outros, mas sem significância | ⚠️ Atenção |

**De onde vem o ganho de precisão:** do **ajuste final**. A versão c1 (só o ajuste, sem denso) ganha quase o mesmo que o denso melhorado. A ideia original, de que a thread densa roubava CPU do SLAM, **não se confirmou**.

---

## 2. Precisão no EuRoC (ATE em **centímetros**)

O EuRoC são gravações de um **drone** voando em ambientes fechados, com uma câmera estéreo. Os nomes das sequências indicam o lugar e a dificuldade:

- **MH** (*Machine Hall*): um galpão industrial grande, com máquinas e estruturas metálicas. Vai de MH01 a MH05.
- **V1** (*Vicon Room 1*): uma sala menor, equipada com câmeras que medem a posição real do drone. Vai de V101 a V103.
- **O número final** é a dificuldade: quanto maior, mais rápido e brusco o voo, e às vezes mais escuro. Por isso o erro sobe em MH04, MH05 e V103.

| Sequência | Onde / dificuldade | Tese | Baseline | Só ajuste final (c1) | Denso fiel | **Denso melhorado** | Variação | Resultado |
|---|---|---|---|---|---|---|---|---|
| MH01 | Galpão, fácil | 3,9 | 4,10 | 3,34 | 3,43 | **3,12** | −24% | 🟢 |
| MH02 | Galpão, fácil | 2,8 | 2,43 | 1,63 | 2,48 | **1,53** | −37% | ✅ |
| MH03 | Galpão, médio | 3,2 | 3,53 | 3,12 | 3,09 | **3,33** | −6% | ➖ |
| MH04 | Galpão, difícil | 11,2 | 7,25 | 5,99 | 7,36 | **5,32** | −27% | 🟢 |
| MH05 | Galpão, difícil | 6,1 | 6,26 | 5,77 | 5,42 | **5,54** | −12% | ➖ |
| V101 | Sala, fácil | 4,2 | 3,65 | 3,71 | 3,63 | **3,63** | 0% | ➖ |
| V102 | Sala, médio | 3,6 | 4,36 | 3,13 | 3,27 | **3,01** | −31% | 🟢 |
| V103 | Sala, difícil | 10,7 | 10,46 | 11,40 | 9,15 | **11,23** | +7% | ⚠️ |

> A coluna "Tese" é só uma referência: foi medida em outro computador, e a tese não diz exatamente como calculou o erro.

---

## 3. Precisão no TUM (ATE em **centímetros**)

| Sequência | Tese | Baseline | Denso fiel | **Denso melhorado** | Variação | Resultado |
|---|---|---|---|---|---|---|
| fr1_desk | 1,7 | 1,75 | 1,73 | **1,77** | +1% | ➖ |
| fr1_room | 5,2 | 7,34 | 7,28 | **7,09** | −3% | ➖ |
| fr2_desk | 1,8 | 1,84 | 1,88 | **1,85** | +1% | ➖ |
| fr2_large_no_loop | 23,7 | 52,2 | 54,3 | **51,5** | −1% | ➖ |
| fr3_office | — | 1,08 | 1,01 | **0,97** | −11% | 🟢 |

---

## 4. Precisão no KITTI (ATE em **metros**)

O carro percorre quilômetros, por isso aqui o erro está em metros.

| Sequência | Tese | Baseline | Denso fiel | **Denso melhorado** | Variação | Resultado |
|---|---|---|---|---|---|---|
| 00 | 1,25 | 1,20 | 1,20 | **1,15** | −5% | ➖ |
| 01 | 5,60 | 14,61 | 14,22 | **13,67** | −6% | ➖ |
| 02 | 1,92 | 5,16 | 5,39 | **4,94** | −4% | ➖ |
| 03 | 0,64 | 1,35 | 1,33 | **1,40** | +3% | ➖ |
| 04 | 0,23 | 0,23 | 0,23 | **0,25** | +8% | ➖ |
| 05 | 0,56 | 0,93 | 0,94 | **0,95** | +2% | ➖ |
| 06 | 1,35 | 0,97 | 0,97 | **0,94** | −3% | ➖ |
| 07 | 0,46 | 0,46 | 0,46 | **0,45** | −1% | ➖ |
| 08 | 2,65 | 3,58 | 3,58 | **3,58** | 0% | ➖ |
| 09 | 3,86 | 1,85 | 1,88 | **1,91** | +3% | ➖ |
| 10 | 1,25 | 1,23 | 1,28 | **1,27** | +3% | ➖ |

Contagem: o denso melhorado ficou melhor em 5 sequências e pior em 6, um empate. O denso fiel ficou pior em 9 de 11, mas isso também não chega a ser significativo.

---

## 5. Tamanho da nuvem de pontos no EuRoC (MB)

| Sequência | Denso fiel | **Denso melhorado** | Redução |
|---|---|---|---|
| MH01 | 181 | **175** | −3% |
| MH02 | 159 | **155** | −3% |
| MH03 | 374 | **251** | −33% |
| MH04 | 766 | **357** | −53% |
| MH05 | 749 | **370** | −51% |
| V101 | 35 | **31** | −12% |
| V102 | 51 | **49** | −4% |
| V103 | 106 | **101** | −5% |

A redução grande nas cenas grandes vem do **filtro de voxel corrigido**. Na versão da tese, o filtro falhava nessas cenas e devolvia a nuvem sem filtrar. Parte da redução também vem de keyframes que a fila limitada descarta, então o mapa melhorado tem algumas lacunas.

Também passamos a gravar um arquivo `.bt` do Octomap, só com ocupação e sem cor, com cerca de **5% do tamanho** do `.ot` colorido.

---

## 6. Velocidade (quadros por segundo, sem pausas)

Quanto maior, melhor.

| Dataset | Tese | Baseline | Denso fiel | **Denso melhorado** |
|---|---|---|---|---|
| TUM fr1_desk | 17,7 a 24,2 | 43,1 | 42,7 | **43,2** |
| EuRoC MH01 | ~15 | 39,4 | 36,2 | **36,8** |
| KITTI 07 | até 10 | 29,0 | 24,0 | **25,0** |

O denso deixa o sistema mais lento que o baseline, e o melhorado perde um pouco menos que o fiel. Os números da tese são mais baixos porque o notebook dela é mais fraco, então eles não servem para comparar diretamente.

---

## 7. Mapas encadeados no TUM (Tabela VII da tese)

O teste roda `fr2_large_with_loop` e depois `fr2_large_no_loop` no mesmo mapa. A tabela mostra o erro médio da segunda sequência, em metros.

| Versão | Sozinha | Encadeada | Queda |
|---|---|---|---|
| Tese | 0,706 | 0,181 | −74% |
| Baseline | 0,523 | 0,072 | −86% |
| Denso fiel | 0,513 | 0,064 | −88% |
| **Denso melhorado** | 0,462 | **0,136** | −71% ⚠️ |

O ganho de encadear aparece até no baseline, porque vem da fusão de mapas do próprio ORB-SLAM3. Na cadeia, o denso melhorado ficou cerca de 2× pior que os outros (p = 0,23, não significativo). É o único ponto em que ele aparece atrás do baseline e precisaria de mais execuções. A cadeia do EuRoC ficou incompleta (144 de 288 execuções).

---

## Em uma frase

**O denso melhorado reduziu o erro no EuRoC (principalmente na MH02, com −37%) e encolheu as nuvens grandes pela metade, sem piorar a precisão de forma significativa em nenhum dataset. Em troca, ficou ~7% mais lento que o baseline, e o ganho de precisão vem do ajuste final, não da thread densa.**
