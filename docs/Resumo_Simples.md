# Resumo simples: melhorias e resultados

Versão curta do `Relatorio_Final.md`. Se quiser os números completos, é lá.

## O ponto de partida

A dissertação de Zhang (2023) adiciona uma **thread de reconstrução densa** ao ORB-SLAM3. Nos resultados dela, a trajetória fica um pouco **pior** que a do ORB-SLAM3 puro no EuRoC.

A nossa hipótese era que a thread densa disputa CPU com o SLAM e atrapalha a otimização do mapa. Então tentamos melhorar o código sem mudar o dataset, a métrica nem o hardware.

## O que foi mudado

| Melhoria | O que faz | Para que serve |
|---|---|---|
| **Fila limitada** | A fila entre o rastreamento e a thread densa passa a ter no máximo 3 itens. Quando enche, descarta os mais antigos. | Evita a thread densa acumular trabalho atrasado. |
| **Prioridade baixa** | A thread densa só usa CPU quando ninguém mais precisa. | Reduz a disputa com o SLAM. |
| **Ajuste global final (C1)** | Uma passada de bundle adjustment no mapa inteiro antes de salvar a trajetória. | Corrige poses de quadros-chave depois do último loop. |
| **Filtro de voxel corrigido** | O filtro da tese estourava um índice inteiro do PCL em cenas grandes e devolvia a nuvem sem filtrar. | Nuvens menores, sem perder o que importa. |
| **Shutdown sincronizado** | O baseline passa a esperar as threads terminarem antes de salvar. | Comparação justa: sem isso o baseline perdia o fim da sequência. |
| **Arquivo `.bt` do Octomap** | Passa a gravar também a versão só de ocupação. | Arquivo bem menor (cerca de 5% do `.ot` colorido). |

Tudo pode ser desligado por configuração. O modo "fiel à tese" continua disponível para comparar lado a lado.

## Como foi testado

- Quatro versões comparadas: **base** (ORB-SLAM3 puro), **c1** (base mais ajuste final), **faithful** (denso como na tese) e **imp** (denso melhorado).
- Execuções entrelaçadas, para a máquina não favorecer nenhuma versão.
- 2 núcleos de CPU, como o notebook da tese.
- 12 execuções por sequência no EuRoC e no TUM; 4 no KITTI.
- Teste de permutação com correção de Holm para comparações múltiplas.
- Os critérios foram escritos **antes** de rodar (arquivos `PRE_REGISTRATION_*`).

## Resultados

**Precisão (ATE) no EuRoC**

- A melhora é real em algumas sequências. A mais forte é a **MH02, com −37%** (0,0243 m → 0,0153 m), que passa na correção de Holm.
- MH01 (−24%), MH04 (−27%) e V102 (−31%) melhoram com p bruto < 0,05, mas sem passar no Holm. É sinal, não prova.
- As outras quatro ficam iguais dentro do ruído.
- Nenhuma sequência piorou de forma significativa. A V103 ficou +7% (não significativo), mas o intervalo de confiança não descarta uma piora maior, então não dá para garantir que não piora.

**De onde vem o ganho**

O ganho de precisão vem do **ajuste global final**, e não do desacoplamento da thread densa. A versão `c1` (sem nada denso) e a `imp` não diferem de forma significativa. A hipótese original, de que disputar CPU era o problema, **não se confirmou** nos nossos testes. O denso fiel à tese também não ficou pior que o baseline.

**Tamanho do mapa**

- Nuvens **33% a 53% menores** nas cenas grandes (MH03, MH04, MH05).
- Isso é determinista, por causa do filtro de voxel corrigido. Parte da redução também vem de keyframes que a fila limitada descarta.

**TUM**

Neutro, com um sinal em fr3_office (−10,6%, não passa no Holm).

**KITTI**

Sem ganho e sem piora significativa (±5% por sequência). O denso fiel tende a piorar um pouco (2 melhores contra 9 piores, não significativo).

**FPS (sem pausas)**

O denso **reduz** o FPS em cerca de 7% no EuRoC MH01 em relação ao baseline. O melhorado perde um pouco menos que o fiel. Isso é custo inerente da thread densa, então nesta métrica não melhoramos.

## O que ficou incompleto ou com ressalva

- **Tabela VII (mapas encadeados):** o EuRoC ficou parcial (144 de 288 execuções, por limite de tempo). No TUM, o denso melhorado ficou cerca de 2 vezes pior que os outros na cadeia. Não é significativo (p = 0,23), mas merece mais execuções.
- **Comparação numérica com a tese:** é indicativa. A tese não diz qual alinhamento de trajetória usou, e o hardware é outro (Ryzen 5 4600G limitado a 2 núcleos, contra um i7-7500U).
- **Compressão da Tabela IX:** a razão de ~5× da tese parece comparar `.pcd` em texto com `.ot` binário. Com os dois lados em binário, a razão em TUM cai para cerca de 2×.
- **Tabela X:** só a parte densa foi medida. As linhas do ORB-SLAM3 puro precisariam de uma build com `REGISTER_TIMES`.
- **Profundidade estéreo:** nossa aquisição leva 190 a 275 ms por keyframe, contra 1,5 ms na tese. A tese deve usar algo bem mais barato que SGBM.

## Conclusão em uma frase

Melhoramos a precisão onde havia margem (EuRoC, com ganho mais claro em MH02) e reduzimos o tamanho dos mapas, sem piora significativa de precisão em nenhum dataset. O ganho vem do ajuste global final, e não da thread densa. FPS e KITTI não melhoraram.
