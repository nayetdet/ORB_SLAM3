# Melhorias propostas — como superar os resultados de Zhang (2023) só mexendo no código

> **Regra deste documento:** só entram mudanças que dependem exclusivamente de alterar
> código ou configuração deste repositório. Nada que exija hardware diferente, dataset
> diferente ou mudança no protocolo de medição.
>
> **Pré-requisito:** as etapas de validação do `docs/Validacao_Pendente.md` precisam
> estar concluídas antes de aplicar qualquer coisa daqui. Sem a linha de base medida na
> mesma máquina, nenhuma dessas mudanças é demonstrável.

---

## O diagnóstico em que tudo isto se apoia

A dissertação reporta erro de trajetória **pior** que o ORB-SLAM3 puro nas 8 sequências
do EuRoC (Tabela V) e empate no KITTI (Tabela VI). O autor atribui a perda à redução do
número de pontos de característica de 2000 para 1000, que ele fez para caber em tempo
real (Seção 4.2), mas **nunca mede a thread densa isoladamente**.

O mecanismo real é disputa de CPU, e ele é rastreável no código:

1. `LocalMapping::InsertKeyFrame` (`src/LocalMapping.cc:288`) marca `mbAbortBA = true` a
   cada quadro-chave novo.
2. Esse sinalizador chega ao otimizador em `Optimizer::LocalBundleAdjustment`
   (`src/Optimizer.cc:1204`, via `setForceStopFlag`).
3. Resultado: **o ajuste de feixes local é interrompido** antes de convergir sempre que
   a fila de quadros-chave acumula.

A thread densa, ao consumir CPU, faz o `LocalMapping` demorar mais para esvaziar a fila,
o que multiplica esses abortos. O mapa esparso fica menos otimizado e o erro sobe.

**Medido aqui** (TUM fr1_desk, mesma máquina, mesmo código, 1 execução cada): com a
reconstrução densa ligada o erro piorou em relação à base. O tamanho do efeito ainda
precisa de 3 a 10 execuções, mas a direção se repetiu nos dois pares medidos.

**A consequência para o trabalho:** a perda de precisão da metodologia dele é um
artefato de **acoplamento**, não um custo inerente da reconstrução densa. As mudanças do
Grupo A partem daí.

---

## Grupo A — Eliminar o custo em precisão (a contribuição principal)

Estas são as mudanças que atacam o diagnóstico acima. São as de maior impacto e as que
sustentam o argumento central do trabalho.

### A1. Limitar a fila entre o Tracking e a thread densa

| | |
|---|---|
| **Onde** | `src/PointCloudMapping.cc:210` (`std::list<QueueItem> mlQueue`), `:689` e `:705` (os dois `push_back`) |
| **Hoje** | A fila é **ilimitada**. Se a thread densa não acompanha, a fila cresce indefinidamente e ela segue consumindo CPU processando quadros-chave atrasados |
| **Mudança** | Capar a fila (2 a 4 itens). Cheia, **descarta** o item mais antigo em vez de bloquear ou acumular |
| **Por que funciona** | A thread densa passa a trabalhar só com o que consegue acompanhar. O `LocalMapping` esvazia a fila de quadros-chave mais rápido, o BA local é abortado menos vezes |
| **Custo** | O mapa denso fica temporalmente mais esparso — alguns quadros-chave não entram. Como quadros-chave vizinhos se sobrepõem muito, a perda de cobertura espacial é pequena |
| **Esforço** | Baixo — ~20 linhas |
| **Risco** | Baixo. É reversível por configuração (`Dense.queueLimit: 0` = comportamento atual, fiel à tese) |
| **Como medir** | Comparar `tum_rgbd` × `tum_rgbd_dense` antes e depois, 5 a 10 execuções. Rodar `validate_dense.py` para confirmar que a nuvem não degradou |

**Importante para a narrativa:** mantenha o comportamento antigo acessível por
configuração. O trabalho precisa mostrar *os dois* — com fila ilimitada (fiel à tese) e
com fila limitada (sua melhoria) — medidos lado a lado.

### A2. Baixar a prioridade da thread densa

| | |
|---|---|
| **Onde** | Construtor de `PointCloudMapping::Impl`, logo após `mThread = std::thread(...)` |
| **Mudança** | `pthread_setschedparam` com política `SCHED_IDLE`, ou `nice(+10)` dentro da thread |
| **Por que funciona** | Ataca o mesmo problema do A1 por outro lado: o escalonador do sistema passa a só dar CPU à reconstrução densa quando ninguém mais precisa |
| **Esforço** | Muito baixo — ~5 linhas |
| **Risco** | Baixo. Em máquina com folga de núcleos o efeito é pequeno; ele aparece justamente no cenário apertado, que é o da dissertação |
| **Observação** | Complementa o A1, não substitui. Aplique os dois e meça separadamente para saber o que cada um rende |

### A3. Modo de reconstrução offline

| | |
|---|---|
| **Onde** | Novo modo em `PointCloudMapping`, acionado em `System::Shutdown` (`src/System.cc:545`) |
| **Mudança** | `Dense.mode: offline` — durante a execução, só guarda `(ponteiro do quadro-chave, imagens)`; a nuvem inteira é construída **depois** que a sequência termina |
| **Por que funciona** | Zero disputa de CPU durante o SLAM. O erro de trajetória passa a ser **idêntico** ao ORB-SLAM3 puro, por construção |
| **Ganho extra** | A nuvem é montada com as poses **já otimizadas**, depois do ajuste global. Fica mais precisa que a da dissertação — ver B1 |
| **Custo** | Memória: guardar as imagens de todos os quadros-chave. Em fr1_desk foram 105 quadros-chave; em KITTI 00 serão milhares. Mitigação: gravar as imagens em disco temporário em vez de RAM |
| **Esforço** | Médio — ~80 linhas |
| **Risco** | Memória em sequências longas. Comece limitando ao TUM e ao EuRoC |

**Este é o argumento mais forte do trabalho:** demonstra que a reconstrução densa custa
**zero** em precisão, contra os 17% a 75% de piora da dissertação. O preço é perder o
"tempo real", que era exatamente o que ele estava tentando preservar ao cortar features.

### A4. Restaurar o número de pontos de característica

| | |
|---|---|
| **Onde** | `ORBextractor.nFeatures` nos `Examples/*/*_Dense.yaml` (configuração, não código) |
| **Hoje** | A dissertação corta de 2000 para 1000 no EuRoC para comprar desempenho (Seção 4.2). Nossos `*_Dense.yaml` herdam o valor do arquivo original (1200 no EuRoC, 2000 no KITTI, 1000 no TUM) |
| **Mudança** | Depois de A1 ou A3, não há mais motivo para cortar. Garanta que o valor é o mesmo do baseline, e documente |
| **Por que funciona** | Mais pontos = mais restrições no ajuste de feixes = menos deriva |
| **Esforço** | Trivial |
| **Cuidado** | Baseline e denso **têm** que usar o mesmo valor, senão a comparação não isola a thread densa |

### A5. Tirar o Octomap do caminho crítico

| | |
|---|---|
| **Onde** | `src/PointCloudMapping.cc:637` — `InsertIntoOctomap` roda por quadro-chave dentro da thread densa |
| **Mudança** | Construir o Octomap **uma vez só**, a partir da nuvem final, em `Save()` |
| **Por que funciona** | Hoje o mesmo espaço é reinserido a cada quadro-chave. Medido aqui: 16,7 ms por quadro-chave, contra 153,27 ms na Tabela X da dissertação. Fazendo uma vez só, o custo sai do laço |
| **Ganho extra** | O Octomap passa a ser construído com as poses otimizadas, igual a B1 |
| **Perda** | Perde a atualização incremental de ocupação. Se você quiser espaço livre esculpido (ver D1), essa atualização importa; se só quer o mapa final, não |
| **Esforço** | Baixo — ~30 linhas |

---

## Grupo B — Melhorar o próprio mapa denso

Aqui o alvo não é o erro de trajetória, é a qualidade da reconstrução. São melhorias
sobre defeitos reais do método dele.

### B1. Reprojetar a nuvem com as poses otimizadas

| | |
|---|---|
| **Onde** | `src/PointCloudMapping.cc:624` — `const Sophus::SE3f Twc = item.pKF->GetPoseInverse()` |
| **Hoje** | Cada nuvem de quadro-chave é transformada para o mundo **no momento em que é processada**, e os pontos ficam congelados ali. É o que o Algoritmo 1 da dissertação descreve, na linha 21 |
| **O defeito** | Quando um fechamento de loop dispara o ajuste global, **todas as poses dos quadros-chave mudam** — mas os pontos densos já inseridos não acompanham. O mapa denso fica inconsistente com o mapa esparso corrigido. Quanto maior o loop, pior a costura |
| **Mudança** | Guardar as nuvens **no referencial da câmera**, junto com o ponteiro do quadro-chave. Transformar para o mundo só na hora de salvar, lendo a pose final |
| **Por que funciona** | A nuvem herda toda a otimização que o backend fez, inclusive o ajuste global pós-loop |
| **Custo** | Memória: guardar as nuvens por quadro-chave em vez de uma nuvem global acumulada. Depois do filtro de voxel isso é aceitável |
| **Esforço** | Médio — ~60 linhas. Sai de graça se A3 for feito |
| **Como medir** | Sequência com fechamento de loop (fr2_large_with_loop, EuRoC MH_05, KITTI 00). Compare a nuvem antes e depois visualmente e pelo `validate_dense.py` |

**Este é um defeito da metodologia da tese, não da nossa implementação.** Vale destacar:
ela reconstrói com poses provisórias e nunca corrige.

### B2. Levar a ideia da equação 34 para onde ela afeta a trajetória

| | |
|---|---|
| **Onde** | `src/Optimizer.cc:909` e `:1345` — matriz de informação das arestas estéreo |
| **Hoje** | `Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2`, onde `invSigma2` vem **só** do nível da pirâmide ORB (`mvInvLevelSigma2[octave]`). **Nada sobre profundidade** |
| **A observação da tese** | A equação 34 dela diz exatamente a coisa certa: um ponto medido longe é menos confiável que um perto, porque o erro de triangulação cresce com a distância |
| **O problema** | Ela aplica isso na **nuvem densa**, que não alimenta o rastreamento. Por construção, não pode mexer no erro de trajetória |
| **Mudança** | Aplicar a mesma ideia na matriz de informação do ajuste de feixes, com a forma fisicamente correta: `σ_z ≈ z²·σ_d/(f·b)`. Ou seja, desinflar o peso das observações distantes |
| **Por que funciona** | O ajuste passa a confiar mais nas medidas boas. É especialmente relevante no KITTI, onde quase tudo é distante |
| **Esforço** | Médio — poucas linhas, mas exige cuidado e medição |
| **Risco** | Mexe no núcleo do otimizador. Meça em todas as sequências antes de afirmar qualquer coisa |

**Narrativa:** *"a observação dele estava certa, mas aplicada na estrutura errada; movida
para o ajuste de feixes, ela passa a afetar a métrica que ele queria melhorar."* Este é o
item que mais se parece com uma contribuição científica, e não só de engenharia.

### B3. Remover pontos espúrios da nuvem

| | |
|---|---|
| **Onde** | `PointCloudMapping::Impl::Run`, depois do filtro de voxel |
| **Mudança** | `pcl::StatisticalOutlierRemoval` (disponível no container). Para o estéreo, somar verificação de consistência esquerda-direita da disparidade |
| **Por que funciona** | A disparidade produz pontos falsos em bordas e regiões sem textura. A dissertação só corta por faixa de profundidade e pela região de interesse |
| **Esforço** | Baixo — ~15 linhas |
| **Ganho** | Mapa visivelmente mais limpo; ajuda também na compactação, já que ruído não comprime bem |

### B4. Filtrar a disparidade com WLS

| | |
|---|---|
| **Onde** | `Impl::GenerateCloudStereo`, após `mpSGBM->compute` |
| **Mudança** | `cv::ximgproc::createDisparityWLSFilter` — **verificado: disponível no container** (`libopencv_ximgproc.so`) |
| **Por que funciona** | Preenche buracos e alinha as bordas da disparidade com as bordas da imagem. É o padrão atual para estéreo denso; a dissertação usa SGBM cru |
| **Custo** | Mais tempo por quadro-chave. Combine com A1/A3 |
| **Esforço** | Baixo — ~20 linhas, mas exige adicionar `opencv_ximgproc` ao `CMakeLists.txt` |

---

## Grupo C — Ficar abaixo da própria linha de base

Os grupos A e B recuperam a precisão do ORB-SLAM3 puro. Estas mudanças tentam ir além
dele — mais ambicioso e mais arriscado.

### C1. Ajuste global final antes de salvar a trajetória

| | |
|---|---|
| **Onde** | `System::Shutdown` (`src/System.cc:545`), antes de salvar. `Optimizer::GlobalBundleAdjustemnt` já existe em `src/Optimizer.cc:52` |
| **Hoje** | O ORB-SLAM3 só roda ajuste global quando fecha um loop. Se o loop fecha no meio, os quadros-chave seguintes não recebem essa otimização |
| **Mudança** | Uma passada de ajuste global no fim da sequência, antes de escrever a trajetória |
| **Por que funciona** | Otimização offline, sem restrição de tempo real, sobre o mapa inteiro |
| **Custo** | Segundos a minutos no fim da execução. **Zero** em tempo real |
| **Esforço** | Baixo — ~10 linhas |
| **Risco** | Baixo, e é a melhoria mais provável de render ganho consistente em todas as sequências |
| **Cuidado honesto** | Isso melhora **baseline e denso igualmente**. Serve para ficar abaixo do número dele, mas **não** para demonstrar que a thread densa ficou barata. Reporte as duas comparações separadas |

### C2. Pré-processamento para sequências escuras

| | |
|---|---|
| **Onde** | `Tracking::GrabImageStereo` e `GrabImageRGBD`, antes da extração ORB |
| **Mudança** | CLAHE (equalização de histograma adaptativa) opcional por configuração |
| **Alvo** | MH_04 e MH_05 do EuRoC, que são escuras e onde a dissertação tem seus piores números (0,112 e 0,061) |
| **Por que funciona** | Mais pontos detectados em região escura = melhor rastreamento |
| **Esforço** | Baixo — ~15 linhas |
| **Risco** | Pode piorar em sequências bem iluminadas. Deixe desligado por padrão e meça sequência a sequência |

### C3. Modo estéreo-inercial no EuRoC

| | |
|---|---|
| **Onde** | Configuração, não código: usar `Examples/Stereo-Inertial/stereo_inertial_euroc` com o `EuRoC.yaml` correspondente, que já traz a calibração da IMU |
| **Por que funciona** | A dissertação ignora completamente a IMU do EuRoC. O modo inercial é muito mais robusto justamente onde o estéreo puro quebra — V1_03 e MH_04, movimento agressivo e pouca luz |
| **Esforço** | Zero de código |
| **Cuidado** | É uma comparação **diferente** — sensor diferente. Não pode entrar na mesma tabela que o estéreo puro sem dizer isso. Reporte como coluna separada |
| **Atenção técnica** | O ground truth muda de referencial: no modo inercial a trajetória salva é do **corpo/IMU**, não da câmera. O harness já trata isso (config `euroc_stereo_inertial`) |

---

## Grupo D — Melhorar as métricas de mapa

### D1. Octomap com espaço livre, a custo aceitável

| | |
|---|---|
| **Onde** | `Impl::InsertIntoOctomap`, chave `Dense.octomapRayCast` |
| **Situação** | Hoje está **desligado**, porque foi a única forma de reproduzir os tamanhos da Tabela IX dela. Ligado, o `.ot` ficou **maior** que a nuvem (39 MB contra 18) e custou 1979 ms por quadro-chave contra 16,7 ms |
| **O ponto** | Sem esculpir espaço livre, o mapa só distingue "ocupado" de "desconhecido". Ele **não serve** para a navegação que a Seção 3.4 dela usa como motivação. Isso vale para o mapa dela também |
| **Mudança** | Ligar o traçado de raios, mas com `maxrange` limitado e `discretize = true` no `insertPointCloud`, e o Octomap construído uma vez só (A5) |
| **Por que funciona** | `discretize` agrupa raios que caem no mesmo voxel, cortando drasticamente o trabalho repetido; `maxrange` evita esculpir corredores enormes no KITTI |
| **Esforço** | Baixo — ~10 linhas |
| **Ganho para o trabalho** | Entrega a capacidade que a dissertação promete mas não cumpre, a um custo que ela não conseguiu |

### D2. Reportar compactação com formatos comparáveis

| | |
|---|---|
| **Situação** | A Tabela IX dela reporta ~5× de redução, mas comparando arquivos de formatos diferentes. Medido aqui: `.pcd` binário dá 1,61×, o mesmo `.pcd` em texto daria 4,22× |
| **Mudança** | Reportar a razão com os dois lados no mesmo formato, e acrescentar o `.bt` do octomap (`writeBinary`, só ocupação, sem cor), que é bem menor que o `.ot` |
| **Esforço** | Baixo |
| **Ganho** | É uma **correção metodológica**, não um ganho numérico. Vale como achado, não como melhoria de métrica |

---

## Ordem sugerida

| Ordem | Mudança | Por quê |
|---|---|---|
| 1 | **A1** (fila limitada) | Maior impacto, menor esforço, ataca o diagnóstico direto |
| 2 | **A4** (restaurar features) | Trivial, e A1 é o que a torna possível |
| 3 | **A3** (modo offline) | O argumento mais forte: custo zero em precisão |
| 4 | **B1** (poses otimizadas) | Sai quase de graça junto com A3, e corrige defeito real da tese |
| 5 | **C1** (ajuste global final) | Ganho consistente, risco baixo |
| 6 | **A5** + **D1** (Octomap) | Tempo e capacidade de navegação |
| 7 | **B3**, **B4** (qualidade da nuvem) | Melhoram o mapa, não a trajetória |
| 8 | **B2** (covariância por profundidade) | O mais interessante cientificamente, e o mais arriscado. Deixe por último |

**Se o tempo for curto: A1 + A4 + C1.** São as três de menor esforço e maior retorno, e
já sustentam o argumento central.

---

## Como medir qualquer uma delas

Sempre em pares, na mesma máquina, com o harness:

```bash
orb python3 evaluation/harness/run_benchmark.py --config tum_rgbd_dense --runs 10 --tag antes
# aplicar a mudança, recompilar
orb python3 evaluation/harness/run_benchmark.py --config tum_rgbd_dense --runs 10 --tag depois
python3 evaluation/harness/run_benchmark.py --compare evaluation/results/antes evaluation/results/depois
```

**Três regras:**

1. **Uma mudança por vez.** Duas juntas e você não sabe qual funcionou.
2. **10 execuções** ao afirmar melhoria. A variação natural medida aqui foi de ~15%: uma
   mudança menor que o desvio reportado não é resultado.
3. **Rode `validate_dense.py`** depois de qualquer alteração no Grupo A ou B. É fácil
   melhorar o erro de trajetória degradando a nuvem sem perceber.

## O que evitar

- **Mexer em vários parâmetros `Dense.*` junto com mudanças de código.** Fixe os
  parâmetros, mude o código, meça. Depois, se quiser, varra parâmetros separadamente.
- **Comparar com os números publicados da dissertação para justificar uma melhoria.**
  Hardware e versões diferentes. A comparação válida é sempre antes × depois na sua
  máquina.
- **Remover o modo fiel à tese.** Toda mudança do Grupo A deve ser acionável por
  configuração, com o comportamento original preservado. O trabalho precisa mostrar os
  dois lados medidos.
