# Foundry — 서브-하네스 주조 공장

> 제안서(proposal)를 입력하면, 그 프로젝트에 **오버피팅된 서브-하네스(sub-harness)** 를
> 자동으로 주조(forge)하고, 운영 결과를 흡수해 다음엔 더 잘 만드는 메타-하네스(meta-harness).

## 한눈에 보는 철학

이 공장의 목표는 **"많이 만드는 것"이 아니라 "가장 가벼운 것을 만드는 것"** 이다.
하네스 요소 하나하나는 모델에게 **세금(tax)** 이다. 요소가 많아지면 모델이 본업보다
**Policy Maze(규칙 미로)** 를 푸느라 헤맨다. 그래서 Foundry는 주조할 때마다 스스로에게 묻는다:

> "이 요소가 없으면 모델이 정말 실패하는가? 아니면 그냥 *있으면 좋아 보여서* 넣는가?"

후자라면 넣지 않는다. ([references/principles.md](references/principles.md) 참고)

## 디렉토리 구조

```
foundry-harness/
├── README.md                  ← 지금 이 파일
├── skills/
│   ├── forge/SKILL.md         ← 핵심. 제안서 → 서브-하네스 7단계 파이프라인
│   └── evolve/SKILL.md        ← 운영 학습 → 가지치기/승급 (양방향 진화)
├── patterns/                  ← 아키텍처 패턴 라이브러리 (변하지 않는 "어휘")
│   ├── pipeline.md
│   ├── fanout-fanin.md
│   ├── expert-pool.md
│   ├── generate-validate.md
│   ├── supervisor.md
│   └── hierarchical.md
├── references/
│   ├── principles.md          ← Foundry 헌법 (세금/단순함/미로 회피)
│   └── maze-auditor.md        ← Policy Maze 자가 감사 체크리스트
└── templates/                 ← 대상 프로젝트에 "방출"되는 산출물 틀
    ├── AGENTS.md.tmpl
    ├── golden-principles.md.tmpl
    ├── rubric.md.tmpl
    ├── sprint-contract.md.tmpl
    └── agent.md.tmpl
```

## 사용법

```
/forge <제안서 파일 경로 또는 내용>      # 서브-하네스를 주조
/evolve <대상 프로젝트 경로>             # 운영 learning.md를 읽고 가지치기·승급
```

> 위 두 명령은 Claude Code 스킬이다. 실제 등록 방법은 이 저장소를 plugin으로 묶거나
> `forge`/`evolve` SKILL.md 내용을 직접 실행시키면 된다.

## 계층(어디까지 Foundry가 만들고, 어디부터 가져다 쓰나)

| 계층 | 누가 담당 |
|---|---|
| **L0 Foundry** (이 저장소) | 제안서 → 서브-하네스 주조 + 진화 |
| **L1 Sub-harness** (대상 repo에 방출) | golden principles · agents · rubric · docs 골격 |
| **L2 Runtime** (실행) | **OMC(oh-my-claudecode)** 등 기존 범용 하네스에 위임. 없으면 단순 task 루프 |

Foundry는 **L2를 재발명하지 않는다.** 주조한 산출물을 OMC 같은 런타임 위에 올린다.
런타임 연동 지점은 [skills/forge/SKILL.md](skills/forge/SKILL.md)의 6단계(deploy)에 명시.
