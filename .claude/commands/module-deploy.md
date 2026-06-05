---
description: 로봇/센서/확장 모듈을 구현→검증→마켓 릴리즈(module_up)→홈페이지 카탈로그 노출까지 끝내는 전과정 런북
argument-hint: "<category/name> (예: robots/omy) | 또는 '구현부터' 자연어로"
---

EasyTrainer 모듈 한 개를 **구현부터 실제 노출까지** 끝내는 프로젝트 전용 스킬.
배경 규칙은 [CLAUDE.md](../../CLAUDE.md) "Modules", 폴더 책임은
[FOLDERS.md](../../FOLDERS.md), 원격 드라이버 설계는
[docs/design-docs/2026-06-02_remote-ssh-robot-driver.md](../../docs/design-docs/2026-06-02_remote-ssh-robot-driver.md) 참고.

핵심 메커니즘 2개는 스크립트로 codify 되어 있다 — 손으로 git/worktree 를 만지지 말 것:
- [scripts/release_module.sh](../../scripts/release_module.sh) — 격리 worktree 로 module_up 에 **모듈만** 커밋·푸시→CI watch→tar.gz 검증
- [scripts/upsert_module_catalog.sh](../../scripts/upsert_module_catalog.sh) — 홈페이지 라이브 카탈로그(Neon)에 1행 upsert

---

## 0. 어떤 모듈인지 정한다

`category ∈ {robots, sensors, extensions}`, 폴더는 `modules/<category>/<name>/`.
이미 비슷한 벤더 모듈이 있으면 **그걸 템플릿으로** 시작한다 (예: ROBOTIS 면 `omx`/`omy`,
Dynamixel SDK 면 `piper`). 기존 모듈 한 개의 `module.json` 을 먼저 정독할 것.

## 1. 구현 (modules/<category>/<name>/)

`module.json` 필수 top-level 키: `id, name, version, category, description, install, dependencies, check`
( [scripts/validate_modules.py](../../scripts/validate_modules.py) 가 강제). 그 외:

- **robots[] / sensors[]** 배열: 각 항목에 `type`, `spec`(joint_dim/joint_names/
  joint_*_bounds/tool_index/custom_fields/ik_available 등), `driver`, `ik`.
- **driver.launch** `{package, launch_file, args:{k:"{설정키|기본값}"}}` — 컨테이너에서
  `ros2 launch` 로 뜬다. `pre_launch`(스크립트)/`post_launch`(ros_service) 훅 가능.
- **driver.remote** (온보드 PC 로봇, SSH): `{enabled_when, host_field, user_field,
  port_field, password_field, default_user, ros_domain_id, read_topic/write_topic/
  write_topic_msg override, provision:[{name,check,run,timeout,optional}], launch}`.
  ssh_host 가 차면 로봇 PC 에서 드라이버 실행, EasyTrainer 는 토픽만 DDS 로 소비.
  → 위 design-doc 참고. (custom_fields 에 `ssh_host/ssh_user/ssh_port/ssh_password`,
  암호 인증 쓰면 deps.apt 에 `sshpass` 추가.)
- **install.ros2** `{target:"ros2/ros2_ws/src", build:"colcon"}` / **install.sdk**
  `{target, install_cmd}`. ros2 패키지는 `ros2/src/<pkg>/`, sdk 는 `sdk/`.
- **ik**: `urdf_path`/`urdf_package_dir` 는 `{ros2_root}` placeholder 사용
  (런타임에 `/root/ros2_ws/src/<id>` 로 치환). `ee_definitions:[{name,parent(joint명),
  offset(=joint 프레임 기준 SE3 translation)}]`, `joints_to_lock`.
- **dependencies.apt/pip**: 호스트엔 절대 설치 안 함. ros2/backend **이미지에 baked**
  (모듈 설치 시 rebuild). 런타임 설치 아님 — [feedback 메모리](../../CLAUDE.md) 정책.

### ⚠️ 패키지 이름 충돌 (ROS 모듈 필수 체크)
colcon 은 src/ 아래를 **재귀** 스캔하므로, 다른 모듈과 **같은 ROS 패키지 이름**이면
둘 다 설치 시 빌드 충돌한다. 업스트림을 가져올 땐 패키지를 리네임하라.
예: OMY 는 `open_manipulator_description/bringup` → `omy_description/omy_bringup`
(package.xml/CMakeLists/setup.py 의 name, 모든 `package://`·`$(find …)`·launch 의
FindPackageShare, generated .urdf 까지 sed 치환).

## 2. 검증 (가능한 한 ros2 컨테이너에서)

```bash
python3 scripts/validate_modules.py modules/<category>/<name>/module.json   # 스키마
python3 -c "import json;json.load(open('modules/<category>/<name>/module.json'))" # JSON
```
ROS 모듈이면 `easytrainer_ros2` 컨테이너에서 실제로:
- 임시 ws 에 패키지 복사 후 `colcon build --packages-select <pkgs>`
- `xacro <urdf.xacro> use_mock_hardware:=true` 파싱 (realsense 등 의존 패키지 확인)
- IK: `pinocchio.buildModelFromUrdf` → `buildReducedModel(joints_to_lock)` → EE frame
  위치 확인 ( `ee_definitions` parent/offset 검증)
- ros2_control xacro 에 컨트롤러 joint 들이 `position` command_interface 로 있는지

프론트엔드 변경(로봇 페이지 custom_fields 등)이 있으면 ko/en i18n 양쪽 + Playwright
검증까지 ([feedback 메모리](../../CLAUDE.md): UI 테스트 필수).

## 3. 마켓 릴리즈 (module_up → CI → Plan-99/Easy-Trainer-Modules)

**버전 규칙**: 모듈 파일을 고쳤으면 `module.json` 의 `version` 을 올린다
(같은 버전은 CI 가 릴리즈 스킵). 신규 모듈은 `1.0.0`.

```bash
bash scripts/release_module.sh <category/name> "커밋 메시지"
```
스크립트가 하는 일(전부 격리 worktree — 현재 작업 트리 안 건드림):
검증 → 기존 릴리즈 중복 체크 → `origin/module_up` 위 worktree → **모듈 폴더만**
커밋 → push → `Package Modules` CI watch → 릴리즈된 `module-<id>-v<ver>.tar.gz`
다운로드해 module.json 내용 검증.

> CI 트리거: `modules-release.yml` 은 `main`/`module_up` push 의 `modules/**` 변경을
> `HEAD~1..HEAD` diff 로 감지. `module_up` 은 모듈 전용 릴리즈 브랜치다(본체 코드 안 섞음).

실패 시: 버전 충돌이면 version 올려 재실행. CI 실패면 Actions 로그 확인.

## 4. 카탈로그 등록 (3곳 — 특히 라이브 DB 를 잊지 말 것)

모듈이 릴리즈돼도 **이 단계 없이는 런처/홈페이지 목록에 안 뜬다.**

1. **런처 레지스트리** — [release/ui/modules.py](../../release/ui/modules.py) `MODULE_REGISTRY`
   에 `ModuleInfo(id=…, asset_name="module-<id>-{version}.tar.gz")` 한 줄 추가.
2. **홈페이지 seed 파일** — [home-next/prisma/seed-modules.sql](../../home-next/prisma/seed-modules.sql)
   에 행 추가 (fresh-seed / 문서용 소스 오브 트루스).
3. **홈페이지 라이브 DB (필수!)** — `/dashboard/modules`·`/api/modules` 는 Neon
   `Module` 테이블에서 읽고, **생성 API 도 자동 seed 도 없다.** 직접 upsert:
   ```bash
   bash scripts/upsert_module_catalog.sh <category/name>          # dry-run: SQL 확인
   bash scripts/upsert_module_catalog.sh <category/name> --prod   # 라이브 Neon 반영 + 검증
   ```
   재배포 불필요(즉시 반영). 스크립트가 라이브 API 로 노출 확인까지 한다.

### 카탈로그 썸네일 이미지 (있으면)
모듈 카드 이미지는 [home-next/src/app/dashboard/modules/ModuleCatalog.tsx](../../home-next/src/app/dashboard/modules/ModuleCatalog.tsx)
의 `MODULE_IMAGE` 맵(`<module_id>` → `/modules/<name>.png`)으로 붙는다. 매핑이 없으면
카드에 이미지가 안 뜬다(나머지 정보는 정상).
- 맵에 `"<module_id>": "/modules/<name>.png"` 한 줄 추가.
- 실제 PNG 를 [home-next/public/modules/](../../home-next/public/modules/)`<name>.png` 에 둔다
  (기존 `piper.png` 등과 같은 폴더/형식). 사용자가 채팅에 붙여넣은 이미지는 바이트로 저장
  못 하니 파일 경로(예: `/tmp/<name>.png`)로 받아 옮긴다.
- **로봇 모듈은 런처(frontend)에도 이미지가 따로 있다** — RobotPage/AssemblyForm 은
  `/images/<company>.png` (`frontend/public/images/`, **회사 단위** — 모델 단위 아님).
  home-next(모델별)와 frontend(회사별)는 **배포가 분리**돼 런타임 공유 불가 → 각 public 에
  **수동으로** 둔다(중복 허용 — 사용자 결정). 같은 회사 여러 모델(예: ROBOTIS 의 omx/omy/
  ai_worker)이면 frontend 는 `ROBOTIS.png` 하나를 공유한다.

## 5. 본체 코드(있으면) 는 별도로

모듈의 새 `driver.*` 필드를 **소비하는 코드**(예: `ros2/ros2_bridge` 의 원격 드라이버
실행기, 프론트 로봇 페이지 필드)는 모듈 tar.gz 와 별개다. 일반 플로우로 커밋/PR →
배포(quick-apply + 컨테이너 재시작, 이미지 deps 는 모듈 설치 시 baked)해야 실제 동작한다.
모듈 릴리즈와 본체 PR 을 **섞지 말 것** (작업 트리에 다른 사람 작업이 있으면 pathspec
커밋으로 내 파일만 분리).

## 6. 보고

- 릴리즈 태그/URL, 검증 결과(파일 수·version), 라이브 API 노출 여부.
- 아직 안 된 시스템 단계(본체 push/PR, 컨테이너 재시작)는 명시.

---

### 인자
`$ARGUMENTS` 가 `<category/name>` 형태면 그 모듈로 3~4단계부터 진행.
"구현부터 …" 같은 자연어면 1단계부터. 비어 있으면 어떤 모듈인지 먼저 물어볼 것.
