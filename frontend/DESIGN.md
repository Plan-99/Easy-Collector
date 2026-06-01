# 프론트엔드 디자인 가이드 (DESIGN.md)

이 문서는 `frontend/` 페이지·컴포넌트를 만들거나 수정할 때 **반드시** 따라야 하는 디자인/구현 패턴을 정리한다.
신규 화면은 기존 페이지(`SensorPage.vue`, `RobotPage.vue` 등)와 **시각적·구조적으로 구분되지 않아야** 한다.
새 패턴이 필요하면 임의로 만들지 말고 이 문서를 먼저 갱신할 것.

---

## 0. 절대 규칙 (먼저 읽기)

1. **커스텀 CSS 금지.** Vue + Quasar 만 사용한다. `<style>` 블록과 임의 `.scss` 추가 금지.
   - 스타일은 Quasar 유틸리티 클래스(`q-pa-md`, `row`, `text-primary` …)와 컴포넌트로 해결한다.
   - 공용 유틸 클래스는 [src/css/app.scss](src/css/app.scss)에만 존재한다 (`border-rounded`, `border-white`, `border-primary` 등). 새 전역 클래스 추가도 지양.
   - 동적으로 계산되는 위치/크기만 인라인 `:style`로 허용 (예: `:style="!robots.length ? 'min-height: 220px;' : ''"`).
2. **모든 텍스트는 i18n.** 하드코딩 문자열 금지. 템플릿은 `$t('key')`, 스크립트는 `const { t } = useI18n()` 후 `t('key')`.
   - 키 추가 시 [src/i18n/ko-KR/index.js](src/i18n/ko-KR/index.js)와 [src/i18n/en-US/index.js](src/i18n/en-US/index.js)를 **동시에** 1:1로 갱신한다.
3. **프론트엔드 기능 추가/변경 후에는 Playwright UI 테스트**(`.claude/skills/playwright-skill`)로 검증하고, 캡처는 Read로 직접 확인한다.
4. **ESLint/Prettier 준수**: 세미콜론 없음, 작은따옴표, print width 100. (pre-commit이 잡지만 작성 시 의식할 것)

---

## 1. 테마 / 색상

색상은 [src/css/quasar.variables.scss](src/css/quasar.variables.scss)에 정의된 브랜드 팔레트를 클래스로만 사용한다. 직접 hex 코드를 쓰지 않는다.

| 역할 | 변수 | 값 | 용도 |
|------|------|-----|------|
| primary | `$primary` | `#00E7FF` (시안) | 강조, 활성 상태, 토글 ON, 선택된 테두리 |
| secondary | `$secondary` | `#2C2D30` | **카드 배경**, 헤더, 드로어 패널 |
| accent | `$accent` | `#FF7017` (주황) | 보조 강조 |
| dark | `$dark` | `#212024` | **페이지 배경**, 입력 필드 배경, 내부 박스 |
| positive | `$positive` | `#21BA45` | 확인/제출 버튼, 성공 |
| negative | `$negative` | `#C10015` | 삭제/에러 |
| warning | `$warning` | `#F2C037` | 경고 |

색상 위계: **페이지 = `bg-dark`** → **카드/패널 = `bg-secondary`** → **카드 안 입력/박스 = `bg-dark`**.

폰트는 전역 `Paperozi`(app.scss `@font-face`). 추가 폰트 지정 금지.

### 자주 쓰는 색 클래스
- 텍스트: `text-white`(기본 본문), `text-primary`(강조), `text-grey-6`/`text-grey-7`(보조 캡션), `text-negative`, `text-orange`.
- 배경: `bg-dark`, `bg-secondary`, `bg-primary`(활성 항목).
- 테두리(app.scss): `border-rounded`(12px radius), `border-white`, `border-primary`.

---

## 2. 페이지 레이아웃 골격

모든 라우트는 [src/router/routes.js](src/router/routes.js)에서 `layouts/v2/MainLayout.vue`의 자식으로 lazy import 된다. 페이지는 `src/pages/v2/*.vue`.

표준 페이지 구조 (`SensorPage`, `RobotPage` 공통):

```vue
<template>
    <q-page class="q-pt-md q-pr-md full-height">
        <!-- 1) 인트로 배너 -->
        <div class="border-rounded bg-secondary q-pa-md q-mb-md row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl" />
            <div>
                <div class="text-h5 text-primary text-bold q-mb-md">{{ $t('xxxIntroTitle') }}</div>
                <div class="text-body text-white">{{ $t('xxxIntroBody') }}</div>
            </div>
        </div>

        <!-- 2) 튜토리얼 힌트 -->
        <TutorialHint class="q-mb-md" :text="$t('tutorialXxxIntro')" />

        <!-- 3) 본문 (카드 그리드 등) -->
        <div class="row q-col-gutter-md"> ... </div>

        <!-- 4) 하단 터미널/폼 다이얼로그 -->
    </q-page>
</template>
```

- 루트는 항상 `<q-page class="q-pt-md q-pr-md full-height">`. (페이지 배경 `bg-dark`는 `MainLayout`의 `q-page-container`가 담당)
- 페이지 상단은 거의 항상 **인트로 배너 + TutorialHint**로 시작한다.
- 간격은 Quasar 스페이싱 클래스(`q-pa-md`, `q-mb-md`, `q-mt-sm`, `q-col-gutter-md`)로만 준다. 임의 margin/padding 인라인 금지.

---

## 3. 카드 그리드 패턴 (목록 화면)

엔티티(센서/로봇 등) 목록은 반응형 카드 그리드로 표시한다. 표준 컬럼: `col-6 col-sm-4 col-md-3 col-lg-2`.

```vue
<div class="row q-col-gutter-md">
    <!-- 항목 카드 -->
    <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="item in items" :key="item.id">
        <q-card
            class="q-pa-md bg-secondary border-rounded border-white text-white full-height"
            :class="selected && item.id === selected.id ? 'border-primary' : ''"
        >
            <!-- 우클릭 컨텍스트 메뉴 (편집/삭제) -->
            <q-menu context-menu>
                <q-list bordered separator>
                    <q-item clickable v-ripple v-close-popup @click="openEdit(item)">
                        <q-item-section>{{ $t('edit') }}</q-item-section>
                        <q-item-section side><q-icon name="edit" size="xs" /></q-item-section>
                    </q-item>
                    <q-item clickable v-ripple class="text-negative" @click="remove(item)">
                        <q-item-section>{{ $t('delete') }}</q-item-section>
                        <q-item-section side><q-icon color="negative" name="visibility" size="xs" /></q-item-section>
                    </q-item>
                </q-list>
            </q-menu>

            <q-img :src="item.image" @click="select(item)" class="cursor-pointer bg-white" ratio="1.5" fit="contain" />

            <q-card-section class="q-pa-none q-mt-sm">
                <div class="text-bold">{{ item.name }}</div>
                <div class="text-grey-6 text-caption">{{ item.type }}</div>
            </q-card-section>

            <!-- 상태 + 토글 -->
            <q-card-section class="q-pa-none q-mt-sm row">
                <div class="text-primary text-caption" v-if="item.status === 'on'">{{ $t('statusOnline') }}</div>
                <div class="text-orange text-caption" v-else-if="item.status === 'loading'">{{ $t('statusLoading') }}</div>
                <div class="text-negative text-caption" v-else-if="item.status === 'error'">{{ $t('statusError') }}</div>
                <div class="text-grey-7 text-caption" v-else>{{ $t('statusOffline') }}</div>
                <q-space />
                <q-toggle :model-value="item.status === 'on'" color="primary" dense @click.stop="toggle(item)" />
            </q-card-section>

            <!-- 로딩 오버레이 -->
            <q-inner-loading :showing="item.status === 'loading'">
                <q-spinner-gears size="50px" color="primary" />
            </q-inner-loading>
        </q-card>
    </div>

    <!-- 빈 상태 카드 -->
    <div class="col" v-if="!items.length">
        <q-card class="full-height border-rounded border-white bg-dark text-white" flat bordered>
            <q-card-section class="text-center">
                <div class="text-h6">{{ $t('noItemTitle') }}</div>
                <div class="text-subtitle2">{{ $t('noItemBody') }}</div>
            </q-card-section>
        </q-card>
    </div>

    <!-- 추가 버튼 카드 -->
    <div class="col-6 col-sm-4 col-md-3 col-lg-2" :style="!items.length ? 'min-height: 220px;' : ''">
        <q-btn color="white" class="full-height full-width border-rounded" outline size="lg" icon="add" @click="openAdd" />
    </div>
</div>
```

공통 규약:
- 선택/활성 카드는 `:class`로 `border-primary` 토글.
- **상태 표시 컬러 규약**: `on` → `text-primary`, `loading` → `text-orange`, `error` → `text-negative`, `off/offline` → `text-grey-7`.
- 편집·삭제는 카드 우클릭 `q-menu context-menu`로 제공.
- 항상 **빈 상태 카드**와 **`add` 아이콘 추가 버튼**을 둔다.
- 비동기 진행은 `q-inner-loading` + `q-spinner-gears color="primary"`.

---

## 4. 폼 / 다이얼로그

폼은 직접 `q-input`을 나열하지 말고 **데이터 주도형** [src/components/v2/FormDialog.vue](src/components/v2/FormDialog.vue)를 사용한다.

폼은 필드 객체 배열로 선언한다:

```js
const sensorForm = ref([
    { key: 'id', value: null },                                  // 숨김 필드 (편집 시 PK)
    { key: 'name', label: t('sensorName'), value: '', default: '', type: 'text' },
    { key: 'type', label: t('sensorType'), value: '', default: '', type: 'select',
      options: computed(() => /* ... */) },
    { key: 'ip_address', label: t('ipAddress'), value: '', default: '', type: 'text',
      show: (form) => /* 조건부 표시 predicate */ },
])
```

지원 `type`: `text`, `number`, `select`, `multiselect`, `multiselect_list`, `checkbox`, `custom`(named slot).
필드 옵션: `label`, `value`, `default`, `placeholder`, `options`, `optional`(검증 제외), `show(form)`(조건부 표시), `maxValues`.

사용:

```vue
<form-dialog
    v-model="showForm"
    :title="$t(isEdit ? 'editTitle' : 'addTitle')"
    :form="form"
    :ok-button-label="$t(isEdit ? 'save' : 'add')"
    @submit="save"
>
    <TutorialHint class="q-mb-md" :text="$t('tutorialForm')" />
</form-dialog>
```

- 다이얼로그를 새로 만들 때도 FormDialog의 스타일을 따른다: 헤더 `bg-dark text-white` + 우상단 닫기 `q-btn round icon="close"`, 본문 `bg-secondary`, 카드 `border-rounded border-white`, 입력은 `dense outlined dark bg-color="dark"`.
- 하단 버튼: 취소 = `color="white" text-color="dark"`, 확인/제출 = `color="positive" unelevated`.
- `openAdd`는 모든 필드를 `default`로 리셋 후 열고, `openEdit`는 엔티티 값으로 채운다(아래 §6).

---

## 5. 공용 컴포넌트 (재사용 우선)

새 컴포넌트를 만들기 전에 [src/components/v2/](src/components/v2/)에 이미 있는지 확인한다.

| 컴포넌트 | 용도 |
|----------|------|
| `FormDialog.vue` | 데이터 주도형 생성/편집 폼 다이얼로그 |
| `TutorialHint.vue` | 인라인 도움말/튜토리얼 힌트 박스 (`:text`, `step`) |
| `BottomTerminal.vue` | 하단 탭형 미리보기/콘솔 영역 (`:tabs`, `tab-label`, `tab-value`, `v-model`) |
| `ProcessConsole.vue` | 백엔드 프로세스 로그 출력 (`:process`) |
| `WebRtcVideo.vue` | 실시간 카메라 스트림 (`:process-id`, `:topic`, `:msg-type`) |
| `EssentialLink.vue` | 좌측 드로어 네비 항목(2단 메뉴 지원) |
| `PipelineGuideDialog.vue` | 파이프라인 안내 다이얼로그 |

- 컴포넌트는 `<script setup>` + `defineProps`/`defineEmits` 사용. props는 `type`/`default`(또는 `required`) 명시.
- import 경로는 `src/...` 또는 `components/...` alias 사용.

---

## 6. 스크립트 / 데이터 흐름 패턴

- **`<script setup>` + Composition API**만 사용. `ref`/`computed`/`onMounted`.
- **API 호출**: `import { api } from 'src/boot/axios'` 후 `api.get/post/put/delete('/...')`. 베이스는 `http://localhost:5000/api`.
- **목록 로드 → 가공** 패턴:
  ```js
  function listItems() {
      return api.get('/items').then((res) => {
          items.value = res.data.items || []
          items.value.forEach(i => { i.image = `/images/${i.type}.png` })
      }).catch((e) => console.error(e))
  }
  onMounted(() => { listItems() })
  ```
- **CRUD 핸들러 네이밍**: `listX`, `openAddXForm`, `openEditXForm`, `saveX`, `deleteX`, `toggleX`.
  - `openAdd*`: 필드를 `default`로 리셋 후 다이얼로그 오픈.
  - `openEdit*`: 엔티티 값으로 필드 채우고 `id` 필드 설정.
  - `save*`: `id` 유무로 `put` vs `post` 분기.
- **실시간/장치 로직은 composable로 분리**: [src/composables/](src/composables/)의 `useSensor`, `useRobot`, `useROS`, `useSocket`, `useWebRTC`, `useTraining` 등. 페이지는 핸들러를 받아 호출만 한다 (`sensor.handler = useSensor(sensor, ...)`).
- **전역 상태는 Pinia store**: [src/stores/](src/stores/)의 `processStore`, `topicStore`, `tutorialStore`, `modulesStore`. `const tutorial = useTutorialStore()` 형태.

---

## 7. 알림 / 사용자 피드백

`Notify` (Quasar plugin, 이미 등록됨)을 사용한다. 직접 토스트 UI를 만들지 않는다.

```js
import { Notify } from 'quasar'

Notify.create({ type: 'positive', message: t('xxxSuccess'), timeout: 4000 })
Notify.create({ type: 'negative', message: t('xxxFailed') })
Notify.create({ type: 'info', message: t('xxxInfo'), timeout: 2500 })
```

- `type`: `positive` / `negative` / `info` / `warning`.
- 메시지는 항상 i18n 키. 에러 메시지는 `e?.response?.data?.message || e.message` 패턴으로 추출.
- 진행 중 오버레이는 `q-inner-loading` 또는 `Loading` plugin.

---

## 8. 아이콘 / 이미지

- 아이콘은 Material Icons 이름 문자열 (`icon="add"`, `name="edit"`, `name="close"`, `name="school"` …).
- 엔티티 이미지는 `/images/{type}.png` 규약, 폴백은 `/images/custom_sensor.png` 류.
- 네비게이션 아이콘은 `MainLayout.vue`의 `linksList`에서 관리.

---

## 9. 네비게이션 추가 체크리스트

새 페이지를 추가할 때:
1. [src/pages/v2/](src/pages/v2/)에 `XxxPage.vue` 생성 (위 골격 준수).
2. [src/router/routes.js](src/router/routes.js)에 lazy import 라우트 추가.
3. [src/layouts/v2/MainLayout.vue](src/layouts/v2/MainLayout.vue)의 `linksList`에 `{ title: t('navXxx'), icon, link }` 추가 (필요 시 `children`로 2단 메뉴).
4. `navXxx`, `xxxIntroTitle/Body`, `tutorialXxxIntro`, 상태/폼 키를 **ko-KR / en-US 양쪽** i18n에 추가.
5. Playwright로 ko/en 모두 렌더 확인 후 캡처 검사.

---

## 10. 빠른 체크리스트 (PR 전)

- [ ] `<style>` 블록 / 커스텀 CSS 없음
- [ ] 모든 텍스트가 `$t()` / `t()` (ko·en 동시 추가)
- [ ] 색은 브랜드 클래스(`text-primary`, `bg-secondary`, `bg-dark` …)로만
- [ ] 카드/폼/알림이 기존 페이지와 동일한 패턴
- [ ] 공용 컴포넌트(FormDialog/TutorialHint/BottomTerminal 등) 재사용
- [ ] ESLint 통과 (세미콜론 없음, 작은따옴표, width 100)
- [ ] Playwright UI 테스트 + 캡처 확인 완료
- [ ] 코드 수정 후 `bash scripts/quick_apply.sh ./ /opt/easytrainer/project`로 런타임 반영
```

