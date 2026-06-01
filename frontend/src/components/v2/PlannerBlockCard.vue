<template>
    <!-- Planner / Curriculum 공용 블록 카드.
         부모는 click / contextmenu / drag 이벤트를 자유롭게 연결할 수 있고,
         settings 같은 자체 액션은 ``header-right`` 슬롯에 끼워 넣는다.
         자식으로 ``<q-menu context-menu>`` 같은 부가 요소를 두려면 default
         슬롯을 그대로 사용. -->
    <div
        class="column"
        :class="bgClass"
        :style="cardStyle"
    >
        <!-- 상단 타입 색상 stripe -->
        <div :style="{ height: '3px', backgroundColor: stripeColor }"></div>
        <div class="q-pa-xs col column">
            <!-- 헤더: 아이콘 + 실행/결과 표시 + 우측 슬롯 -->
            <div class="row items-center no-wrap">
                <q-icon
                    :name="typeIcon"
                    :color="typeIconColor"
                    size="xs"
                    class="q-mr-xs"
                />
                <q-spinner
                    v-if="running"
                    color="primary"
                    size="xs"
                />
                <q-icon
                    v-else-if="resultIcon"
                    :name="resultIcon"
                    :color="resultColor"
                    size="xs"
                />
                <q-space />
                <slot name="header-right" />
            </div>
            <!-- 제목 -->
            <div class="text-caption text-bold text-white ellipsis q-mt-xs">
                {{ block.name || blockConfig?.label || block.type }}
            </div>
            <!-- 부제 — 슬롯 우선, 없으면 type 별 기본 -->
            <div class="text-caption text-grey-5 ellipsis">
                <slot name="subtitle">{{ defaultSubtitle }}</slot>
            </div>
            <q-space />
            <!-- 뱃지 row: sync / until-done / duration / step progress.
                 세로 stack (full-width 카드) 에선 우측 정렬이 더 균형 — 가로
                 카드 (160px 고정) 에선 좌측 기본. ``badgesAlign`` 으로 제어. -->
            <div
                v-if="hasBadges"
                class="row q-mt-xs q-gutter-x-xs"
                :class="{ 'justify-end': badgesAlign === 'right' }"
            >
                <q-badge
                    v-if="block.type === 'sync'"
                    rounded
                    color="teal-8"
                >
                    <q-icon name="sync" size="xs" class="q-mr-xs" />{{ $t('plannerSyncBadge') }}
                </q-badge>
                <q-badge
                    v-else-if="block.type === 'checkpoint' && block.until_done"
                    rounded
                    color="purple-8"
                >
                    <q-icon name="check_circle" size="xs" class="q-mr-xs" />{{ $t('plannerUntilDoneBadge') }}
                </q-badge>
                <q-badge
                    v-else-if="block.duration != null"
                    rounded
                    :color="block.type === 'timesleep' ? 'orange-8' : 'purple-8'"
                >
                    <q-icon name="timer" size="xs" class="q-mr-xs" />{{ block.duration }}s
                </q-badge>
                <q-badge
                    v-if="block.type === 'checkpoint' && progress"
                    rounded
                    color="blue-grey-7"
                    :style="{ fontSize: '9px', padding: '1px 4px' }"
                >
                    {{ progress.step }}<span v-if="progress.maxSteps">/{{ progress.maxSteps }}</span>
                </q-badge>
            </div>
        </div>
        <!-- 부가 요소 (context menu 등) — default 슬롯 -->
        <slot />
    </div>
</template>

<script setup>
import { computed } from 'vue';

const props = defineProps({
    block: { type: Object, required: true },
    // backend `/api/planner/block_configs` 의 한 entry. 미지정 시 아이콘/색은 default.
    blockConfig: { type: Object, default: null },
    // 실행 중이면 spinner, 끝나면 result 아이콘 (finished/stopped/error).
    running: { type: Boolean, default: false },
    result: { type: String, default: null },
    // 체크포인트 step progress. { step, maxSteps }. maxSteps null 이면 step 만 표시.
    progress: { type: Object, default: null },
    // 부제 기본값 분기 (joint_position / move_relative_ee 일 때 workspace 이름).
    workspaceName: { type: String, default: '' },
    // 카드 폭 — null 이면 부모 컨테이너에 맞춰 채움 (curriculum 세로 stack).
    // 문자열 (e.g. '160px') 이면 fixed (planner 가로 row).
    width: { type: String, default: null },
    // 강조 상태. true 면 primary border + bg-blue-grey-9.
    active: { type: Boolean, default: false },
    // 강제 bg class — 미지정 시 active 에 따라 default.
    bgClass: { type: String, default: null },
    // 강제 stripe 색 — 미지정 시 type 별 기본.
    stripeColor: { type: String, default: null },
    // 뱃지 정렬 — 'left' (가로 카드 기본) | 'right' (세로 stack 카드 권장).
    badgesAlign: {
        type: String,
        default: 'left',
        validator: (v) => ['left', 'right'].includes(v),
    },
});

const BLOCK_TYPE_COLORS = {
    joint_position: '#2196F3',
    move_relative_ee: '#00BCD4',
    replay_episode: '#4CAF50',
    checkpoint: '#9C27B0',
    timesleep: '#FF9800',
    sync: '#009688',
    query_pose: '#3F51B5',
};

const stripeColor = computed(() => props.stripeColor
    || BLOCK_TYPE_COLORS[props.block.type]
    || '#9E9E9E');

const typeIcon = computed(() => props.blockConfig?.icon || 'help');
const typeIconColor = computed(() => props.blockConfig?.color || 'grey');

const resultIcon = computed(() => {
    if (props.result === 'finished') return 'check_circle';
    if (props.result === 'stopped') return 'cancel';
    if (props.result === 'error') return 'error';
    return null;
});
const resultColor = computed(() => {
    if (props.result === 'finished') return 'positive';
    if (props.result === 'stopped') return 'orange';
    if (props.result === 'error') return 'negative';
    return 'grey';
});

const bgClass = computed(() => {
    if (props.bgClass) return props.bgClass;
    return props.active ? 'bg-blue-grey-9' : 'bg-grey-10';
});

const cardStyle = computed(() => ({
    width: props.width || null,
    minWidth: props.width || null,
    maxWidth: props.width || null,
    flexShrink: 0,
    borderRadius: '6px',
    overflow: 'hidden',
    border: props.active
        ? '1px solid var(--q-primary)'
        : '1px solid rgba(255,255,255,0.06)',
    transition: 'border-color 0.15s ease',
}));

const defaultSubtitle = computed(() => {
    const b = props.block;
    switch (b.type) {
        case 'joint_position':
        case 'move_relative_ee':
            return props.workspaceName;
        case 'checkpoint':
            return b.checkpoint_name || '';
        case 'replay_episode':
            return `${b.dataset_name || `Dataset ${b.dataset_id}`} · ep.${b.episode_index}`;
        case 'timesleep':
            return '—';
        case 'sync':
            return `ID: ${b.sync_id || '—'}`;
        case 'query_pose':
            return b.service_name || '—';
        default:
            return '';
    }
});

const hasBadges = computed(() => {
    const b = props.block;
    return b.type === 'sync'
        || b.duration != null
        || b.until_done
        || (b.type === 'checkpoint' && props.progress);
});
</script>
