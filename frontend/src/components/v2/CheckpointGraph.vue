<template>
    <div
        class="relative-position bg-dark border-rounded"
        :style="`height: ${resolvedHeight}; width: 100%; max-width: 100%; min-width: 0; overflow: auto;`"
    >
        <!-- 빈 상태 — 아직 학습 이력이 없을 때 -->
        <div
            v-if="!flowNodes.length"
            class="absolute-full flex flex-center text-grey column"
        >
            <q-icon name="account_tree" size="48px" class="q-mb-sm" />
            <div>{{ emptyText || $t('checkpointGraphEmpty') }}</div>
        </div>

        <!-- 콘텐츠 크기만큼의 내부 영역. 넘치면 바깥 div 가 네이티브 스크롤로 모든 브랜치를
             보여준다(확대/축소·드래그 이동 없이). -->
        <div
            v-else
            :style="`width: ${layout.width}px; height: ${layout.height}px; min-width: 100%; min-height: 100%;`"
        >
        <VueFlow
            v-model:nodes="flowNodes"
            v-model:edges="flowEdges"
            :nodes-draggable="false"
            :nodes-connectable="false"
            :edges-updatable="false"
            :elements-selectable="true"
            :pan-on-drag="false"
            :pan-on-scroll="false"
            :zoom-on-scroll="false"
            :zoom-on-pinch="false"
            :zoom-on-double-click="false"
            :prevent-scrolling="false"
            :default-viewport="{ x: 16, y: 16, zoom: 1 }"
            class="full-width full-height"
            @pane-ready="anchorFixed"
        >
            <template #node-checkpoint="{ data }">
                <!-- leaf(위) → root(아래) 세로 흐름. 자식이 위, 부모가 아래.
                     엣지: 부모(source).Top → 자식(target).Bottom. Handle 은 앵커 전용(숨김). -->
                <Handle type="target" :position="Position.Bottom" :style="{ opacity: 0 }" />
                <q-card
                    flat
                    bordered
                    class="cursor-pointer bg-dark text-white text-center border-rounded q-px-sm q-py-xs"
                    :class="data.selected ? 'border-primary' : 'border-white'"
                    :style="`width: ${NODE_W}px;`"
                    @click="$emit('node-click', data.checkpoint)"
                >
                    <q-menu v-if="$slots.menu" context-menu>
                        <q-list bordered separator>
                            <slot name="menu" :checkpoint="data.checkpoint" />
                        </q-list>
                    </q-menu>

                    <div class="row items-center no-wrap q-gutter-x-xs">
                        <q-icon
                            :name="statusIcon(data.checkpoint.status)"
                            :color="statusColor(data.checkpoint.status)"
                            size="sm"
                        >
                            <q-spinner
                                v-if="data.checkpoint.status === 'running'"
                                color="orange"
                                size="sm"
                                class="absolute-full"
                            />
                        </q-icon>
                        <div class="col text-left">
                            <div class="text-bold">#{{ data.checkpoint.id }}</div>
                            <div class="text-caption">
                                <span :class="`text-${statusColor(data.checkpoint.status)}`">
                                    {{ statusLabel(data.checkpoint.status) }}
                                </span>
                            </div>
                        </div>
                    </div>
                </q-card>
                <Handle type="source" :position="Position.Top" :style="{ opacity: 0 }" />
            </template>
        </VueFlow>
        </div>
    </div>
</template>

<script setup>
import { ref, computed, watch, nextTick } from 'vue';
import { VueFlow, Handle, Position, useVueFlow } from '@vue-flow/core';
import { useI18n } from 'vue-i18n';
// vue-flow 구조용 스타일시트(엣지 path/transform 등). 라이브러리 필수 CSS —
// 사용자 승인된 No-custom-CSS 예외. theme-default 는 import 안 함(Quasar 카드로 직접 스타일).
import '@vue-flow/core/dist/style.css';

const { t } = useI18n();

const props = defineProps({
    // 전체 상태(finished/running/queued/failed/...) 체크포인트 목록. load_model_id 로 계보 구성.
    checkpoints: { type: Array, default: () => [] },
    // 현재 선택(하이라이트)된 체크포인트 id.
    selectedId: { type: [Number, String], default: null },
    height: { type: [Number, String], default: '100%' },
    emptyText: { type: String, default: '' },
});

defineEmits(['node-click']);

const NODE_W = 124;
const COL_W = 140; // lane(브랜치) 가로 간격 — NODE_W 와의 차(16px)가 노드 사이 여백
const ROW_H = 84;

const LANE_COLORS = ['#bf00bf', '#ff7043', '#26a69a', '#ffca28', '#42a5f5', '#ec407a', '#9ccc65', '#ab47bc'];
const laneColor = (idx) => LANE_COLORS[(idx || 0) % LANE_COLORS.length];

const resolvedHeight = computed(() =>
    typeof props.height === 'number' ? `${props.height}px` : props.height,
);

function statusColor(status) {
    switch (status) {
        case 'finished': return 'primary';
        case 'running': return 'orange';
        case 'queued': return 'amber';
        case 'failed': return 'negative';
        case 'canceled': return 'grey-6';
        default: return 'grey-5'; // waiting 등
    }
}
function statusIcon(status) {
    switch (status) {
        case 'finished': return 'psychology';
        case 'running': return 'model_training';
        case 'queued': return 'schedule';
        case 'failed': return 'error';
        case 'canceled': return 'block';
        default: return 'hourglass_empty';
    }
}
function statusLabel(status) {
    switch (status) {
        case 'finished': return t('checkpointStatusFinished');
        case 'running': return t('checkpointStatusRunning');
        case 'queued': return t('checkpointStatusQueued');
        case 'failed': return t('checkpointStatusFailed');
        case 'canceled': return t('checkpointStatusCanceled');
        default: return t('checkpointStatusWaiting');
    }
}

// load_model_id 계보 → git-graph 트리 레이아웃.
//  · 실패(failed) 체크포인트는 숨긴다.
//  · row(y)  = 루트로부터의 깊이(세대). 부모(위) → 자식(아래) 세로 흐름.
//  · lane(x) = 브랜치 컬럼. 첫 자식은 부모 레인 계승, 나머지는 새 레인(분기 → 가로).
//  · 서로 다른 브랜치(독립 루트=연결 컴포넌트)는 세로 스택이 아니라 가로로 나란히 배치한다.
//    각 컴포넌트는 row 0(상단)에서 시작하고, laneBase 만큼 오른쪽으로 오프셋된다.
const layout = computed(() => {
    const cps = (props.checkpoints || []).filter((c) => c.status !== 'failed');
    if (!cps.length) return { nodes: [], edges: [], width: 0, height: 0 };

    const byId = new Map(cps.map((c) => [c.id, c]));
    const childrenOf = new Map();
    const hasParent = new Map();
    for (const c of cps) {
        const p = c.load_model_id;
        if (p != null && byId.has(p)) {
            hasParent.set(c.id, true);
            if (!childrenOf.has(p)) childrenOf.set(p, []);
            childrenOf.get(p).push(c);
        }
    }

    const ts = (c) => new Date(c.created_at || 0).getTime() || 0;
    const cmp = (a, b) => ts(a) - ts(b) || a.id - b.id;
    for (const arr of childrenOf.values()) arr.sort(cmp);
    const roots = cps.filter((c) => !hasParent.get(c.id)).sort(cmp);

    const pos = new Map(); // id -> { lane, row }
    const visited = new Set();
    let laneBase = 0;
    for (const root of roots) {
        let nextLocalLane = 0;
        let maxLocalLane = 0;
        const dfs = (node, lane, row) => {
            if (visited.has(node.id)) return; // 순환 방어
            visited.add(node.id);
            if (lane > maxLocalLane) maxLocalLane = lane;
            pos.set(node.id, { lane: laneBase + lane, row });
            const kids = childrenOf.get(node.id) || [];
            kids.forEach((kid, k) => dfs(kid, k === 0 ? lane : ++nextLocalLane, row + 1));
        };
        dfs(root, 0, 0);
        laneBase += maxLocalLane + 1; // 다음 컴포넌트는 바로 옆 컬럼으로(브랜치 간격 최소)
    }

    // 경계 계산. pos.row = 루트로부터의 깊이.
    let maxLane = 0, maxDepth = 0;
    for (const p of pos.values()) {
        if (p.lane > maxLane) maxLane = p.lane;
        if (p.row > maxDepth) maxDepth = p.row;
    }

    // y 반전: leaf(깊은 노드)가 맨 위, root(깊이 0)가 맨 아래로 가도록 한다.
    // 모든 루트는 같은 하단 baseline(row = maxDepth)에 정렬된다.
    const selId = props.selectedId == null ? null : String(props.selectedId);
    const nodes = cps.map((c) => {
        const p = pos.get(c.id) || { lane: 0, row: 0 };
        return {
            id: String(c.id),
            type: 'checkpoint',
            position: { x: p.lane * COL_W, y: (maxDepth - p.row) * ROW_H },
            data: { checkpoint: c, selected: selId === String(c.id) },
            draggable: false,
        };
    });
    const edges = cps
        .filter((c) => hasParent.get(c.id))
        .map((c) => ({
            id: `e-${c.load_model_id}-${c.id}`,
            source: String(c.load_model_id),
            target: String(c.id),
            type: 'smoothstep',
            animated: byId.get(c.id)?.status === 'running',
            style: { stroke: laneColor(pos.get(c.id)?.lane), strokeWidth: 2 },
        }));

    // 콘텐츠 경계 → 내부 영역 크기(네이티브 스크롤 범위). 뷰포트 오프셋(16,16) 여유 포함.
    const width = maxLane * COL_W + NODE_W + 48;
    const height = maxDepth * ROW_H + 96;
    return { nodes, edges, width, height };
});

// vue-flow 는 노드 객체를 내부에서 변형(computedPosition 등)하므로 v-model 로 연결하고
// 소스(layout) 변화 시 새 배열로 교체한다.
const flowNodes = ref(layout.value.nodes);
const flowEdges = ref(layout.value.edges);

const { setViewport } = useVueFlow();

// 확대/축소/이동은 비활성(위 VueFlow props). 그래프는 항상 동일한 고정 위치·배율(zoom 1,
// 좌상단 기준)에서 보인다. 데이터가 바뀌어도 뷰포트를 같은 자리로 고정한다.
function anchorFixed() {
    setViewport({ x: 16, y: 16, zoom: 1 });
}

watch(
    layout,
    (next) => {
        flowNodes.value = next.nodes;
        flowEdges.value = next.edges;
        nextTick(anchorFixed);
    },
    { deep: false },
);
</script>
