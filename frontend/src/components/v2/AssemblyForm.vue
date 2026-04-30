<template>
    <div class="row q-gutter-x-md">
        <div>
            <q-input
                v-model="assemblyForm.name"
                dense
                dark
                outlined
                bg-color="dark"
                :label="$t('assemblyName')"
            ></q-input>
            <canvas ref="canvasRef" width="300" height="300"></canvas>
        </div>
        <div class="col">
            <q-scroll-area style="height: 300px" class="text-white">
                <div class="row q-col-gutter-md">
                    <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="robot in robots" :key="robot.id">
                        <q-card class="q-pa-md bg-secondary border-rounded border-white text-white cursor-pointer"
                            @click="includeRobot(robot, null, isIncluded(robot))"
                            :class="{
                                'border-primary': isIncluded(robot)
                            }"
                        >
                            <q-img :src="robot.image" class="bg-white" ratio="1.5" fit="contain">
                            </q-img>

                            <q-card-section class="q-pa-none q-mt-sm">
                                <div class="text-bold">{{ robot.name }}</div>
                                <div class="text-grey-6 text-caption">{{ robot.type }}</div>
                            </q-card-section>
                            <q-card-section class="q-pa-none q-mt-sm">
                                <div class="row q-col-gutter-xs" v-if="isIncluded(robot) && robot.role === 'single_arm'">
                                    <q-btn
                                        class="col"
                                        dense
                                        color="primary"
                                        :text-color="assemblyForm.left_arm_id !== robot.id ? 'white' : 'black'"
                                        :flat="assemblyForm.left_arm_id !== robot.id"
                                        @click.stop="includeRobot(robot, 'left_arm')"
                                    >{{ $t('assemblyLeftBtn') }}</q-btn>
                                    <q-btn
                                        dense
                                        class="col"
                                        color="primary"
                                        :text-color="assemblyForm.right_arm_id !== robot.id ? 'white' : 'black'"
                                        :flat="assemblyForm.right_arm_id !== robot.id"
                                        @click.stop="includeRobot(robot, 'right_arm')"
                                    >{{ $t('assemblyRightBtn') }}</q-btn>
                                </div>
                                <div
                                    class="row q-col-gutter-xs"
                                    v-if="isIncluded(robot) && robot.role === 'tool'"
                                >
                                    <q-btn
                                        class="col"
                                        dense
                                        color="primary"
                                        :text-color="assemblyForm.left_tool_id !== robot.id ? 'white' : 'black'"
                                        :flat="assemblyForm.left_tool_id !== robot.id"
                                        @click.stop="includeRobot(robot, 'left_tool')"
                                    >{{ $t('assemblyLeftBtn') }}</q-btn>
                                    <q-btn
                                        dense
                                        class="col"
                                        color="primary"
                                        :text-color="assemblyForm.right_tool_id !== robot.id ? 'white' : 'black'"
                                        :flat="assemblyForm.right_tool_id !== robot.id"
                                        @click.stop="includeRobot(robot, 'right_tool')"
                                    >{{ $t('assemblyRightBtn') }}</q-btn>
                                </div>
                            </q-card-section>
                        </q-card>
                    </div>
                </div>
            </q-scroll-area>
            <div class="row">
              <q-space></q-space>
              <q-btn
                class="q-mt-md"
                color="primary"
                text-color="black"
                @click="saveAssembly"
              >{{ $t('assemblySaveBtn') }}</q-btn>
            </div>
        </div>
    </div>
</template>

<script setup>
import { computed, defineProps, onMounted, ref, watch, defineEmits } from 'vue';
import { api } from 'src/boot/axios';
import { Notify } from 'quasar';
import { useI18n } from 'vue-i18n';

const { t } = useI18n();

const canvasRef = ref(null);
let ctx = null;

const props = defineProps({
    assembly: {
        type: Object,
        required: false,
    }
});

const emit = defineEmits(['save']);


const robots = ref([]);

function listRobots() {
    api.get('/robots').then((response) => {
        robots.value = response.data.robots;
        robots.value.forEach(robot => {
            robot.image = '/images/' + robot.company + '.png'; // Default image if not provided
        });
    });
}

function includeParts(parts, robotId) {
    Object.entries(assemblyForm.value).forEach(([key, value]) => {
        if (value === robotId) {
            assemblyForm.value[key] = null;
        }
    });
    parts.forEach(part => {
        const originalId = assemblyForm.value[`${part}_id`];
        Object.entries(assemblyForm.value).forEach(([key, value]) => {
            if (value === originalId) {
                assemblyForm.value[key] = null;
            }
        });
        assemblyForm.value[`${part}_id`] = robotId;
    });
}
function includeRobot(robot, part, exclude=false) {
    if (exclude) {
        if (assemblyForm.value.left_arm_id === robot.id) assemblyForm.value.left_arm_id = null;
        if (assemblyForm.value.right_arm_id === robot.id) assemblyForm.value.right_arm_id = null;
        if (assemblyForm.value.left_tool_id === robot.id) assemblyForm.value.left_tool_id = null;
        if (assemblyForm.value.right_tool_id === robot.id) assemblyForm.value.right_tool_id = null;
        return;
    }
    if (robot.role === 'single_arm') {
        if (part === 'left_arm') includeParts(['left_arm'], robot.id);
        else if (part === 'right_arm') includeParts(['right_arm'], robot.id);
        else if (assemblyForm.value.left_arm_id && assemblyForm.value.right_arm_id) includeParts(['right_arm'], robot.id);
        else if (!assemblyForm.value.left_arm_id) includeParts(['left_arm'], robot.id);
        else if (!assemblyForm.value.right_arm_id) includeParts(['right_arm'], robot.id);
        else includeParts(['left_arm'], robot.id);

        if (robot.tool_inner && assemblyForm.value.left_arm_id === robot.id) includeParts(['left_arm', 'left_tool'], robot.id);
        if (robot.tool_inner && assemblyForm.value.right_arm_id === robot.id) includeParts(['right_arm', 'right_tool'], robot.id);
    } else if (robot.role === 'tool') {
        if (part === 'left_tool') includeParts(['left_tool'], robot.id);
        else if (part === 'right_tool') includeParts(['right_tool'], robot.id);
        else if (assemblyForm.value.left_tool_id && assemblyForm.value.right_tool_id) includeParts(['right_tool'], robot.id);
        else if (!assemblyForm.value.left_tool_id) includeParts(['left_tool'], robot.id);
        else if (!assemblyForm.value.right_tool_id) includeParts(['right_tool'], robot.id);
        else includeParts(['left_tool'], robot.id);
    } else if (robot.role === 'dual_arm') {
        includeParts(['left_arm', 'right_arm'], robot.id);
        if (robot.tool_inner) includeParts(['left_arm', 'right_arm', 'left_tool', 'right_tool'], robot.id);
    }
}

function isIncluded(robot) {
    return assemblyForm.value.left_arm_id === robot.id ||
           assemblyForm.value.right_arm_id === robot.id ||
           assemblyForm.value.left_tool_id === robot.id ||
           assemblyForm.value.right_tool_id === robot.id;
}

const assemblyForm = ref({
    id: props.assembly ? props.assembly.id : null,
    name: props.assembly ? props.assembly.name : '',
    left_arm_id: props.assembly ? props.assembly.left_arm_id : null,
    right_arm_id: props.assembly ? props.assembly.right_arm_id : null,
    left_tool_id: props.assembly ? props.assembly.left_tool_id : null,
    right_tool_id: props.assembly ? props.assembly.right_tool_id : null,
})

// 1. 상태 관리 (Props로 받아도 되지만, 여기선 내부 상태로 구현)
const activeParts = computed(() => {
    const parts = [];
    if (assemblyForm.value.left_arm_id) parts.push('left_arm');
    if (assemblyForm.value.right_arm_id) parts.push('right_arm');
    if (assemblyForm.value.left_tool_id) parts.push('left_tool');
    if (assemblyForm.value.right_tool_id) parts.push('right_tool');
    return parts;
});

function saveAssembly() {
    if (!assemblyForm.value.left_arm_id && !assemblyForm.value.right_arm_id &&
        !assemblyForm.value.left_tool_id && !assemblyForm.value.right_tool_id) {
        Notify.create({
            type: 'negative',
            message: t('assemblyValidationNoPart')
        });
        return;
    }
    if (!assemblyForm.value.name) {
        Notify.create({
            type: 'negative',
            message: t('assemblyValidationNoName')
        });
        return;
    }
    emit('save', assemblyForm.value);
}

// 색상 상수
const COLORS = {
  default: '#d1d5db',
  highlight: '#ef4444',
  outline: '#374151'
};

// 2. 헬퍼 함수들
const getColor = (partName) => {
    if (['head', 'body', ''].includes(partName)) return COLORS.outline;
    return activeParts.value.includes(partName) ? COLORS.highlight : COLORS.default;
};

const setStyle = (partName) => {
  if (!ctx) return;
  ctx.fillStyle = getColor(partName);
  ctx.strokeStyle = COLORS.outline;
  ctx.lineWidth = 2;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
};

// 3. 그리기 함수들 구현
const drawHead = (x, y) => {
  ctx.beginPath();
  ctx.arc(x, y, 30, 0, Math.PI * 2);
  setStyle('head');
  ctx.fill();
  ctx.stroke();
};

const drawBody = (x, y) => {
  const w = 70;
  const h = 110;
  const r = 15;
  
  ctx.beginPath();
  ctx.moveTo(x - w/2 + r, y - h/2);
  ctx.lineTo(x + w/2 - r, y - h/2);
  ctx.quadraticCurveTo(x + w/2, y - h/2, x + w/2, y - h/2 + r);
  ctx.lineTo(x + w/2, y + h/2 - r);
  ctx.quadraticCurveTo(x + w/2, y + h/2, x + w/2 - r, y + h/2);
  ctx.lineTo(x - w/2 + r, y + h/2);
  ctx.quadraticCurveTo(x - w/2, y + h/2, x - w/2, y + h/2 - r);
  ctx.lineTo(x - w/2, y - h/2 + r);
  ctx.quadraticCurveTo(x - w/2, y - h/2, x - w/2 + r, y - h/2);
  
  setStyle('body');
  ctx.fill();
  ctx.stroke();
};

const drawLimb = (startX, startY, dx, dy, partName) => {
  ctx.beginPath();
  ctx.moveTo(startX, startY);
  ctx.lineTo(startX + dx, startY + dy);
  
  ctx.strokeStyle = getColor(partName);
  ctx.lineWidth = 20;
  ctx.lineCap = 'round';
  ctx.stroke();

  // 외곽선 디테일 (선택사항)
  ctx.beginPath();
  ctx.lineWidth = 1;
  ctx.strokeStyle = COLORS.outline;
  // 단순화를 위해 외곽선은 생략하거나 추가 구현 가능
};

const drawHand = (x, y, partName) => {
  ctx.beginPath();
  ctx.arc(x, y, 12, 0, Math.PI * 2);
  setStyle(partName);
  ctx.fill();
  ctx.stroke();
};

// 4. 메인 그리기 로직
const draw = () => {
  if (!canvasRef.value || !ctx) return;
  
  const { width, height } = canvasRef.value;
  ctx.clearRect(0, 0, width, height);

  const cx = 150;
  const cy = 50;

  // 그리기 순서: 머리 -> 팔/다리 -> 몸통 -> 손
  drawHead(cx, cy + 30);
  drawBody(cx, cy + 120);
  drawLimb(cx - 35, cy + 70, -40, 60, 'left_arm');
  drawLimb(cx + 35, cy + 70, 40, 60, 'right_arm');
//   drawLimb(cx - 20, cy + 180, -20, 80, 'legs_default');
//   drawLimb(cx + 20, cy + 180, 20, 80, 'legs_default');
  drawHand(cx - 75, cy + 130, 'left_tool');
  drawHand(cx + 75, cy + 130, 'right_tool');
};

// 5. 라이프사이클 및 감시자
// onMounted 대신 watch를 사용하여 canvasRef가 실제 DOM 엘리먼트가 될 때를 감지합니다.
watch(canvasRef, (newCanvasElement) => {
  if (newCanvasElement) {
    ctx = newCanvasElement.getContext('2d');
    draw();
  }
});

// activeParts 배열이 변경될 때마다 자동으로 다시 그리기
watch(activeParts, () => {
  draw();
}, { deep: true });

onMounted(() => {
    listRobots();
});

</script>