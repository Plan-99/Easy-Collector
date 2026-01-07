<template>
    <div class="row q-gutter-x-md">
        <div class="col">
            <div v-for="(j, i) in props.robot.joint_names" :key="i" class="q-gutter-x-md text-white q-mb-sm">
                <div class="border-white q-px-md q-py-xs text-center row flex flex-center">
                    <q-btn
                        dense
                        flat
                        color="white"
                        icon="remove"
                        @click="moveOneJoint(i, -robotStepSize)"
                    ></q-btn>
                    <q-space></q-space>
                    <div class="q-mx-sm">
                        {{ j }}
                    </div>
                    <div class="q-mx-sm">
                        {{ props.robot.jointState ? props.robot.jointState[i]?.toFixed(4) : 'Unreadable' }}
                    </div>
                    <q-space></q-space>
                    <q-btn
                        dense
                        flat
                        color="white"
                        icon="add"
                        @click="moveOneJoint(i, robotStepSize)"
                    ></q-btn>
                </div>
            </div>
        </div>
        <div class="col" v-if="props.robot.ik_available">
            <div v-for="(r, i) in (props.robot.role === 'dual_arm' ? ['left', 'right'] : [''])" :key="i">
                <div v-for="(p, j) in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']" :key="j" class="q-gutter-x-md text-primary q-mb-sm">
                    <div class="border-primary q-px-md q-py-xs text-center row flex flex-center">
                        <q-btn
                            dense
                            flat
                            color="primary"
                            icon="remove"
                            @click="moveOneEE(i, j, -robotStepSize)"
                        ></q-btn>
                        <q-space></q-space>
                        <div class="q-mx-sm">
                            {{ r }} {{ p }}
                        </div>
                        <div class="q-mx-sm">
                            {{ props.robot.eePos && Object.values(props.robot.eePos).length ? Object.values(props.robot.eePos)[i][j]?.toFixed(4) : 'Unreadable' }}
                        </div>
                        <q-space></q-space>
                        <q-btn
                            dense
                            flat
                            color="primary"
                            icon="add"
                            @click="moveOneEE(i, j, +robotStepSize)"
                        ></q-btn>
                    </div>
                </div>
            </div>
        </div>
    </div>
    
</template>
<script setup>
import { defineProps, ref } from 'vue';
const props = defineProps({
    robot: {
        type: Object,
        required: true
    }
});

function moveOneJoint(index, delta) {
    const jointState = props.robot.jointState;
    jointState[index] += delta;
    props.robot.handler.moveRobotJoint(jointState);
}

function moveOneEE(tool_index, p_index, delta) {
    const eePos = Object.values(props.robot.eePos)[tool_index]
    eePos[p_index] += delta;
    const eePosDict = {};
    if (props.robot.role === 'dual_arm') {
        eePosDict['left'] = tool_index === 0 ? eePos : Object.values(props.robot.eePos)[0];
        eePosDict['right'] = tool_index === 1 ? eePos : Object.values(props.robot.eePos)[1];
    } else {
        eePosDict['ee'] = eePos;
    }
    props.robot.handler.moveRobotEE(eePosDict);
}

const robotStepSize = ref(0.01);

</script>