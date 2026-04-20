<template>
    <div class="row">
        <q-space></q-space>
        <q-input
            dense
            outlined
            dark
            bg-color="dark"
            v-model.number="robotStepSize"
            type="number"
            label="Step Size"
            class="q-mb-md q-mt-xs col-2"
        ></q-input>
    </div>
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
                <div v-for="(p, j) in ['x', 'y', 'z', 'ax', 'ay', 'az']" :key="j" class="q-gutter-x-md text-primary q-mb-sm">
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
    // const jointState = props.robot.jointState;
    // jointState[index] += delta;
    // props.robot.handler.moveRobotJoint(jointState);
    if (props.robot.role === 'tool') {
        delta = 0.12 * (delta > 0 ? 1 : -1); // tool
    }
    const deltaAction = new Array(props.robot.joint_names.length).fill(0);
    deltaAction[index] = delta;
    props.robot.handler.moveRobotJointDelta(deltaAction);
}

function moveOneEE(tool_index, p_index, delta) {
    // const eePos = Object.values(props.robot.eePos)[tool_index]
    // eePos[p_index] += delta;
    // const eePosDict = {};
    // if (props.robot.role === 'dual_arm') {
    //     eePosDict['left'] = tool_index === 0 ? eePos : Object.values(props.robot.eePos)[0];
    //     eePosDict['right'] = tool_index === 1 ? eePos : Object.values(props.robot.eePos)[1];
    // } else {
    //     eePosDict['ee'] = eePos;
    // }
    // props.robot.handler.moveRobotEE(eePosDict);
    if (p_index < 3) {
        delta = delta * 1; // x, y, z
    } else if (p_index < 6) {
        delta = delta * 4; // roll, pitch, yaw in radians
    } else {
        delta = 0.12 * (delta > 0 ? 1 : -1); // tool
    }
    const deltaPos = {};
    const toolName = props.robot.role === 'dual_arm' ? (tool_index === 0 ? 'L_ee' : 'R_ee') : 'ee';
    deltaPos[toolName] = Array(props.robot.tool_inner ? 7 : 6).fill(0); // x, y, z, roll, pitch, yaw, tool
    deltaPos[toolName][p_index] = delta;
    props.robot.handler.moveRobotEEDelta(deltaPos);
}

const robotStepSize = ref(0.006);

</script>