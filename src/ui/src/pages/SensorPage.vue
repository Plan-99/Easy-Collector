<template>
    <q-page class="q-pa-md full-height">
        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2"  v-for="sensor in sensors" :key="sensor.id">
                <q-card>
                    <q-img :src="sensor.image" @click="watchSensor(sensor)" class="cursor-pointer" ratio="1">
                        <div class="absolute-bottom text-h6 row q-gutter-x-sm">
                            <div>{{ sensor.name }}</div>
                            <q-space></q-space>
                            <q-icon v-if="sensor.status === 'on' && watchingSensor && watchingSensor.id === sensor.id" color="positive" name="visibility" size="sm" class="cursor-pointer"></q-icon>
                            <q-icon v-if="sensor.status === 'on'" color="positive" name="power_settings_new" size="sm" class="cursor-pointer" @click.stop="toggleSensor(sensor)"></q-icon>
                            <q-icon v-if="sensor.status === 'off'" name="power_settings_new" size="sm" class="cursor-pointer" @click.stop="toggleSensor(sensor)"></q-icon>
                            <q-icon v-if="sensor.status === 'loading'" color="orange-6" name="power_settings_new" size="sm" class="cursor-pointer"></q-icon>
                        </div>
                        <q-menu context-menu>
                            <q-list bordered separator>
                                <q-item clickable v-ripple v-close-popup @click="showSensorForm = true; sensorForm = sensor">
                                    <q-item-section>Edit Sensor</q-item-section>
                                    <q-item-section side>
                                        <q-icon name="edit" size="xs" />
                                    </q-item-section>
                                </q-item>
                                <!-- <q-item clickable v-ripple class="text-negative" @click="deleteSensor(sensor)">
                                    <q-item-section>Delete Sensor</q-item-section>
                                    <q-item-section side>
                                        <q-icon color="negative" name="delete" size="xs" />
                                    </q-item-section>
                                </q-item> -->
                            </q-list>
                        </q-menu>
                    </q-img>


                    <q-card-section>
                    <div class="text-grey-6 text-caption">Serial No.</div>
                    <div class="row">
                        <div>{{ sensor.serial_no }}</div>
                        <q-space></q-space>
                        <!-- <q-icon name="edit" class="cursor-pointer" clickable @click="sensorForm = sensor" v-if="sensor.status === 'off'"></q-icon> -->
                    </div>
                    <!-- <div v-else>
                        <q-input v-model="sensorForm.serial_no" outlined dense @blur="editSensor"></q-input>
                    </div> -->
                    </q-card-section>
                </q-card>
            </div>
            
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" style="min-height: 220px;">
                <q-btn color="grey-8" class="full-height full-width" outline size="lg" icon="add" @click="showSensorForm = true"></q-btn>
            </div>
        </div>
        <bottom-terminal
            :tabs="sensors.filter((e) => e.status !== 'off')"
            tab-label="name"
            tab-value="id"
            v-model="watchingSensor"
            v-if="sensors.filter((e) => e.status !== 'off').length > 0 && watchingSensor"
            @update:model-value="watchSensor($event)"
        >
            <template v-for="sensor in sensors.filter((e) => e.status !== 'off')" :key="sensor.id" v-slot:[sensor.id]>
                <div class="q-pa-md row q-gutter-x-md">
                    <web-rtc-video
                        :process-id="`sensor_${sensor.id}`"
                        :topic="sensor.topic"
                        ref="sensorVideo"
                        style="height: 330px;"
                        :loading="sensor.status !== 'on'"
                    ></web-rtc-video>
                    <div class="col">
                        <process-console 
                            :process="sensor.process_id" 
                            :key="sensor.id"
                        />
                    </div>
                </div>
            </template>
        </bottom-terminal>
        <q-dialog v-model="showSensorForm">
            <q-card style="min-width: 350px">
                <q-card-section class="q-pt-none">
                    <q-card-section>
                        <div class="text-h6 text-center">Sensor</div>
                    </q-card-section>
                    <q-input
                        dense
                        v-model="sensorForm.name"
                        label="Sensor Name"
                        autofocus
                        class="q-mb-md"
                    />

                    <q-select
                        dense
                        v-model="sensorForm.type"
                        :options="sensorTypeOptions"
                        label="Sensor Type"
                        class="q-mb-md"
                        map-options
                        emit-value
                    />

                    <q-input
                        dense
                        v-model="sensorForm.serial_no"
                        label="Serial Number"
                    />
                </q-card-section>

                <q-card-actions align="center" class="text-primary">
                    <q-btn flat label="Save" v-close-popup @click="saveSensor" />
                    <q-btn flat color="grey-7" label="Close" v-close-popup @click="sensorForm = {}" />
                </q-card-actions>
            </q-card>
        </q-dialog>
    </q-page>
</template>

<script setup>
import { onMounted, ref } from 'vue';

import { api } from 'src/boot/axios';
import ProcessConsole from 'src/components/ProcessConsole.vue';
import { Notify } from 'quasar';
import BottomTerminal from 'src/components/BottomTerminal.vue';
import WebRtcVideo from 'src/components/WebRtcVideo.vue';
import { useSensor } from '../composables/useSensor'; 


const sensors = ref([]);

const sensorForm = ref({});
const watchingSensor = ref(null);

const sensorTypeOptions = [
    { label: 'Realsense Camera', value: 'realsense_camera' }
]

function listSensors() {                                                                                                     
    return api.get('/sensors').then((response) => {
        sensors.value = response.data.sensors || [];
        sensors.value.forEach(sensor => {
            sensor.image = '/images/' + sensor.type + '.png'; // Default image if not provided
            sensor.handler = useSensor(sensor, () => {
                watchSensor(sensor);
            });
        });
    }).catch((error) => {
        console.error('Error fetching sensors:', error);
    });
}

function saveSensor() {
    if (!sensorForm.value.name || !sensorForm.value.type || !sensorForm.value.serial_no) {
        Notify.create({
            color: 'negative',
            message: 'Please fill the form'
        })
        return;
    }
    if (sensorForm.value.id) {
        return api.put(`/sensor/${sensorForm.value.id}`, {
            'serial_no': sensorForm.value.serial_no,
            'name': sensorForm.value.name,
            'type': sensorForm.value.type
        }).then(() => {
            sensorForm.value = {};
        })
    } else {
        return api.post(`/sensor`, {
            'serial_no': sensorForm.value.serial_no,
            'name': sensorForm.value.name,
            'type': sensorForm.value.type
        }).then(() => {
            sensorForm.value = {};
            listSensors()
        })
    }
}

// function deleteSensor(sensor) {
//     if (sensor.status === 'on') {
//         Notify.create({
//             color: 'negative',
//             message: 'Turn off the sensor first.'
//         })
//         return;
//     }
//     return api.delete(`/sensor/${sensor.id}`).then(() => {
//         listSensors()
//     })
// }

function toggleSensor(sensor) {
    if (sensor.status === 'on') {
        sensor.handler.stopSensor().then(() => {
            watchingSensor.value = null; // Stop watching if sensor is stopped
        })
    } else {
        sensor.handler.startSensor().then(() => {
            watchSensor(sensor); // Start watching the sensor after it is started
        })
    }
}


const sensorVideo = ref({});
function watchSensor(sensor) {
    if (sensor.status === 'off') {
        return;
    }
    watchingSensor.value = sensor; // Start watching the selected sensor
}

const showSensorForm = ref(false)

onMounted(() => {
    listSensors()
})


</script>
