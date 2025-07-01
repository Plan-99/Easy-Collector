<template>
    <q-page class="q-pa-md full-height">
        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2"  v-for="sensor in sensors" :key="sensor.id">
                <q-card>
                    <q-img :src="sensor.image" @click="watchSensor(sensor)" class="cursor-pointer" ratio="1">
                        <div class="absolute-bottom text-h6 row q-gutter-x-sm">
                            <div>{{ sensor.name }}</div>
                            <q-space></q-space>
                            <q-icon v-if="sensor.status === 'on' && watchingSensor === sensor" color="positive" name="visibility" size="sm" class="cursor-pointer"></q-icon>
                            <q-icon v-if="sensor.status === 'on'" color="positive" name="power_settings_new" size="sm" class="cursor-pointer" @click.stop="toggleSensor(sensor)"></q-icon>
                            <q-icon v-if="sensor.status === 'off'" name="power_settings_new" size="sm" class="cursor-pointer" @click.stop="toggleSensor(sensor)"></q-icon>
                            <q-icon v-if="sensor.status === 'loading'" color="orange-6" name="power_settings_new" size="sm" class="cursor-pointer"></q-icon>
                        </div>
                        <div class="absolute-top-right row q-gutter-x-sm" style="background: none;">
                            <q-btn-dropdown
                                dropdown-icon="more_vert"
                                flat
                                text-color="dark"
                                round
                            >
                                <q-list bordered separator>
                                    <q-item clickable v-ripple v-close-popup @click="showSensorForm = true; sensorForm = sensor">
                                        <q-item-section>Edit Sensor</q-item-section>
                                        <q-item-section side>
                                            <q-icon name="edit" size="xs" />
                                        </q-item-section>
                                    </q-item>
                                    <q-item clickable v-ripple class="text-negative" @click="deleteSensor(sensor)">
                                        <q-item-section>Delete Sensor</q-item-section>
                                        <q-item-section side>
                                            <q-icon color="negative" name="delete" size="xs" />
                                        </q-item-section>
                                    </q-item>
                                </q-list>
                            </q-btn-dropdown>
                        </div>
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
        <div class="absolute-bottom bg-grey-4"  v-if="watchingSensor">
            <q-separator />
            <div class="q-pa-md row q-gutter-x-md">
                <video ref="sensorVideo"
                    class="col-4"
                    autoplay playsinline muted 
                    style="background-color: black; height: 360px;"
                >
                </video>
                <div class="col">
                    <div style="height: 30px" class="row">
                        <div 
                            class="bg-dark col text-white text-center" 
                            v-for="sensor in sensors.filter((e) => e.status !== 'off')"
                            :key="sensor.id"
                            :style="sensor.id !== watchingSensor.id ? 'border: 1px solid #ffffff' : ''"
                            :class="sensor.id !== watchingSensor.id ? 'cursor-pointer': ''"
                            @click="watchSensor(sensor)"
                        >{{ sensor.name }}</div>
                    </div>
                    <process-console 
                        :process="sensor.process_id" 
                        v-for="sensor in sensors.filter((e) => e.status !== 'off')"
                        :key="sensor.id"
                        v-show="sensor.id === watchingSensor.id"
                    />
                </div>
            </div>
        </div>
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
import { onMounted, onUnmounted, ref } from 'vue';

import { useSocket } from '../composables/useSocket';
import { useWebRTC } from '../composables/useWebRTC';
import { api } from 'src/boot/axios';
import ProcessConsole from 'src/components/ProcessConsole.vue';
import { Notify } from 'quasar';
const { socket } = useSocket();
const { connect } = useWebRTC(socket);

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
            sensor.status = 'off'; // Initialize sensor state
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
            listSensors().then(() => {
                listProcesses();
            })
        })
    }
}

function deleteSensor(sensor) {
    return api.delete(`/sensor/${sensor.id}`).then(() => {
        listSensors().then(() => {
            listProcesses();
        })
    })
}

function listProcesses() {
    return api.get('/processes').then((response) => {
        const processes = response.data.processes || [];
        sensors.value.forEach(sensor => {
            const process = processes.find(p => p === sensor.process_id);
            if (process) {
                sensor.status = 'on'; // Sensor is running
                sensor.process = process;
                if (!watchingSensor.value) {
                    watchSensor(sensor)
                }
            } else {
                sensor.status = 'off'; // Sensor is not running
            }
        });
    }).catch((error) => {
        console.error('Error fetching processes:', error);
    });
}

function toggleSensor(sensor) {
    if (sensor.status === 'on') {
        stopSensor(sensor)
    } else {
        startSensor(sensor)
    }
}

function startSensor(sensor) {
    sensor.status = 'loading';
    return api.post('/sensor:start', sensor).catch((error) => {
        console.error('Error starting sensor:', error);
        sensor.status = 'off'; // Reset status on error
    });
}

function stopSensor(sensor) {
    sensor.status = 'loading';
    return api.post('/sensor:stop', sensor).catch((error) => {
        console.error('Error stopping sensor:', error);
        sensor.status = 'on'; // Reset status on error
    });
}

const sensorVideo = ref({});
function watchSensor(sensor) {
    if (sensor.status === 'off') {
        return;
    }
    watchingSensor.value = sensor; // Start watching the selected sensor
    connect(sensor, (event) => {
        const newStream = new MediaStream();
        newStream.addTrack(event.track); // 현재 track 추가
        sensorVideo.value.srcObject = newStream;
    });
}

const showSensorForm = ref(false)

onMounted(() => {

    socket.on('start_process', (data) => {
        const sensor = sensors.value.find(s => s.process_id === data.id);
        if (sensor) {
            sensor.status = 'on';
            watchSensor(sensor)
        }
    });

    socket.on('stop_process', (data) => {
        const sensor = sensors.value.find(s => s.process_id === data.id);
        if (sensor) {
            sensor.status = 'off';
            if (watchingSensor.value && watchingSensor.value.id === sensor.id && sensors.value.find((e) => e.status === 'on')) {
                watchSensor(sensors.value.find((e) => e.status === 'on'))
            } else {
                watchingSensor.value = null
            }
        }
    });

    listSensors().then(() => {
        listProcesses();
    })
})

onUnmounted(() => {
    socket.off('start_process');
});

</script>
