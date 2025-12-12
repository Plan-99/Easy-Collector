<template>
    <q-page class="q-pt-lg q-pr-lg full-height">
        <div class="border-rounded bg-secondary q-pa-lg q-mb-lg row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl"></q-img>
            <div>
                <div class="text-h5 text-primary text-bold q-mb-lg">{{ $t('sensorIntroTitle') }}</div>
                <div class="text-body text-white">{{ $t('sensorIntroBody') }}</div>
                <div class="text-body text-white">{{ $t('sensorIntroBody2') }}</div>
            </div>
        </div>
        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2"  v-for="sensor in sensors" :key="sensor.id">
                <q-card class="q-pa-md bg-secondary border-rounded border-white text-white" :class="watchingSensor && sensor.id === watchingSensor.id ? 'border-primary' : ''">
                    <q-img :src="sensor.image" @click="watchSensor(sensor)" class="cursor-pointer bg-white" ratio="2.5" fit="contain">
                        <q-menu context-menu>
                            <q-list bordered separator>
                                <q-item clickable v-ripple v-close-popup @click="openEditSensorForm(sensor)">
                                    <q-item-section>Edit Sensor</q-item-section>
                                    <q-item-section side>
                                        <q-icon name="edit" size="xs" />
                                    </q-item-section>
                                </q-item>
                                <q-item clickable v-ripple class="text-negative" @click="deleteSensor(sensor)">
                                    <q-item-section>Hide Sensor</q-item-section>
                                    <q-item-section side>
                                        <q-icon color="negative" name="visibility" size="xs" />
                                    </q-item-section>
                                </q-item>
                            </q-list>
                        </q-menu>
                    </q-img>


                    <q-card-section class="q-pa-none q-mt-sm">
                        <div class="text-bold">{{ sensor.name }}</div>
                        <div class="text-grey-6 text-caption" v-if="sensor.serial_no">Serial No. {{ sensor.serial_no }}</div>
                    </q-card-section>
                    <q-card-section class="q-pa-none q-mt-sm row">
                        <div class="text-primary text-caption" v-if="sensor.status === 'on'">ONLINE</div>
                        <div class="text-orange text-caption" v-if="sensor.status === 'loading'">LOADING</div>
                        <div class="text-grey-7 text-caption" v-if="sensor.status === 'off'">OFFLINE</div>
                        <q-space></q-space>
                        <q-toggle
                            :model-value="sensor.status === 'on'"
                            color="primary"
                            dense
                            @click.stop="toggleSensor(sensor)"
                        />
                    </q-card-section>
                    <q-inner-loading :showing="sensor.status === 'loading'">
                        <q-spinner-gears size="50px" color="primary" />
                    </q-inner-loading>
                </q-card>
            </div>

            <div class="col" v-if="!sensors.length">
                <q-card class="full-height border-rounded border-white bg-dark text-white" flat bordered>
                    <q-card-section class="text-center">
                        <div class="text-h6">{{ $t('noSensorTitle') }}</div>
                        <div class="text-subtitle2">{{ $t('noSensorBody') }}</div>
                    </q-card-section>
                </q-card>
            </div>
            
            <div class="col-6 col-sm-4 col-md-3  col-lg-2" style="min-height: 220px;" >
                <q-btn color="white" class="full-height full-width border-rounded" outline size="lg" icon="add" @click="openAddSensorForm"></q-btn>
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
                <div class="row q-gutter-x-md">
                    <web-rtc-video
                        :process-id="`sensor_${sensor.id}`"
                        :topic="sensor.topic"
                        ref="sensorVideo"
                        :loading="sensor.status !== 'on'"
                        style="height: 300px"
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
        <form-dialog
            v-model="showSensorForm"
            :title="$t(sensorForm.find((e) => e.key === 'id').value ? 'sensorEditFormTitle' : 'sensorAddFormTitle')"
            :form="sensorForm"
            @submit="saveSensor"
            :ok-button-label="$t(sensorForm.find((e) => e.key === 'id').value ? 'save' : 'add')"
        ></form-dialog>
    </q-page>
</template>

<script setup>
import { onMounted, ref} from 'vue';

import { api } from 'src/boot/axios';
import ProcessConsole from 'src/components/v2/ProcessConsole.vue';
import { Notify } from 'quasar';
import BottomTerminal from 'src/components/v2/BottomTerminal.vue';
import WebRtcVideo from 'src/components/v2/WebRtcVideo.vue';
import { useSensor } from '../../composables/useSensor';
import FormDialog from 'src/components/v2/FormDialog.vue';

import { useI18n } from 'vue-i18n'
const { t } = useI18n()


const sensors = ref([]);

const sensorForm = ref([
    { key: 'id', value: null },
    { key: 'name', label: t('sensorName'), value: '', default: '', type: 'text' },
    { key: 'type', label: t('sensorType'), value: '', default: '', type: 'select', options: [
        { label: 'Realsense Camera', value: 'realsense_camera' }
    ] },
    { key: 'serial_no', label: t('serialNUmber'), value: '', default: '', type: 'text', show: (form) => form.find(f => f.key === 'type').value === 'realsense_camera' }
]);
const showSensorForm = ref(false);
const watchingSensor = ref(null);

// const sensorTypeOptions = [
//     { label: 'Realsense Camera', value: 'realsense_camera' }
// ]

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

function openAddSensorForm() {
    sensorForm.value.forEach(field => {
        field.value = field.default;
    });
    showSensorForm.value = true;
}

function openEditSensorForm(sensor) {
    sensorForm.value.forEach(field => {
        field.value = sensor[field.key] || field.default;
    });
    sensorForm.value.find((e) => e.key === 'id').value = sensor.id; // Set ID for edit
    showSensorForm.value = true;
}   

function saveSensor(formData) {
    if (sensorForm.value.find((e) => e.key === 'id').value) {
        return api.put(`/sensor/${formData.id}`, formData).then(() => {
            sensorForm.value.forEach(field => field.value = field.default); // Reset form fields
            listSensors()
        })
    }
    return api.post(`/sensor`, formData).then(() => {
        sensorForm.value.forEach(field => field.value = field.default); // Reset form fields
        listSensors()
    })
}

function deleteSensor(sensor) {
    if (sensor.status === 'on') {
        Notify.create({
            color: 'negative',
            message: 'Turn off the sensor first.'
        })
        return;
    }
    return api.delete(`/sensor/${sensor.id}`).then(() => {
        listSensors()
    })
}

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


onMounted(() => {
    listSensors()
})


</script>
