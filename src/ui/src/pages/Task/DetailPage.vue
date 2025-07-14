<template>
  <q-page>
    <div class="row">
      <q-breadcrumbs class="col-3 q-pa-sm q-pl-md" style="font-size: 1.2rem" separator=">">
        <q-breadcrumbs-el label="Task" to="/tasks" />
        <q-breadcrumbs-el :label="`${task ? task.name : 'Loading...' } ${task ? task.image : ''}`" />
      </q-breadcrumbs>
      <q-tabs
        v-model="tab"
        dense
        class="text-grey-7 bg-blue-2 col-9"
        active-color="primary"
        indicator-color="primary"
        align="justify"
        narrow-indicator
      >
        <!-- <q-tab name="overview" label="Overview" :to="`/tasks/${$route.params.taskId}/overview`" /> -->
        <q-route-tab name="data_collection" label="Data Collection" :to="`/tasks/${$route.params.id}/data_collection`" />
        <q-route-tab name="train" label="Train" :to="`/tasks/${$route.params.id}/train`" />
        <q-route-tab name="test" label="Test" :to="`/tasks/${$route.params.id}/test`" />
      </q-tabs>
    </div>
    
    <router-view />
  </q-page>
</template>

<script setup>
import { onMounted, ref } from 'vue'
import { api } from 'boot/axios'
import { useRoute } from 'vue-router' 

const $route = useRoute()

const task = ref(null);
function getTask() {
  const taskId = $route.params.id
  return api.get(`/tasks/${taskId}`)
    .then((res) => {
      task.value = res.data.task
    })
    .catch(error => {
      console.error('Error fetching task:', error)
      throw error
    })
} 

onMounted(() => {
  getTask()
})

const tab = ref('overview')

</script>
