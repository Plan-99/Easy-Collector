<template>
    <q-page>
      <div class="row">
        <q-tabs
          dense
          class="text-grey-7 bg-blue-2 full-width"
          active-color="primary"
          indicator-color="primary"
          align="justify"
          narrow-indicator
        >
          <!-- <q-tab name="overview" label="Overview" :to="`/tasks/${$route.params.taskId}/overview`" /> -->
          <q-route-tab name="execute" label="Execute" to="/vla/execute" />
          <q-route-tab name="finetunning" label="Fine tunning" :to="`/vla/finetunning`" />
        </q-tabs>
      </div>
      
      <router-view />
    </q-page>
</template>
  
<script setup>

</script>
  