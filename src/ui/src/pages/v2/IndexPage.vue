<template>
  <q-page class="flex flex-center">
    <img
      alt="Quasar logo"
      src="~assets/quasar-logo-vertical.svg"
      style="width: 200px; height: 200px"
      @click="hello"
    >
  </q-page>
</template>

<script setup>
function hello() {
  eel.hello_world()
}
</script>
