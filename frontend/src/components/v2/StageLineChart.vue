<template>
  <Line :data="chartData" :options="chartOptions" />
</template>

<script setup>
import { computed } from 'vue'
import { Line } from 'vue-chartjs'
import {
  Chart as ChartJS, Title, Tooltip, Legend,
  PointElement, LineElement, CategoryScale, LinearScale,
} from 'chart.js'

ChartJS.register(Title, Tooltip, Legend, PointElement, LineElement, CategoryScale, LinearScale)

const props = defineProps({
  labels: { type: Array, default: () => [] },
  values: { type: Array, default: () => [] },
  label: { type: String, default: '' },
  color: { type: String, default: '#26A69A' },
})

const chartData = computed(() => ({
  labels: props.labels,
  datasets: [{
    label: props.label,
    data: props.values,
    borderColor: props.color,
    backgroundColor: props.color,
    tension: 0.3,
    pointRadius: 3,
  }],
}))

const chartOptions = {
  responsive: true,
  maintainAspectRatio: false,
  plugins: { legend: { labels: { color: '#ccc' } } },
  scales: {
    x: { ticks: { color: '#aaa' }, grid: { color: 'rgba(255,255,255,0.08)' } },
    y: { ticks: { color: '#aaa' }, grid: { color: 'rgba(255,255,255,0.08)' } },
  },
}
</script>
