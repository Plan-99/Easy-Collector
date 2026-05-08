import { defineStore } from 'pinia';
import { computed, ref } from 'vue';
import { api } from 'boot/axios';

export const useModulesStore = defineStore('modules', () => {
    const modules = ref([]);
    const initialized = ref(false);

    async function fetch() {
        try {
            const res = await api.get('/modules/installed');
            modules.value = res.data?.modules || [];
        } catch (err) {
            console.error('Failed to fetch installed modules:', err);
            modules.value = [];
        } finally {
            initialized.value = true;
        }
    }

    async function initialize() {
        if (initialized.value) return;
        await fetch();
    }

    const idSet = computed(() => new Set(modules.value.map((m) => m.id)));

    function has(id) {
        return idSet.value.has(id);
    }

    function get(id) {
        return modules.value.find((m) => m.id === id) || null;
    }

    return { modules, initialized, fetch, initialize, has, get };
});
