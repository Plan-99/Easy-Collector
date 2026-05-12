<template>
    <q-page class="q-pt-md q-pr-md full-height">
        <div class="border-rounded bg-secondary q-pa-md q-mb-md row">
            <q-img src="images/robot1.png" style="width: 100px" class="q-mr-xl"></q-img>
            <div>
                <div class="text-h5 text-primary text-bold q-mb-md">{{ $t('assembleIntroTitle') }}</div>
                <div class="text-body text-white">{{ $t('assembleIntroBody') }}</div>
                <div class="text-body text-white">{{ $t('assembleIntroBody2') }}</div>
            </div>
        </div>

        <TutorialHint class="q-mb-md" :text="$t('tutorialAssembleIntro')" />

        <div class="row q-col-gutter-md">
            <div class="col-6 col-sm-4 col-md-3 col-lg-2" v-for="assembly in assemblies" :key="assembly.id">
                <q-card class="q-pa-md bg-secondary border-rounded border-white text-white cursor-pointer" @click="watchingAssembly = assembly"
                    :class="{ 'border-primary': watchingAssembly && watchingAssembly.id === assembly.id }"
                >
                    <q-card-section class="q-pa-none">
                        <div class="text-bold text-h6">{{ assembly.name }}</div>
                    </q-card-section>
                    <q-card-section class="q-pa-none q-mt-md">
                        <div v-for="part in ['left_arm', 'left_tool', 'right_arm', 'right_tool']" :key="part">{{ part }} <span class="text-primary">{{ robots.find(e => e.id === assembly[part + '_id'])?.name }}</span></div>
                    </q-card-section>
                    <q-inner-loading :showing="assembly.status === 'loading'">
                        <q-spinner-gears size="50px" color="primary" />
                    </q-inner-loading>
                    <q-menu context-menu>
                            <q-list bordered separator>
                                <q-item clickable v-ripple class="text-negative" @click="deleteAssembly(assembly)">
                                    <q-item-section>{{ $t('assemblyContextHide') }}</q-item-section>
                                    <q-item-section side>
                                        <q-icon color="negative" name="visibility" size="xs" />
                                    </q-item-section>
                                </q-item>
                            </q-list>
                        </q-menu>
                </q-card>
            </div>
        </div>

        <bottom-terminal
            v-model="watchingAssembly"
            :tabs="[...assemblies, { name: $t('assemblyTabNew'), id: 'new' }]"
            tab-label="name"
            tab-value="id"
            no-close
        >
            <template #new>
                <TutorialHint class="q-mb-sm" :text="$t('tutorialAssembleNew')" />
                <assembly-form
                    :assembly="null"
                    @save="saveAssembly"
                ></assembly-form>
            </template>
            <template v-slot:[assembly.id] v-for="assembly in assemblies" :key="assembly.id">
                <assembly-form
                    :assembly="assembly"
                    @save="saveAssembly"
                ></assembly-form>
            </template>
        </bottom-terminal>

    </q-page>
</template>
<script setup>
import { ref, onMounted } from 'vue';
import { api } from 'src/boot/axios';
import AssemblyForm from 'src/components/v2/AssemblyForm.vue';
import BottomTerminal from 'src/components/v2/BottomTerminal.vue';
import TutorialHint from 'src/components/v2/TutorialHint.vue';
import { useI18n } from 'vue-i18n';

const { t } = useI18n();

const assemblies = ref([]);
// const watchingAssembly = ref(null);

function listAssemblies() {
    api.get('/assemblies').then((response) => {
        assemblies.value = response.data.assemblies;
    });
}

const robots = ref([]);
function listRobots() {
    api.get('/robots').then((response) => {
        robots.value = response.data.robots;
    });
}

const watchingAssembly = ref({
    name: t('assemblyTabNew'),
    id: 'new',
});

function saveAssembly(assemblyForm) {
    // If assembly has id, it's an edit
    if (assemblyForm.id) {
        api.put(`/assembly/${assemblyForm.id}`, assemblyForm).then(() => {
            listAssemblies();
        });
    } else {
        api.post('/assembly', assemblyForm).then(() => {
            listAssemblies();
        });
    }
}

function deleteAssembly(assembly) {
    api.delete(`/assembly/${assembly.id}`).then(() => {
        listAssemblies();
        if (watchingAssembly.value && watchingAssembly.value.id === assembly.id) {
            watchingAssembly.value = { name: t('assemblyTabNew'), id: 'new' };
        }
    });
}

onMounted(() => {
    listAssemblies();
    listRobots();
});

</script> 