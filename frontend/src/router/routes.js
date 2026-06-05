const routes = [
  {
    path: '/',
    component: () => import('layouts/v2/MainLayout.vue'),
    children: [
      { path: '', redirect: '/sensors' },
      { path: 'sensors', component: () => import('src/pages/v2/SensorPage.vue') },
      { path: 'robots/management', component: () => import('src/pages/v2/RobotPage.vue') },
      { path: 'robots/assemble', component: () => import('src/pages/v2/AssemblePage.vue') },
      { path: 'robots/teleoperation', component: () => import('src/pages/v2/TeleoperationPage.vue') },
      { path: 'workspace', component: () => import('src/pages/v2/WorkspacePage.vue') },
      { path: 'train', component: () => import('src/pages/v2/TrainPage.vue') },
      { path: 'planner', component: () => import('src/pages/v2/PlannerPage.vue') },
      { path: 'curriculum', component: () => import('src/pages/v2/CurriculumPage.vue') },
      { path: 'datasets', component: () => import('pages/v2/DatasetPage.vue') }
    ]
  },
  // Always leave this as last one,
  // but you can also remove it
  {
    path: '/:catchAll(.*)*',
    component: () => import('pages/ErrorNotFound.vue')
  }
]

export default routes
