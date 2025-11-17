const routes = [
  {
    path: '/',
    component: () => import('layouts/v2/MainLayout.vue'),
    children: [
      { path: '', component: () => import('pages/v2/IndexPage.vue') },
      { path: 'sensors', component: () => import('src/pages/v2/SensorPage.vue') },
      { path: 'robots', component: () => import('src/pages/v2/RobotPage.vue') },
      { path: 'workspace', component: () => import('src/pages/v2/WorkspacePage.vue') },
      { path: 'train', component: () => import('src/pages/v2/TrainPage.vue') },
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
