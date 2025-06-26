const routes = [
  {
    path: '/',
    component: () => import('layouts/MainLayout.vue'),
    children: [
      { path: '', component: () => import('pages/IndexPage.vue') },
      { path: 'sensors', component: () => import('src/pages/SensorPage.vue') },
      { path: 'robots', component: () => import('pages/RobotPage.vue') },
      { path: 'tasks', component: () => import('pages/TaskPage.vue') }
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
