const routes = [
  {
    path: '/',
    component: () => import('layouts/MainLayout.vue'),
    children: [
      { path: '', component: () => import('pages/IndexPage.vue') },
      { path: 'sensors', component: () => import('src/pages/SensorPage.vue') },
      { path: 'robots', component: () => import('pages/RobotPage.vue') },
      { path: 'tasks', component: () => import('src/pages/Task/ListPage.vue') },
      { path: 'tasks/:id', component: () => import('src/pages/Task/DetailPage.vue'),
        children: [
          // { path: 'overview', component: () => import('src/pages/Task/OverviewPage.vue') },
          { path: 'data_collection', component: () => import('src/pages/Task/DataCollectionPage.vue') },
          { path: 'train', component: () => import('src/pages/Task/TrainPage.vue') },
          { path: 'test', component: () => import('src/pages/Task/TestPage.vue') },
        ]
      },
      { path: 'vla', component: () => import('src/pages/VLA/MainPage.vue'),
        children: [
          { path: 'execute', component: () => import('src/pages/VLA/ExecutePage.vue') },
        ],
      },
      { path: 'datasets', component: () => import('pages/DatasetPage.vue') }
    ]
  },
  {
    path: '/v2/',
    component: () => import('layouts/v2/MainLayout.vue'),
    children: [
      { path: '', component: () => import('pages/v2/IndexPage.vue') },
      { path: 'sensors', component: () => import('src/pages/v2/SensorPage.vue') },
      { path: 'robots', component: () => import('src/pages/v2/RobotPage.vue') },
      { path: 'tasks', component: () => import('src/pages/v2/Task/ListPage.vue') },
      { path: 'tasks/:id', component: () => import('src/pages/v2/Task/DetailPage.vue'),
        children: [
          // { path: 'overview', component: () => import('src/pages/Task/OverviewPage.vue') },
          { path: 'data_collection', component: () => import('src/pages/v2/Task/DataCollectionPage.vue') },
          { path: 'train', component: () => import('src/pages/v2/Task/TrainPage.vue') },
          { path: 'test', component: () => import('src/pages/v2/Task/TestPage.vue') },
        ]
      },
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
