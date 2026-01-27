// 파일 경로: src/boot/socket.js

import { boot } from 'quasar/wrappers'
import { io } from "socket.io-client";

// 백엔드 주소 (센서 네임스페이스)
const socket = io('http://localhost:5000/sensor', { 
  transports: ['websocket'],
  autoConnect: true
});

export default boot(({ app }) => {
  app.config.globalProperties.$socket = socket
})

// 중요: 여기서 socket을 export 해줘야 다른 파일에서 import { socket }이 가능함
export { socket }