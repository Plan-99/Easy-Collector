// src/composables/useSocket.js
import { ref, readonly } from 'vue';
import { io } from 'socket.io-client';

const backendUrl = 'http://192.168.50.46:5000'; // 백엔드 URL을 환경 변수로 설정하거나 다른 방법으로 관리할 수 있습니다.

// 모듈 스코프에서 소켓 인스턴스를 한 번만 생성 (싱글톤)
const socket = io(backendUrl, {
  transports: ['websocket'],
  reconnectionAttempts: 5,
  reconnectionDelay: 1000,
});

// 공유할 반응형 상태
const isConnected = ref(socket.connected);
const imageData = ref(null);

// 연결 이벤트 처리
socket.on('connect', () => {
  console.log('Connected to server, Socket ID:', socket.id);
  isConnected.value = true;
});

// 연결 해제 이벤트 처리
socket.on('disconnect', () => {
  console.log('Disconnected from server');
  isConnected.value = false;
});

// 커스텀 'image' 이벤트 처리
socket.on('image', (data) => {
  console.log('Received image data:', data);
  imageData.value = data; // 받은 데이터를 반응형 상태에 저장
});

// 이 컴포저블 함수는 필요한 데이터와 소켓 인스턴스를 반환합니다.
export function useSocket() {
  return {
    socket: socket, // 소켓 인스턴스를 직접 수정하지 못하도록 readonly로 감쌀 수 있습니다.
    isConnected: readonly(isConnected),
    imageData: readonly(imageData),
  };
}