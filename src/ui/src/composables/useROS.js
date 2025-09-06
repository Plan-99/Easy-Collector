import { ref, shallowRef } from 'vue';
import ROSLIB from 'roslib';

const socketUrl = 'ws://localhost:9090'

// ROS 관련 로직을 모아둔 컴포저블 함수
export function useROS() {
  // ROS 객체와 연결 상태를 반응형으로 관리
  const ros = shallowRef(null); // ROSLIB.Ros 인스턴스 (전체 반응성 필요 없음)
  const isConnected = ref(false);

  const connectROS = () => {
    if (isConnected.value) return;

    ros.value = new ROSLIB.Ros({ url: socketUrl });

    ros.value.on('connection', () => {
      isConnected.value = true;
      console.log('✅ ROS에 성공적으로 연결되었습니다.');
    });

    ros.value.on('error', (error) => {
      console.error('❌ ROS 연결 중 오류가 발생했습니다: ', error);
    });

    ros.value.on('close', () => {
      isConnected.value = false;
      console.log('🔌 ROS 연결이 종료되었습니다.');
    });
  };

  /**
   * ROS 연결을 해제합니다.
   */
  const disconnectROS = () => {
    if (ros.value) {
      ros.value.close();
    }
  };

  /**
   * 특정 토픽에 메시지를 발행하는 함수를 생성하여 반환합니다.
   * @param {string} topicName - 퍼블리시할 토픽 이름
   * @param {string} messageType - 토픽의 메시지 타입
   * @returns {function(object): void} 메시지 객체를 받아 퍼블리시하는 함수
   */
  const createPublisher = (topicName, messageType) => {
    if (!isConnected.value) {
      console.error('ROS에 연결되지 않아 퍼블리셔를 생성할 수 없습니다.');
      return () => {}; // 빈 함수 반환
    }

    const topic = new ROSLIB.Topic({
      ros: ros.value,
      name: topicName,
      messageType: messageType,
    });

    // 메시지 데이터를 받아 실제 발행을 수행하는 함수를 반환
    return (messageData) => {
      const message = new ROSLIB.Message(messageData);
      topic.publish(message);
    };
  };

  const createSubscriber = (topicName, messageType, callback) => {
    if (!isConnected.value) {
      console.error('ROS에 연결되지 않아 구독자를 생성할 수 없습니다.');
      return { unsubscribe: () => {} }; // 빈 함수 반환
    }

    const listener = new ROSLIB.Topic({
      ros: ros.value,
      name: topicName,
      messageType: messageType,
    });

    // 메시지가 도착하면 사용자가 제공한 콜백 함수를 호출
    listener.subscribe(callback);

    // 외부에서 구독을 취소할 수 있도록 unsubscribe 함수를 반환
    return {
      unsubscribe: () => {
        listener.unsubscribe();
      },
    };
  };

  function sendJointState(joint_names, joint_pos, publishFunc) {
    const now = new Date();
    const secs = Math.floor(now.getTime() / 1000);

    const jointStateMessage = {
      header: {
        stamp: {
          secs: secs,
          nsecs: (now.getTime() % 1000) * 1000000,
        },
        frame_id: '',
      },
      name: joint_names,
      position: joint_pos,
      velocity: [],
      effort: [],
    };

    // 준비된 퍼블리시 함수를 호출하여 메시지 발행
    publishFunc(jointStateMessage);
  }

  // 외부에서 사용할 상태와 함수들을 반환
  return {
    isConnected,
    connectROS,
    disconnectROS,
    createPublisher,
    createSubscriber,
    sendJointState,
  };
}