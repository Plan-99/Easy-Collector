import { watch } from 'vue';
import { api } from 'boot/axios';
import { Notify } from 'quasar';
import { t } from 'boot/i18n';
import { useSocket } from 'src/composables/useSocket.js';
import { useTopicStore } from 'src/stores/topicStore';

const { socket } = useSocket();

/**
 * Robot composable — push 모델 기반.
 *
 * `topicStore.isPublished(robot.read_topic)` 변화를 watch하여 robot.status를
 * 자동으로 'on' / 'off'로 동기화하고, 토픽이 등장하는 시점에 backend gRPC
 * 구독(`/robot/{id}/:subscribe_robot`)을 자동 시작한다.
 *
 * `startRobot()`은 driver spawn만 담당 — 토픽 검증/구독 등록은 store watcher가 처리.
 * `stopRobot()`은 driver 정지 + 로컬 상태 초기화.
 *
 * 호환성: `checkRobotTopic`은 노출만 유지 (no-op)되어 기존 호출처가 깨지지 않는다.
 */
export function useRobot(robot, robotOnCallback = () => {}) {
  const topicStore = useTopicStore();

  const formatError = (error) => {
    if (error?.response?.data?.message) return error.response.data.message;
    if (error?.message) return error.message;
    try {
      return JSON.stringify(error);
    } catch {
      return String(error);
    }
  };

  // 초기 상태 (robot 객체에 reactive 필드 보장)
  robot.status = robot.status || 'off';
  robot.jointSub = null;
  robot.jointState = [];
  robot.jointAction = [];
  robot.eePos = {};
  robot.eeTarget = {};
  if (!robot.lastError) robot.lastError = null;

  let subscribed = false;

  function ensureSocketSubscription() {
    if (subscribed) return;
    socket.on(`robot_status_${robot.id}`, (data) => {
      if (data.connected === false) {
        // 백엔드가 stale 판정 — UI는 토픽 watcher가 다음 tick에 정정.
        robot.jointState = [];
        robot.jointAction = [];
        robot.eePos = {};
        robot.eeTarget = {};
        return;
      }
      robot.jointState = data.joint_states;
      robot.jointAction = data.joint_actions;
      robot.eePos = data.ee_pos;
      robot.eeTarget = data.ee_target;
    });
    subscribed = true;
  }

  function teardownSocketSubscription() {
    if (!subscribed) return;
    socket.off(`robot_status_${robot.id}`);
    subscribed = false;
  }

  // 토픽이 등장하면 → backend에 gRPC 구독 시작 + status='on' + on-callback
  // 토픽이 사라지면 → status='off' + 로컬 캐시 초기화
  watch(
    () => topicStore.isPublished(robot.read_topic),
    (isPublished, wasPublished) => {
      if (isPublished && !wasPublished) {
        robot.status = 'on';
        robot.lastError = null;
        ensureSocketSubscription();
        // 백엔드 구독 시작 (record/observe를 위해 joint state stream 필요)
        api.post(`/robot/${robot.id}/:subscribe_robot`).catch(() => {});
        try {
          robotOnCallback();
        } catch (e) {
          console.error('robotOnCallback error:', e);
        }
      } else if (!isPublished && wasPublished) {
        robot.status = 'off';
        robot.jointState = [];
        robot.jointAction = [];
        robot.eePos = {};
        robot.eeTarget = {};
      }
    },
    { immediate: true },
  );

  function status() {
    return robot.status;
  }

  function startRobot() {
    robot.status = 'loading';
    robot.lastError = null;
    return api
      .post('/robot:start', robot)
      .then(() => {
        // 토픽이 발행되면 store watcher가 자동으로 status='on' + subscribe.
        // 만약 일정 시간 안에 토픽이 안 보이면 사용자가 직접 stop 가능.
      })
      .catch((error) => {
        const msg = formatError(error);
        console.error('Error starting robot:', msg);
        robot.lastError = msg;
        robot.status = 'off';
      });
  }

  function stopRobot() {
    robot.status = 'loading';
    return api
      .post('/robot:stop', robot)
      .then(() => {
        robot.status = 'off';
        robot.lastError = null;
        robot.jointState = [];
        robot.jointAction = [];
        robot.eePos = {};
        robot.eeTarget = {};
        teardownSocketSubscription();
        api.post(`/robot/${robot.id}/:unsubscribe_robot`).catch(() => {});
      })
      .catch((error) => {
        const msg = formatError(error);
        console.error('Error stopping robot:', msg);
        robot.status = 'on';
      });
  }

  // 외부에서 명시적으로 socketio joint state stream을 구독하고 싶을 때.
  // 토픽 watcher에 의해 이미 자동으로 호출되지만, 페이지 마운트 시 즉시 콜백을
  // 받기 원하는 경우(예: WatchingRobot)에 한해 직접 호출 가능.
  function subscribeRobot(callback) {
    socket.off(`robot_status_${robot.id}`);
    socket.on(`robot_status_${robot.id}`, (data) => {
      if (data.connected === false) {
        robot.jointState = [];
        robot.jointAction = [];
        robot.eePos = {};
        robot.eeTarget = {};
        return;
      }
      robot.jointState = data.joint_states;
      robot.jointAction = data.joint_actions;
      robot.eePos = data.ee_pos;
      robot.eeTarget = data.ee_target;
      callback?.(data.joint_states, data.joint_actions);
    });
    subscribed = true;
  }

  function unSubscribeRobot() {
    api.post(`/robot/${robot.id}/:unsubscribe_robot`).catch(() => {});
    teardownSocketSubscription();
    robot.jointState = [];
    robot.jointAction = [];
    robot.eePos = {};
    robot.eeTarget = {};
  }

  function moveRobotJoint(goal_pos) {
    socket.emit('move_robot_joint', { robot, goal_pos });
  }
  function moveRobotEE(goal_pos) {
    socket.emit('move_robot_ee', { robot, goal_pos });
  }
  function moveRobotEEDelta(delta_pos) {
    socket.emit('move_robot_ee_delta', { robot, delta_pos });
  }
  function moveRobotJointDelta(delta_pos) {
    socket.emit('move_robot_joint_delta', { robot, delta_pos });
  }

  function goOriginPos() {
    api
      .post(`/robot/${robot.id}/:move_to`, { goal_pos: robot.homepose })
      .then(() => {
        Notify.create({ color: 'positive', message: t('robotPoseMovedToOrigin') });
      })
      .catch((error) => {
        console.error('Error moving robot to origin:', error);
        Notify.create({ color: 'negative', message: t('robotPoseMoveToOriginFailed') });
      });
  }

  // 호환성 stub — push 모델 도입 전엔 시작/체크에서 명시 호출되던 함수.
  // 더 이상 필요하지 않지만 호출처가 남아 있어 no-op로 유지.
  function checkRobotTopic() {
    /* no-op: topicStore가 push 받아 자동으로 상태 동기화 */
  }

  return {
    startRobot,
    stopRobot,
    status,
    moveRobotJoint,
    moveRobotEE,
    moveRobotEEDelta,
    moveRobotJointDelta,
    goOriginPos,
    subscribeRobot,
    unSubscribeRobot,
    checkRobotTopic,
  };
}
