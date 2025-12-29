import { api } from 'boot/axios';
// import { useROS } from './useROS';
import { Notify } from 'quasar';
import { useSocket } from 'src/composables/useSocket.js';

const { socket } = useSocket();

export function useRobot(robot, robotOnCallback=() => {}) {
  // const { createSubscriber, connectROS } = useROS();
  let robotTopicChecker = null;

  const formatError = (error) => {
    if (error?.response?.data?.message) {
      return error.response.data.message;
    }
    if (error?.message) {
      return error.message;
    }
    try {
      return JSON.stringify(error);
    } catch {
      return String(error);
    }
  };

  robot.status = robot.status || 'off';
  robot.jointSub = null;
  robot.jointState = [];
  robot.jointAction = [];
  robot.eePos = {};
  robot.eeTarget = {};
  // let cmdPublisher = () => {};

  if (!robot.lastError) {
    robot.lastError = null;
  }

  if (robot.status === 'on') {
    api.post(`/robot/${robot.id}/:subscribe_robot`);
    robotOnCallback();
  }

  function status() {
    return robot.status
  }

  function startRobot() {
    robot.status = 'loading'
    robot.lastError = null;
    return api.post('/robot:start', robot).then(() => {
      checkRobotTopic()
    }).catch((error) => {
      const msg = formatError(error);
      console.error('Error starting robot:', msg);
      robot.lastError = msg;
      robot.status = 'off';
    });
  }

  function stopRobot() {
    robot.status = 'loading';
    if (robotTopicChecker) {
      clearInterval(robotTopicChecker);
      robotTopicChecker = null;
    }
    return api.post('/robot:stop', robot).then(() => {
      robot.status = 'off';
      robot.lastError = null;
      robot.jointState = [];
      robot.jointAction = [];
      robot.eePos = {};
      robot.eeTarget = {};
      socket.off(`robot_status_${robot.id}`);
      api.post(`/robot/${robot.id}/:unsubscribe_robot`);
    }).catch((error) => {
      const msg = formatError(error);
      console.error('Error stopping robot:', msg);
      robot.status = 'on';
    });
  }

  function subscribeRobot(callback) {
    socket.off(`robot_status_${robot.id}`);
    socket.on(`robot_status_${robot.id}`, (data) => {
      if (data.connected === false) {
        robot.status = 'off';
        robot.lastError = null;
        robot.jointState = [];
        robot.jointAction = [];
        robot.eePos = {};
        robot.eeTarget = {};
        return;
      }
      if (data.connected === true && robot.status !== 'on') {
        robot.status = 'on';
        robot.lastError = null;
      }
      robot.jointState = data.joint_states;
      robot.jointAction = data.joint_actions;
      robot.eePos = data.ee_pos;
      robot.eeTarget = data.ee_target;
      callback(data.joint_states, data.joint_actions);
    });
    // if (robot.status === 'off') {
    //   return;
    // }
    // if (robot.jointSub) {
    //   robot.jointSub.unsubscribe();
    // }
    // robot.jointSub = createSubscriber(robot.read_topic, robot.read_topic_msg, callback);
  }

  function unSubscribeRobot() {
    if (robotTopicChecker) {
      clearInterval(robotTopicChecker);
      robotTopicChecker = null;
    }
    // if (robot.jointSub) {
    //   robot.jointSub.unsubscribe();
    //   robot.jointSub = null;
    // }
    api.post(`/robot/${robot.id}/:unsubscribe_robot`);
    robot.status = 'off';
    robot.jointState = [];
    robot.jointAction = [];
    robot.eePos = {};
    robot.eeTarget = {};
    socket.off(`robot_status_${robot.id}`);
  }


  function moveRobotJoint(goal_pos) {
    socket.emit('move_robot_joint', {
      robot,
      goal_pos
    });
  }

  function moveRobotEE(goal_pos) {
    socket.emit('move_robot_ee', {
      robot,
      goal_pos
    });
  }

  function goOriginPos() {
    api.post(`/robot/${robot.id}/:move_to`, {
      goal_pos: robot.homepose
    }).then(() => {
      Notify.create({
        color: 'positive',
        message: 'Robot moved to origin position'
      });
    }).catch((error) => {
      console.error('Error moving robot to origin:', error);
      Notify.create({
        color: 'negative',
        message: 'Failed to move robot to origin'
      });
    });
  }

  function checkRobotTopic(maxSteps = 10) {
    if (robotTopicChecker) {
      clearInterval(robotTopicChecker);
    }
    let steps = 0;
    robotTopicChecker = setInterval(() => {
      const handleNoTopic = () => {
        steps++;
        if (steps >= maxSteps) {
          clearInterval(robotTopicChecker);
          robotTopicChecker = null;
          robot.status = 'off';
          robot.lastError = 'Robot topics not found after start.';
          api.post('/robot:stop', robot);
          api.post(`/robot/${robot.id}/:unsubscribe_robot`);
        }
      };

      api.get(`/topics`)
      .then((res) => {
        const isPublished = Boolean(res.data.topics.find(topic => topic.name === robot.read_topic));
        if (isPublished) {
          robot.status = 'on';
          robot.lastError = null;
          robotOnCallback();
          api.post(`/robot/${robot.id}/:subscribe_robot`);
          clearInterval(robotTopicChecker);
          robotTopicChecker = null;
          return;
        }
        handleNoTopic();
      })
      .catch(error => {
        console.error('Error setting up WebRTC:', error);
        handleNoTopic();
      });
    }, 1000);
  }

  return {
    startRobot,
    stopRobot,
    status,
    moveRobotJoint,
    moveRobotEE,
    goOriginPos,
    subscribeRobot,
    unSubscribeRobot,
  };
}
