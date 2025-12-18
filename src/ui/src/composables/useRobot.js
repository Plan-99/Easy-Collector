import { api } from 'boot/axios';
// import { useROS } from './useROS';
import { Notify } from 'quasar';
import { useSocket } from 'src/composables/useSocket.js';

const { socket } = useSocket();

export function useRobot(robot, robotOnCallback=() => {}) {
  // const { createSubscriber, connectROS } = useROS();
  let robotTopicChecker = null;

  checkRobotTopic(1);

  // connectROS();

  robot.jointSub = null;
  robot.jointState = [];
  robot.jointAction = [];
  robot.eePos = {};
  robot.eeTarget = {};
  // let cmdPublisher = () => {};

  function status() {
    return robot.status
  }

  function startRobot() {
    robot.status = 'loading'
    return api.post('/robot:start', robot).then(() => {
      checkRobotTopic()
    }).catch((error) => {
      console.error('Error starting robot:', error);
      robot.status = 'off';
    });
  }

  function stopRobot() {
    robot.status = 'loading'
    return api.post('/robot:stop', robot).then(() => {
      checkRobotTopic(1)
    }).catch((error) => {
      console.error('Error stopping robot:', error);
      robot.status = 'on';
    });
  }

  function subscribeRobot(callback) {
    socket.on(`robot_status_${robot.id}`, (data) => {
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
    // if (robot.jointSub) {
    //   robot.jointSub.unsubscribe();
    //   robot.jointSub = null;
    // }
    api.post(`/robot/${robot.id}/:unsubscribe_robot`);
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
      api.get(`/topics`)
      .then((res) => {
        const isPublished = Boolean(res.data.topics.find(topic => topic.name === robot.read_topic));
        if (isPublished) {
          robot.status = 'on';
          robotOnCallback();
          api.post(`/robot/${robot.id}/:subscribe_robot`);
          clearInterval(robotTopicChecker);
        }
      })
      .catch(error => {
        console.error('Error setting up WebRTC:', error);
      });
      steps++;
      if (steps >= maxSteps) {
        clearInterval(robotTopicChecker);
        if (robot.status === 'on') {
          stopRobot();
        }
        robot.status = 'off';
        api.post(`/robot/${robot.id}/:unsubscribe_robot`);
      }
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