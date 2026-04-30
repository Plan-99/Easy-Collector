// This is just an example,
// so you can safely delete all default props below

export default {
  failed: 'Action failed',
  success: 'Action was successful',

  sensorIntroTitle: 'Robot, First Meet the World',
  sensorIntroBody: "This is the step where we connect the 'senses' that allow the robot to see and feel the world for the first time.",
  sensorIntroBody2: "See for yourself how Easy Trainer transforms complex hardware into the robot's eyes and ears with just a single click.",
  sensorAddFormTitle: 'Add New Sensor',
  sensorEditFormTitle: 'Edit Sensor',
  sensorName: 'Sensor Name',
  sensorType: 'Sensor Type',
  serialNUmber: 'Serial Number',
  ipAddress: 'IP Address',
  noSensorTitle: 'No Sensors Added Yet',
  noSensorBody: "Click the add button to add a new sensor.",

  robotIntroTitle: 'Robot, Control Your Body',
  robotIntroBody: "Now that the robot can see and feel, it's time to give it the ability to move and interact with its environment.",
  robotIntroBody2: "With Easy Trainer, setting up your robot's actuators is as simple as a few clicks, allowing your robot to take its first steps into the world.",
  robotAddFormTitle: 'Add New Robot',
  robotEditFormTitle: 'Edit Robot',
  robotName: 'Robot Name',
  robotType: 'Robot Type',
  noRobotTitle: 'No Robots Added Yet',
  noRobotBody: "Click the add button to add a new robot.",

  workspaceIntroTitle: 'Workspace, Your Robot\'s Playground',
  workspaceIntroBody: "Define the environment where your robot will operate. This is where all the action happens!",
  workspaceIntroBody2: "With Easy Trainer, setting up your robot's workspace is quick and intuitive, allowing you to focus on what matters most - training your robot to excel in its tasks.",
  workspaceCreateFormTitle: 'Create New Workspace',
  workerkspaceEditFormTitle: 'Edit Workspace',
  workspaceName: 'Workspace Name',
  workspaceSensorFormTitle: 'Set Workspace Sensors',
  workspaceRobotFormTitle: 'Set Workspace Robots',

  trainIntroTitle: 'Train, Teach Your Robot New Skills',
  trainIntroBody: "It's time to put your robot to the test and teach it new skills through training.",
  trainIntroBody2: "With Easy Trainer, you can effortlessly set up training sessions that help your robot learn and adapt to its environment.",
  trainCreateFormTitle: 'Create New Training Session',
  trainEditFormTitle: 'Edit Training Session',

  assembleIntroTitle: 'Assemble, Build Your Ideal Robot',
  assembleIntroBody: "Combine different robot parts to create your ideal robot configuration.",
  assembleIntroBody2: "With Easy Trainer, assembling your robot is a breeze, allowing you to experiment with various setups to find the perfect fit for your tasks.",
  assembleCreateFormTitle: 'Create New Assembly',
  assembleEditFormTitle: 'Edit Assembly',
  noAssemblyTitle: 'No Assemblies Created Yet',
  noAssemblyBody: "Click the add button to create a new assembly.",

  checkpointEditFormTitle: 'Edit Checkpoint',

  sensorSetting: 'Sensor Settings',
  robotSetting: 'Robot Settings',
  taskSetting: 'Task Settings',

  sensorConfig: 'Sensor Configuration',
  robotConfig: 'Robot Configuration',

  datasetAddFormTitle: 'Add New Dataset',
  datasetEditFormTitle: 'Edit Dataset',
  datasetName: 'Dataset Name',

  leaderTele: 'Easy Controller',
  keyboardTele: 'Keyboard',
  externalTele: 'External',
  viveTele: 'Vive + External',

  // Teleoperation page / dialog / console
  menuTeleoperation: 'Teleoperation',
  teleopIntroTitle: 'Teleoperation',
  teleopIntroBody:
    'Assembly별로 텔레오퍼레이션을 설정하고 실행합니다. Keyboard 모드는 항상 사용 가능하며, Leader Robot 모드는 별도 설정 후 사용 가능합니다.',
  teleopSettingsTooltip: 'Teleoperation Settings',
  teleopModeKeyboardTip: 'Keyboard teleop available',
  teleopModeLeaderOnTip: 'Leader robot teleop configured',
  teleopModeLeaderOffTip: 'Leader robot teleop not configured',
  teleopSettingTitle: 'Teleoperation Setting',
  teleopTabGeneral: 'General',
  teleopTabKeyboard: 'Keyboard',
  teleopTabLeader: 'Leader Robot',
  teleopGeneralDescription:
    'End-effector offset (m). 각 로봇의 IK 끝점을 기본 프레임에서 [x, y, z] 만큼 이동시킨 가상의 EE를 사용합니다. 기본값은 robot_configs.py의 ee_definitions이며, 사용자가 값을 입력하면 해당 로봇의 default를 override 합니다. 적용하려면 로봇을 다시 시작해야 합니다.',
  teleopGeneralResetToDefault: 'Reset to default',
  teleopGeneralNoIk: '이 로봇은 IK ee_definitions가 정의되지 않았습니다.',
  teleopKeyboardDescription:
    '키보드 텔레옵 키 매핑. 각 키를 (x/y/z/ax/ay/az/tool) 축의 +/- 방향으로 매핑합니다. scale은 EE step size에 곱해지는 가중치 (회전축 기본 20, gripper 30).',
  teleopKeyboardDeltaHint:
    'EE delta = step_size × scale × sign 으로 계산되어 매 keydown마다 전송됩니다.',
  teleopKeyboardStepSize: 'Step size (m)',
  teleopKeyboardLeftArm: 'Left arm',
  teleopKeyboardRightArm: 'Right arm',
  teleopKeyboardKey: 'Key',
  teleopKeyboardAxis: 'Axis',
  teleopKeyboardSign: 'Sign',
  teleopKeyboardScale: 'Scale',
  teleopResetToDefaults: 'Reset to defaults',
  teleopRobotOn: 'Robot ON',
  teleopRobotOff: 'Robot is off',
  teleopRobotOffHint:
    'Turn the robots on to configure leader teleoperation. 현재 로봇 포즈를 origin offset으로 저장하려면 로봇이 켜져 있어야 합니다.',
  teleopStartKeyboard: 'Start Keyboard',
  teleopStopKeyboard: 'Stop Keyboard',
  teleopStartLeader: 'Start Leader',
  teleopStopLeader: 'Stop Leader',
  teleopStartTeleoperation: 'Start Teleoperation',
  teleopStopTeleoperation: 'Stop Teleoperation',
  teleopMethod: 'Method',
  teleopMethodKeyboard: 'Keyboard',
  teleopMethodLeader: 'Leader Robot',
  teleopArmLabel: 'Arm',
  teleopLeftLabel: 'Left',
  teleopRightLabel: 'Right',
  teleopStartLeaderRobot: 'Start Leader Robot',
  teleopStopLeaderRobot: 'Stop Leader Robot',

  // MonitoringWindow
  noSensorsMsg: 'No sensors available. Please add sensors to the workspace.',
  noRobotsMsg: 'No robots available. Please add robots to the workspace.',
  startAllDevices: 'Start all sensors and robots to view live data streams.',
  sensorSuffix: 'sensor',
  robotSuffix: 'body',
  unreadable: 'Unreadable',
  replayPlay: 'Play',
  replayStop: 'Stop',
  replayActionType: 'Action Type',
  replayActionQaction: 'Joint (qaction)',
  replayActionEeDelta: 'EE Delta',
  frequencyHz: 'Frequency (Hz)',
  startInference: 'Start Inference',
  stopInference: 'Stop Inference',
  inferenceSettings: 'Inference Settings',
  reInferenceSteps: 'Re-inference Steps',
  reInferenceStepsHint: '1 = temporal ensemble every step, N = re-infer every N steps',
  temporalEnsembleCoeff: 'Temporal Ensemble Coefficient',
  temporalEnsembleCoeffHint: 'Exponential weight decay (0 = uniform, positive = favor older actions)',
  selectDataset: 'Select Dataset for Data Collection',
  teleoperationType: 'Teleoperation Type:',
  viveOnlySingleArm: 'Only available for single_arm assembly',
  rec: 'REC',
  viveRobotDialogTitle: 'Vive Teleoperation Mode',
  viveRobotDialogMsg: 'Do you want to move the real robot?',
  viveWithRobot: 'Yes (with real robot)',
  viveWithoutRobot: 'No (Vive only, images + EE delta)',
  viveWaiting: 'Waiting for VIVE controller...',
  movingToHomepose: 'Moving to home pose...',
  stopCollection: 'STOP',
  completeEpisode: 'DONE',
  episodeSaved: 'Episode saved successfully.',
  errorCompleteEpisode: 'Failed to complete episode.',
  terminal: 'Terminal',
  viveConnectFail: 'VIVE connection failed',
  selectDatasetRequired: 'Please select a dataset for data collection',
  errorStartCollection: 'Error starting data collection',
  errorStartTest: 'Error starting test',
  errorStopTest: 'Error stopping test',
  errorStartReplay: 'Error starting replay',
  errorStopReplay: 'Error stopping replay',
  inferenceStopped: 'Inference stopped',

  cancel: 'Cancel',
  create: 'Create',
  add: 'Add',
  save: 'Save',

  // Tutorial hints (Korean for now — keys are i18n-ready for future English/locale switch)
  tutorialRobotIntro:
    '등록된 로봇을 모아둔 곳이에요. 카드를 누르면 화면 아래에서 직접 움직여볼 수 있고, 튜토리얼에선 연습용 가상 로봇이 자동으로 들어와 있어요.',
  tutorialRobotCard: '이게 연습용 가상 로봇이에요. 눌러서 움직여보세요.',
  tutorialRobotAdd: '새 로봇을 추가하려면 여기를 누르세요.',
  tutorialRobotPendant:
    '−/+ 버튼으로 관절을 조금씩 움직여볼 수 있어요. 자주 쓰는 자세는 ADD POSE 버튼으로 저장해두면 한 번에 그 자세로 돌아갑니다.',
  tutorialRobotForm:
    '새 로봇을 등록하는 화면이에요. 로봇 종류를 먼저 고르면 필요한 항목만 나타납니다.',

  tutorialSensorIntro:
    '로봇이 보는 화면(카메라)을 등록하는 곳이에요. 카드를 누르면 화면이 잘 들어오는지 확인할 수 있고, 튜토리얼에선 연습용 가상 카메라가 자동으로 들어와 있어요.',
  tutorialSensorCard: '이게 연습용 가상 카메라예요. 눌러서 화면을 확인해보세요.',
  tutorialSensorAdd: '새 카메라/센서를 추가하려면 여기를 누르세요.',
  tutorialSensorPreview: '카메라가 지금 보고 있는 화면이에요.',
  tutorialSensorForm:
    '새 카메라/센서를 등록하는 화면이에요. 센서 종류를 먼저 고르면 필요한 항목만 나타납니다.',

  tutorialAssembleIntro:
    '로봇 여러 대를 한 세트로 묶어두는 곳이에요. (예: 왼팔 + 오른팔 + 그리퍼) 앞으로 작업과 학습은 이 세트 단위로 진행됩니다.',
  tutorialAssembleNew: '이름을 정하고, 아래 로봇 중에서 포함할 것들을 골라 저장하세요.',

  tutorialWorkspaceIntro:
    '워크스페이스는 하나의 작업(예: 컵 옮기기)을 위한 모든 것을 모아두는 곳이에요. 사용할 카메라/로봇, 모은 데이터, 학습 결과를 한 곳에서 관리합니다. 위 드롭다운에서 워크스페이스를 먼저 골라주세요.',
  tutorialWorkspaceSetting:
    '이 작업에 사용할 카메라와 로봇을 고르고, 종류별로 세부 설정을 하는 탭이에요.',
  tutorialWorkspaceData:
    '학습용 시연 데이터를 모으고 관리하는 탭이에요. 폴더를 만들고, 오른쪽 화면에서 로봇을 움직여 에피소드를 모으세요.',
  tutorialWorkspaceInference:
    '학습이 끝난 AI를 골라 실제 로봇에게 작업을 시켜보는 탭이에요.',
  tutorialWorkspaceSensorConfig:
    '오른쪽 영상에서 마우스로 드래그하면 학습에 쓸 영역만 잘라낼 수 있고, 회전 각도도 정할 수 있어요.',
  tutorialWorkspaceRobotConfig:
    '로봇이 작업을 시작할 때 취할 기본 자세(홈 포즈)를 정해주세요.',
  tutorialWorkspaceMonitoring:
    '여기 보이는 카메라 영상이나 로봇 카드를 누르면, 왼쪽에서 그 항목의 세부 설정을 할 수 있어요.',

  tutorialTrainIntro:
    '모은 데이터로 AI에게 작업을 가르치는 곳이에요. 위에서 워크스페이스를 먼저 고르면 단계별 학습이 시작됩니다.',
  tutorialTrainStep1:
    '어떤 데이터로 가르칠지 고르세요. 여러 개를 동시에 고를 수 있어요.',
  tutorialTrainStep2:
    'AI 종류를 정하는 단계예요. 새로 만들거나, 이전에 학습한 결과를 가져와 이어서 학습할 수도 있어요.',
  tutorialTrainStep3:
    '학습할 컴퓨터(서버) 주소를 넣고 연결을 확인한 뒤 시작하세요. 학습이 끝나면 워크스페이스 inference 탭에서 바로 써볼 수 있어요.',

  // Pipeline guide
  pipelineGuideTitle: '전체 사용 가이드',
  pipelineGuideSubtitle: '센서 등록부터 학습·추론까지 EasyTrainer 전체 흐름',
  pipelineWhereLabel: '어디서',
  pipelineWhatLabel: '무엇을',
  pipelineWhyLabel: '왜 필요한가',

  pipelineStep1Title: '1. 센서 등록',
  pipelineStep1Where: '왼쪽 메뉴의 "Sensors" 탭',
  pipelineStep1What:
    '로봇이 사용할 카메라/센서를 추가하고 종류(USB 웹캠, RealSense, 또는 외부 ROS 토픽)를 정해요. 카드를 누르면 라이브 화면이 잘 들어오는지 바로 확인할 수 있습니다.',
  pipelineStep1Why:
    '로봇이 환경을 "보려면" 시각 입력이 먼저 있어야 해요. 여기서 등록한 센서가 학습 데이터의 관찰(observation)이 되고, 추론할 때도 AI의 입력으로 그대로 들어갑니다. 모든 후속 단계가 이 센서 연결에 의존하므로 가장 먼저 셋업해두는 게 좋아요.',

  pipelineStep2Title: '2. 로봇 등록',
  pipelineStep2Where: '왼쪽 메뉴의 "Robots > Management" 탭',
  pipelineStep2What:
    '제어할 로봇을 추가하고 종류(Piper, Dynamixel, Unitree, 또는 외부 ROS 토픽)를 정합니다. 펜던트의 −/+ 버튼으로 관절을 움직여보고, 자주 쓰는 자세는 ADD POSE로 저장해두면 한 번에 그 자세로 돌아갈 수 있어요.',
  pipelineStep2Why:
    '학습할 로봇의 "몸"을 정의하는 단계예요. 통신과 동작이 정상인지 펜던트로 미리 확인해두면, 나중에 데이터 수집 도중 통신 문제로 시간 날리는 일을 막을 수 있어요.',

  pipelineStep3Title: '3. 어셈블리 만들기',
  pipelineStep3Where: '왼쪽 메뉴의 "Robots > Assemble" 탭',
  pipelineStep3What:
    '등록한 로봇들을 left_arm / right_arm / left_tool / right_tool 슬롯에 끼워 하나의 세트로 묶어요. 단일 팔도, 양팔 + 그리퍼 조합도 모두 어셈블리 단위로 정의됩니다.',
  pipelineStep3Why:
    '실제 작업은 보통 여러 부위가 함께 움직여요(예: 왼팔이 잡고 오른팔이 옮기기). 이후 단계인 데이터 수집·학습·추론이 모두 어셈블리 단위로 동작하기 때문에, 같은 로봇 구성을 일관되게 묶어두는 단계입니다.',

  pipelineStep4Title: '4. 워크스페이스 만들기 + 데이터 수집',
  pipelineStep4Where: '왼쪽 메뉴의 "Workspace" 탭',
  pipelineStep4What:
    '하나의 작업(예: 컵 옮기기, 블록 쌓기)을 위한 공간을 만들고 어떤 어셈블리·센서를 쓸지 정합니다. setting 탭에서 카메라 크롭/회전·로봇 홈 포즈를 정하고, data 탭에서 텔레오퍼레이션으로 시연을 녹화해 데이터셋을 모아요.',
  pipelineStep4Why:
    '같은 로봇이라도 작업이 달라지면 환경 설정·데이터·학습 결과가 모두 분리되어야 해요. 워크스페이스는 "이 작업에 필요한 모든 것"을 한 묶음으로 캡슐화합니다. 최종 학습 품질은 여기서 모은 시연 데이터의 양과 다양성에 가장 크게 좌우돼요.',

  pipelineStep5Title: '5. 학습',
  pipelineStep5Where: '왼쪽 메뉴의 "Train" 탭',
  pipelineStep5What:
    '워크스페이스를 고른 뒤 3단계로 진행됩니다 — (1) 학습에 쓸 데이터셋 선택, (2) AI 종류 선택(ACT, Diffusion, Pi0 등) 또는 기존 체크포인트 이어 학습, (3) 학습 서버 주소를 넣고 연결을 확인한 뒤 시작.',
  pipelineStep5Why:
    '모은 시연을 보고 AI가 "비슷한 상황에서 어떻게 움직여야 하는지"를 배우는 단계예요. 데이터가 너무 적거나 자세/조명이 한쪽으로 쏠리면 학습이 잘 안 됩니다. 결과가 만족스럽지 않으면 4번으로 돌아가 데이터를 더 모으거나 다양화하세요.',

  pipelineStep6Title: '6. 추론 (실제 실행)',
  pipelineStep6Where: '"Workspace" 탭 → 워크스페이스 선택 → "inference" 서브탭',
  pipelineStep6What:
    '학습이 끝난 체크포인트를 골라 Start Inference를 누르면 로봇이 학습한 대로 자율 동작합니다. 추론 주기(re-inference steps)·temporal ensemble 계수 같은 세부 옵션도 여기서 조정해요.',
  pipelineStep6Why:
    '학습이 잘 됐는지 실제 로봇으로 검증하는 마지막 단계예요. 의도대로 안 움직이면 4번으로 가서 데이터를 보강하거나, 5번에서 다른 정책/체크포인트로 재학습합니다. 이 4 → 5 → 6 사이클을 반복하면서 성능을 끌어올리는 게 EasyTrainer 워크플로의 핵심이에요.',

  pipelineClose: '닫기',
}
