-- Seed the Module catalog. Mirrors release/ui/modules.py MODULE_REGISTRY.
-- All prices default to 0 KRW; admin updates them via /admin/modules later.
-- Idempotent: safe to re-run. Existing rows keep their priceKrw + active flags.

INSERT INTO "Module" (
  "id", "name", "category", "description", "priceKrw", "active",
  "required", "installByDefault", "assetName", "dependencies",
  "createdAt", "updatedAt"
) VALUES
  -- Core / feature (hidden in launcher UI, required=true → always installed)
  ('core',                 '메인 프레임워크',        'core',      'Flask API, Frontend, DB 등 기본 시스템',                         0, true, true,  true,  NULL,                                                ARRAY[]::TEXT[], NOW(), NOW()),
  ('import',               '로봇/센서 Import',       'feature',   '로봇·센서 불러오기',                                              0, true, true,  true,  NULL,                                                ARRAY[]::TEXT[], NOW(), NOW()),
  ('controller',           'Easy Controller',        'feature',   '텔레오퍼레이션 제어',                                             0, true, true,  true,  NULL,                                                ARRAY[]::TEXT[], NOW(), NOW()),
  ('training',             '학습 (Training)',        'feature',   '모방학습 정책 훈련',                                              0, true, true,  true,  NULL,                                                ARRAY[]::TEXT[], NOW(), NOW()),
  ('inference',            '추론 (Inference)',       'feature',   '학습된 정책 실행',                                                0, true, true,  true,  NULL,                                                ARRAY[]::TEXT[], NOW(), NOW()),
  ('sim_mujoco_tutorial',  '튜토리얼 시뮬레이터',    'feature',   'MuJoCo 기반 튜토리얼 환경 (가상 로봇/카메라/테이블/블록)',         0, true, true,  true,  'module-sim_mujoco_tutorial-{version}.tar.gz',       ARRAY[]::TEXT[], NOW(), NOW()),
  -- Robot drivers (downloaded from Plan-99/Easy-Trainer-Modules releases)
  ('robot_piper',          'Piper',                  'robot',     'Agilex Piper 로봇',                                              0, true, false, false, 'module-robot_piper-{version}.tar.gz',                ARRAY[]::TEXT[], NOW(), NOW()),
  ('robot_dynamixel',      'Dynamixel',              'robot',     'Robotis 서보 모터',                                              0, true, false, false, 'module-robot_dynamixel-{version}.tar.gz',            ARRAY[]::TEXT[], NOW(), NOW()),
  ('robot_unitree',        'Unitree',                'robot',     'Unitree 로봇',                                                   0, true, false, false, 'module-robot_unitree-{version}.tar.gz',              ARRAY[]::TEXT[], NOW(), NOW()),
  ('robot_jaka',           'Jaka',                   'robot',     'Jaka 협동로봇',                                                  0, true, false, false, 'module-robot_jaka-{version}.tar.gz',                 ARRAY[]::TEXT[], NOW(), NOW()),
  ('robot_fairino',        'Fairino',                'robot',     'Fairino 협동로봇 (SDK 직접 제어)',                                0, true, false, false, 'module-robot_fairino-{version}.tar.gz',              ARRAY[]::TEXT[], NOW(), NOW()),
  ('robot_rbpodo',         'RBPodo',                 'robot',     'Rainbow Robotics RBPodo',                                       0, true, false, false, 'module-robot_rbpodo-{version}.tar.gz',               ARRAY[]::TEXT[], NOW(), NOW()),
  ('robot_kinova',         'Kinova Kortex',          'robot',     'Kinova Kortex 로봇',                                             0, true, false, false, 'module-robot_kinova-{version}.tar.gz',               ARRAY[]::TEXT[], NOW(), NOW()),
  ('robot_techman',        'Techman TM',             'robot',     'Techman TM 로봇',                                                0, true, false, false, 'module-robot_techman-{version}.tar.gz',              ARRAY[]::TEXT[], NOW(), NOW()),
  ('gripper_generic',      'Generic Gripper',        'robot',     '범용 그리퍼 인터페이스',                                          0, true, false, false, 'module-gripper_generic-{version}.tar.gz',            ARRAY[]::TEXT[], NOW(), NOW()),
  ('gripper_robotiq',      'Robotiq Gripper',        'robot',     'Robotiq 그리퍼',                                                 0, true, false, false, 'module-gripper_robotiq-{version}.tar.gz',            ARRAY[]::TEXT[], NOW(), NOW()),
  ('gripper_onrobot',      'OnRobot Gripper',        'robot',     'OnRobot 그리퍼',                                                 0, true, false, false, 'module-gripper_onrobot-{version}.tar.gz',            ARRAY[]::TEXT[], NOW(), NOW()),
  ('custom_interfaces',    'Custom Interfaces',      'robot',     'EasyTrainer 커스텀 ROS 메시지',                                  0, true, false, false, 'module-custom_interfaces-{version}.tar.gz',          ARRAY[]::TEXT[], NOW(), NOW()),
  ('serial_comm',          'Serial Communication',   'robot',     '시리얼 통신 유틸리티',                                            0, true, false, false, 'module-serial_comm-{version}.tar.gz',                ARRAY[]::TEXT[], NOW(), NOW()),
  -- Sensors
  ('sensor_webcam',        'Webcam',                 'sensor',    'USB 웹캠 (Logitech 등)',                                         0, true, false, false, 'module-sensor_webcam-{version}.tar.gz',              ARRAY[]::TEXT[], NOW(), NOW()),
  ('sensor_realsense',     'Intel RealSense',        'sensor',    'Intel RealSense 깊이 카메라 (D435, D405 등)',                    0, true, false, false, 'module-sensor_realsense-{version}.tar.gz',           ARRAY[]::TEXT[], NOW(), NOW()),
  -- Extensions
  ('vr_teleop',            'VR 텔레오퍼레이션',      'extension', 'VR 기반 원격 조종',                                              0, true, false, false, 'module-vr_teleop-{version}.tar.gz',                  ARRAY[]::TEXT[], NOW(), NOW()),
  ('test_arm',             'Test Arm',               'extension', '테스트 로봇 암 시뮬레이션',                                       0, true, false, false, 'module-test_arm-{version}.tar.gz',                   ARRAY[]::TEXT[], NOW(), NOW()),
  ('sam3',                 'SAM 3 세그멘테이션',     'extension', 'Meta SAM 3 기반 객체 세그멘테이션 (텍스트/박스 프롬프트, 첫프레임 detect + 트래커)',  0, true, false, false, 'module-sam3-{version}.tar.gz',                       ARRAY[]::TEXT[], NOW(), NOW())
ON CONFLICT ("id") DO UPDATE SET
  -- Refresh metadata but never overwrite the admin-set price or active flag.
  "name"             = EXCLUDED."name",
  "category"         = EXCLUDED."category",
  "description"      = EXCLUDED."description",
  "required"         = EXCLUDED."required",
  "installByDefault" = EXCLUDED."installByDefault",
  "assetName"        = EXCLUDED."assetName",
  "dependencies"     = EXCLUDED."dependencies",
  "updatedAt"        = NOW();
