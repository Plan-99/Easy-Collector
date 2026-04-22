#include "dual_arm_admittance_control/admittance_controller.hpp"
#include <tf2_eigen/tf2_eigen.hpp> // Eigen <-> ROS 메시지 변환


DualArmAdmittanceController::DualArmAdmittanceController()
: rclcpp::Node("dual_arm_admittance_control_node"){

}
DualArmAdmittanceController::~DualArmAdmittanceController() {

}
void DualArmAdmittanceController::initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing DualArmAdmittanceController...");

  //잘됬던 파라미터 후보 1
  M << 5.0, 5.0, 5.0;      // Mass [kg]
  D << 100.0, 100.0, 100.0;
  // D << 160.0, 160.0, 160.0; // Damping [Ns/m]
  K << 600.0, 600.0, 600.0; 
  // K << 1200.0, 1200.0, 1200.0; // Stiffness [N/m]

  // M << 10.0, 10.0, 10.0;      // Mass [kg]
  // D << 900.0, 900.0, 900.0; // Damping [Ns/m]
  // K << 300.0, 300.0, 300.0; // Stiffness [N/m]
  

  const int NUM_JOINTS = 6;
  previous_joint_positions_left_.resize(NUM_JOINTS, 0.0);
  previous_joint_positions_right_.resize(NUM_JOINTS, 0.0);
  current_joint_positions_left_.resize(NUM_JOINTS, 0.0);
  current_joint_positions_right_.resize(NUM_JOINTS, 0.0);
  current_velocities_left_.resize(NUM_JOINTS, 0.0);
  current_velocities_right_.resize(NUM_JOINTS, 0.0);
  max_velocities_.resize(NUM_JOINTS, 0.5); // 임시값
  max_acceles_.resize(NUM_JOINTS, 0.5);   // 임시값


  // ROS Class load
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this(), "robot_description");
  kinematic_model_ = robot_model_loader_->getModel();
  if (!kinematic_model_) {
    RCLCPP_ERROR(this->get_logger(), "로봇 모델을 로드하는 데 실패했습니다.");
    return;
  }
  robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
  ik_state_left_ = std::make_shared<moveit::core::RobotState>(*robot_state_);
  ik_state_right_ = std::make_shared<moveit::core::RobotState>(*robot_state_);

  

  // 3. Subscriber (힘/토크 센서)
  force_sub_left_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/RFT_FORCE_left", 10,
    std::bind(&DualArmAdmittanceController::force_left_callback, this, std::placeholders::_1));

  force_sub_right_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/RFT_FORCE_right", 10,
    std::bind(&DualArmAdmittanceController::force_right_callback, this, std::placeholders::_1));
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&DualArmAdmittanceController::joint_state_callback, this, std::placeholders::_1));
  trajectory_sub_left_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/admittance/planned_trajectory_left", 1,
      std::bind(&DualArmAdmittanceController::trajectory_left_callback, this, std::placeholders::_1));

  trajectory_sub_right_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/admittance/planned_trajectory_right", 1,
      std::bind(&DualArmAdmittanceController::trajectory_right_callback, this, std::placeholders::_1));
  plan_admittance_trigger_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/plan_execute/admittance", 10, std::bind(&DualArmAdmittanceController::plan_execute_AdmittanceCallback, this, std::placeholders::_1));
  admittance_pose_lock_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/admittance/pose_lock", 10, std::bind(&DualArmAdmittanceController::admittance_pose_LockCallback, this, std::placeholders::_1));

  // 4. Publisher (관절 궤적 제어 명령)
  joint_trajectory_publisher_left_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/fairino16_controller_left/joint_trajectory", 1); 

  joint_trajectory_publisher_right_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/fairino16_controller_right/joint_trajectory", 1); 
  get_traj_check_pub_ = this->create_publisher<std_msgs::msg::Bool>("execute_return", 10);

  set_bias_trigger_ = this->create_publisher<std_msgs::msg::Int32>("/set_sensor_bias_command", 10);
  force_pub_left_ = this->create_publisher<geometry_msgs::msg::Vector3>("filtered_force_left", 10);
  force_pub_right_ = this->create_publisher<geometry_msgs::msg::Vector3>("filtered_force_right", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

 
    timer_ = this->create_wall_timer(
      CONTROL_PERIOD,
      std::bind(&DualArmAdmittanceController::control_loop_callback, this));

  RCLCPP_INFO(this->get_logger(), "DualArmAdmittanceController initialized. Control loop started at %.2f Hz.", 1.0 / DT);
}

// === 콜백 함수 ===
void DualArmAdmittanceController::admittance_pose_LockCallback(const std_msgs::msg::Int32::SharedPtr msg)
{

  if (msg->data == 1)
  {
    pose_lock_trigger = true;

    RCLCPP_WARN(this->get_logger(), "💡 Admittance Pose Lock Mode **ON**");
  }
  else if (msg->data == 0)
  {
    pose_lock_trigger = false;
    RCLCPP_WARN(this->get_logger(), "🚫 Admittance Pose Lock Mode **OFF**");
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Admittance Pose Lock Mode received unknown value (%d). State unchanged.", msg->data);
    return;
  }

}
void DualArmAdmittanceController::plan_execute_AdmittanceCallback(const std_msgs::msg::Int32::SharedPtr msg)
{

  if (msg->data == 1)
  {
    
    admittance_trigger = false;
    start_admittance_requested_ = true; // <-- 시작 요청 플래그 ON
    // is_initial_admittance_start_ = false;
    velocity_virtual_left_ = Eigen::Vector3d::Zero();  // <-- 가상 속도 초기화 (0)
    position_offset_left_ = Eigen::Vector3d::Zero();     // <-- 위치 오프셋 초기화 (0)
    velocity_virtual_right_ = Eigen::Vector3d::Zero();  
    position_offset_right_ = Eigen::Vector3d::Zero();  
    // target_ee_pose_planned_left_ = Eigen::Isometry3d::Identity();
    // target_ee_pose_planned_right_ = Eigen::Isometry3d::Identity();
    // base_target_pos_left = Eigen::Vector3d::Zero();
    // base_target_pos_right = Eigen::Vector3d::Zero();

    std_msgs::msg::Int32 trigger;
    trigger.data = 1;
    set_bias_trigger_->publish(trigger);
    RCLCPP_WARN(this->get_logger(), "💡 Admittance Mode **ON** (Next plan will be published to controller).");
    return;
  }
  else if (msg->data == 0)
  {
    admittance_trigger = false;
    is_initial_admittance_start_ = true;
    start_admittance_requested_ = false;
    RCLCPP_WARN(this->get_logger(), "🚫 Admittance Mode **OFF** (Next plan will be executed by MoveIt!).");
    return;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Admittance Mode received unknown value (%d). State unchanged.", msg->data);
    return;
  }
  

}
trajectory_msgs::msg::JointTrajectory DualArmAdmittanceController::resample_trajectory(
    const trajectory_msgs::msg::JointTrajectory& input_traj, double dt)
{
    trajectory_msgs::msg::JointTrajectory resampled_traj;
    resampled_traj.joint_names = input_traj.joint_names;
    
    if (input_traj.points.empty()) {
        return resampled_traj;
    }

    // 1. 원본 궤적의 전체 지속 시간 계산
    double total_duration = input_traj.points.back().time_from_start.sec + 
                            input_traj.points.back().time_from_start.nanosec / 1e9;
    
    // === NEW LOGIC: Apply Resample Multiplier for correction ===
    // 💡 참고: 이 값은 일반적으로 클래스 멤버로 정의되어 ROS 파라미터로 로드되어야 합니다.
    // 현재는 임시로 1.0으로 설정합니다. 오차 보정을 위해 1.012나 0.998 등으로 수정하여 테스트해야 합니다.
    const double RESAMPLE_MULTIPLIER = 1.6; //1.65
    
    // 조정된 전체 지속 시간: 이 시간을 기준으로 최종 포인트 개수를 계산합니다.
    double adjusted_total_duration = total_duration * RESAMPLE_MULTIPLIER;
    
    // 2. 리샘플링할 포인트 개수 계산
    // adjusted_total_duration을 기준으로 궤적 포인트 수를 계산하여 길이를 보정합니다.
    size_t num_points = static_cast<size_t>(std::ceil(adjusted_total_duration / dt));
    
    // 최소 1개의 포인트는 있어야 합니다.
    if (num_points == 0) num_points = 1;


    // 3. 각 리샘플링 포인트의 시간 계산
    for (size_t i = 0; i < num_points; ++i) {
        double current_time = i * dt;
        
        // 보간에 사용할 시간은 원본 궤적의 시간을 초과할 수 없습니다.
        double time_for_interpolation = std::min(current_time, total_duration); 

        // Adjusted_total_duration이 total_duration보다 작은 경우 (RESAMPLE_MULTIPLIER < 1.0)
        // 마지막 포인트를 넘어서면 (total_duration 기준) 종료해야 합니다.
        if (current_time > adjusted_total_duration) {
            break; 
        }

        // 4. 선형 보간(Linear Interpolation)을 위한 인접한 원본 포인트 찾기
        const trajectory_msgs::msg::JointTrajectoryPoint* start_point = nullptr;
        const trajectory_msgs::msg::JointTrajectoryPoint* end_point = nullptr;

        // 현재 보간 시간(time_for_interpolation)을 포함하는 세그먼트 찾기
        for (size_t j = 0; j < input_traj.points.size(); ++j) {
            double point_time = input_traj.points[j].time_from_start.sec + 
                                input_traj.points[j].time_from_start.nanosec / 1e9;

            if (point_time >= time_for_interpolation) { 
                if (j == 0) {
                    // 시작점보다 빠르거나 시작점 자체일 경우
                    start_point = &input_traj.points[0];
                    end_point = &input_traj.points[0];
                } else {
                    start_point = &input_traj.points[j-1];
                    end_point = &input_traj.points[j];
                }
                break;
            } else if (j == input_traj.points.size() - 1) {
                // 궤적의 마지막 포인트
                start_point = &input_traj.points.back();
                end_point = &input_traj.points.back();
                break;
            }
        }

        if (!start_point || !end_point) continue;

        double t_start = start_point->time_from_start.sec + start_point->time_from_start.nanosec / 1e9;
        double t_end = end_point->time_from_start.sec + end_point->time_from_start.nanosec / 1e9;
        
        trajectory_msgs::msg::JointTrajectoryPoint new_point;
        new_point.time_from_start = rclcpp::Duration::from_seconds(current_time); // 궤적 시간은 current_time 사용
        
        // 5. 보간 계수 계산
        double alpha = 0.0;
        if (t_end > t_start) {
            // 보간 계수는 time_for_interpolation(원본 궤적 시간 내)을 기준으로 계산합니다.
            alpha = (time_for_interpolation - t_start) / (t_end - t_start);
        } else if (t_end == t_start) {
            alpha = 0.0; // 동일한 시간 (시작점/끝점)
        }

        // 6. 위치, 속도, 가속도 선형 보간
        size_t num_joints = input_traj.joint_names.size();
        new_point.positions.resize(num_joints);
        
        // 속도 및 가속도 보간은 선택적이며, 단순 위치 제어에서는 생략 가능하지만,
        // MoveIt! 궤적 형태를 유지하기 위해 보간합니다.
        new_point.velocities.resize(num_joints, 0.0);
        new_point.accelerations.resize(num_joints, 0.0);

        for (size_t j = 0; j < num_joints; ++j) {
            // 위치 보간
            double q_start = start_point->positions[j];
            double q_end = end_point->positions[j];
            new_point.positions[j] = q_start + alpha * (q_end - q_start);
            
            // 속도 보간 (선택적)
            if (!start_point->velocities.empty() && !end_point->velocities.empty()) {
                double v_start = start_point->velocities[j];
                double v_end = end_point->velocities[j];
                new_point.velocities[j] = v_start + alpha * (v_end - v_start);
            }

            // 가속도 보간 (선택적)
            if (!start_point->accelerations.empty() && !end_point->accelerations.empty()) {
                double a_start = start_point->accelerations[j];
                double a_end = end_point->accelerations[j];
                new_point.accelerations[j] = a_start + alpha * (a_end - a_start);
            }
        }
        
        resampled_traj.points.push_back(new_point);
    }
    
    // 마지막 포인트의 time_from_start를 adjusted_total_duration으로 정확히 설정합니다.
    if (!resampled_traj.points.empty()) {
        resampled_traj.points.back().time_from_start = rclcpp::Duration::from_seconds(adjusted_total_duration);
        // 마지막 포인트의 위치는 total_duration에서의 위치를 가져왔으므로 그대로 둡니다.
    }


    return resampled_traj;
}

void DualArmAdmittanceController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    
    // 1. MoveIt! RobotState 객체에 현재 조인트 상태를 적용 (IK/FK 계산을 위해 유지)
    robot_state_->setVariableValues(*msg);

    // IK 계산을 위해 ik_state_left_/right_ 에도 값을 복사해 둡니다.
    *ik_state_left_ = *robot_state_;
    *ik_state_right_ = *robot_state_;
    
    // --- 2. msg에서 current_joint_positions_로 직접 업데이트 (매 콜백마다) ---
    
    // 임시 벡터를 사용하여 조인트 값을 임시로 저장합니다.
    std::vector<double> temp_current_left(joint_names_left.size());
    std::vector<double> temp_current_right(joint_names_right.size());
    bool left_found_all = true;
    bool right_found_all = true;

    // A. 왼팔 조인트 위치 찾기 및 업데이트
    for (size_t i = 0; i < joint_names_left.size(); ++i) {
        const std::string& joint_name = joint_names_left[i];
        
        // msg->name 배열에서 조인트 이름을 찾습니다.
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
        
        if (it != msg->name.end()) {
            size_t msg_index = std::distance(msg->name.begin(), it); 
            if (msg_index < msg->position.size()) {
                // current_joint_positions_left_의 순서(i)에 맞게 msg의 위치 값을 저장
                temp_current_left[i] = msg->position[msg_index];
            }
        } else {
            // 해당 조인트가 메시지에 없으면 경고 후 플래그 설정
            RCLCPP_WARN_ONCE(this->get_logger(), "Left Joint %s not found in JointState message! Check joint_names_left.", joint_name.c_str());
            left_found_all = false;
        }
    }
    
    // B. 오른팔 조인트 위치 찾기 및 업데이트
    for (size_t i = 0; i < joint_names_right.size(); ++i) {
        const std::string& joint_name = joint_names_right[i];
        
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
        
        if (it != msg->name.end()) {
            size_t msg_index = std::distance(msg->name.begin(), it); 
            if (msg_index < msg->position.size()) {
                temp_current_right[i] = msg->position[msg_index];
            }
        } else {
            RCLCPP_WARN_ONCE(this->get_logger(), "Right Joint %s not found in JointState message! Check joint_names_right.", joint_name.c_str());
            right_found_all = false;
        }
    }

    // 매핑 성공 시에만 최종적으로 current_joint_positions_ 업데이트
    if (left_found_all) {
        current_joint_positions_left_ = temp_current_left;
    }
    if (right_found_all) {
        current_joint_positions_right_ = temp_current_right;
    }
    
    // --- 3. 초기 위치 복사 로직 (Admittance Mode ON 시 한 번만) ---
    
    static bool is_initial_copy_done = false; // 콜백 함수 내 static 변수 유지
    
    if (admittance_trigger) {
        if (!is_initial_copy_done) {
            // 어드미턴스가 켜지고 아직 초기 복사가 되지 않았을 때만 실행
            
            // 왼팔: 이전 위치, 목표 위치를 현재 위치로 설정
            if (left_found_all) {
                previous_joint_positions_left_ = current_joint_positions_left_;
                target_joint_positions_left_ = current_joint_positions_left_; 

                RCLCPP_WARN(this->get_logger(), "Left Arm Joint Start Pos: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                            current_joint_positions_left_[0], current_joint_positions_left_[1], 
                            current_joint_positions_left_[2], current_joint_positions_left_[3], 
                            current_joint_positions_left_[4], current_joint_positions_left_[5]);
      
            }
            // 오른팔: 이전 위치, 목표 위치를 현재 위치로 설정
            if (right_found_all) {
                previous_joint_positions_right_ = current_joint_positions_right_;
                target_joint_positions_right_ = current_joint_positions_right_;
                RCLCPP_WARN(this->get_logger(), "Right Arm Joint Start Pos: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                            current_joint_positions_right_[0], current_joint_positions_right_[1], 
                            current_joint_positions_right_[2], current_joint_positions_right_[3], 
                            current_joint_positions_right_[4], current_joint_positions_right_[5]);
            }
            
            is_initial_copy_done = true;
            RCLCPP_WARN(this->get_logger(), "Admittance start position initialized to current joint state.");
        }
    } else {
        // 어드미턴스가 꺼져 있으면 다음 시작을 위해 플래그를 리셋
        is_initial_copy_done = false;
    }
}

void DualArmAdmittanceController::trajectory_left_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  const trajectory_msgs::msg::JointTrajectory& input_traj = *msg;

  planned_trajectory_left_ = resample_trajectory(input_traj, DT);

  // planned_trajectory_left_ = *msg;
  trajectory_index_left_ = 0;
  is_following_trajectory_left_ = true;
  RCLCPP_WARN(this->get_logger(), "Left arm received new trajectory (%zu points). Starting Admittance following.", planned_trajectory_left_.points.size());

}

void DualArmAdmittanceController::trajectory_right_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  // right_admit_trigger_ = true;

  const trajectory_msgs::msg::JointTrajectory& input_traj = *msg;

  planned_trajectory_right_ = resample_trajectory(input_traj, DT);
  // planned_trajectory_right_ = *msg;
  trajectory_index_right_ = 0;
  is_following_trajectory_right_ = true;
  RCLCPP_WARN(this->get_logger(), "Right arm received new trajectory (%zu points). Starting Admittance following.", planned_trajectory_right_.points.size());
}

void DualArmAdmittanceController::force_left_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    // LPF 계산을 위해 이전 필터링된 값을 임시 저장
    geometry_msgs::msg::Wrench filtered_prev = filtered_force_left_.wrench;

    // 1. 센서 입력값 (New Force)
    double raw_x = 1.0 * msg->wrench.force.x;
    double raw_y = 1.0 * msg->wrench.force.y;
    double raw_z = 1.0 * msg->wrench.force.z;
    
    // 2. 기준점 제거 (Bias Removal) 로직 적용

    // X축 처리
    double bias_removed_x = 0.0;
    if (std::abs(raw_x) > threshold_) {
        // |F_raw| > T 이면: F_removed = F_raw - sign(F_raw) * T
        bias_removed_x = raw_x - std::copysign(threshold_, raw_x);
    }
    
    // Y축 처리
    double bias_removed_y = 0.0;
    if (std::abs(raw_y) > threshold_) {
        bias_removed_y = raw_y - std::copysign(threshold_, raw_y);
    }

    // Z축 처리
    double bias_removed_z = 0.0;
    if (std::abs(raw_z) > threshold_) {
        bias_removed_z = raw_z - std::copysign(threshold_, raw_z);
    }

    // 3. 1차 저역 통과 필터 (LPF) 적용
    // F(t) = 0.9 * F(t-1) + 0.1 * F_bias_removed
    
    filtered_force_left_.wrench.force.x = 0.9 * filtered_prev.force.x + 0.1 * bias_removed_x;
    filtered_force_left_.wrench.force.y = 0.9 * filtered_prev.force.y + 0.1 * bias_removed_y;
    filtered_force_left_.wrench.force.z = 0.9 * filtered_prev.force.z + 0.1 * bias_removed_z;
    
    // Header는 덮어써줍니다.
    filtered_force_left_.header = msg->header;
}

void DualArmAdmittanceController::force_right_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    // LPF 계산을 위해 이전 필터링된 값을 임시 저장
    geometry_msgs::msg::Wrench filtered_prev = filtered_force_right_.wrench;

    // 1. 센서 입력값 (New Force)
    double raw_x = 1.0 * msg->wrench.force.x;
    double raw_y = 1.0 * msg->wrench.force.y;
    double raw_z = 1.0 * msg->wrench.force.z;
    
    // 2. 기준점 제거 (Bias Removal) 로직 적용

    // X축 처리
    double bias_removed_x = 0.0;
    if (std::abs(raw_x) > threshold_) {
        // |F_raw| > T 이면: F_removed = F_raw - sign(F_raw) * T
        bias_removed_x = raw_x - std::copysign(threshold_, raw_x);
    }
    
    // Y축 처리
    double bias_removed_y = 0.0;
    if (std::abs(raw_y) > threshold_) {
        bias_removed_y = raw_y - std::copysign(threshold_, raw_y);
    }

    // Z축 처리
    double bias_removed_z = 0.0;
    if (std::abs(raw_z) > threshold_) {
        bias_removed_z = raw_z - std::copysign(threshold_, raw_z);
    }

    // 3. 1차 저역 통과 필터 (LPF) 적용
    // F(t) = 0.9 * F(t-1) + 0.1 * F_bias_removed
    
    filtered_force_right_.wrench.force.x = 0.9 * filtered_prev.force.x + 0.1 * bias_removed_x;
    filtered_force_right_.wrench.force.y = 0.9 * filtered_prev.force.y + 0.1 * bias_removed_y;
    filtered_force_right_.wrench.force.z = 0.9 * filtered_prev.force.z + 0.1 * bias_removed_z;
    
    filtered_force_right_.header = msg->header;
}

// === 제어 로직 함수 ===

void DualArmAdmittanceController::compute_admittance(
  const Eigen::Vector3d& force,
  double dt,
  Eigen::Vector3d& velocity_out,
  Eigen::Vector3d& position_offset_out,
  const Eigen::Vector3d& M,
  const Eigen::Vector3d& D,
  const Eigen::Vector3d& K
)
{
  // a = (F - D*v - K*x) / M
  Eigen::Vector3d restoring_force = K.cwiseProduct(position_offset_out);
  Eigen::Vector3d damping_force = D.cwiseProduct(velocity_out);

  Eigen::Vector3d acc = (force - damping_force - restoring_force).cwiseQuotient(M);

  velocity_out += acc * dt;
  position_offset_out += velocity_out * dt;


}

void DualArmAdmittanceController::apply_force_balance(
    Eigen::Vector3d& pose_left_offset_out,
    Eigen::Vector3d& pose_right_offset_out,
    const Eigen::Vector3d& force_left,
    const Eigen::Vector3d& force_right,
    double dt
)
{

    Eigen::Vector3d force_diff = force_left - force_right;
    Eigen::Vector3d balance_vel = K_balance.cwiseProduct(force_diff); // 단순화된 K*F 관계


    Eigen::Vector3d relative_offset = balance_vel * dt;
    

    pose_left_offset_out = relative_offset;
    pose_right_offset_out = -relative_offset;
}


double DualArmAdmittanceController::applyConstraints(
    const double v_curr, const double v_cmd,
    const double accel, const double decel, const double eta, const double dt)
{
  double dv = v_cmd - v_curr;
  double v_component_max;
  double v_component_min;
  if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
    v_component_max = accel  * dt;
    v_component_min = -accel  *dt;
  } else {
    v_component_max = -decel  *dt;
    v_component_min = decel  *dt;
  }
  return v_curr + std::clamp(eta * dv, v_component_min, v_component_max);
}

void DualArmAdmittanceController::publish_trajectory(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub,
  const std::vector<std::string>& joint_names,
  const std::vector<double>& positions, const std::vector<double>& next_positions, double dt)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  traj.points.push_back(point);
  point.positions = next_positions;
  point.time_from_start = rclcpp::Duration::from_seconds(dt);
  traj.points.push_back(point);
  pub->publish(traj);
}

// === 핵심 제어 루프 ===

void DualArmAdmittanceController::control_loop_callback()
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);

  auto node = shared_from_this(); 
  double dt = DT; // 하드코딩된 DT 사용 (주기적인 떨림 방지 위해)
  const moveit::core::JointModelGroup* group_left = kinematic_model_->getJointModelGroup("fairino16_v6_group_left");
  const moveit::core::JointModelGroup* group_right = kinematic_model_->getJointModelGroup("fairino16_v6_group_right");

  // 1. 센서 값 읽기 및 좌표계 변환 (EE -> World)
  Eigen::Vector3d force_in_ee_left;
  // 코드의 좌표계 변환: X -> Y, Y -> X, Z -> Z (부호는 로봇/센서 설정에 따름)
  force_in_ee_left <<  filtered_force_left_.wrench.force.z,    // EE X: +Sensor Z (오른팔과 동일 부호로 시작)
                        -filtered_force_left_.wrench.force.y,   // EE Y: -Sensor Y (오른팔과 반대 부호로 시작)
                         filtered_force_left_.wrench.force.x;    // EE Z: +Sensor X (오른팔과 동일 부호로 시작)
  Eigen::Vector3d force_in_ee_right;
  
  // 🚨 오른팔 수정: Y와 X에 모두 (-) 부호를 적용하여 대칭 구조를 만듭니다.
  force_in_ee_right << filtered_force_right_.wrench.force.z,  // EE X축: -Y
                       -filtered_force_right_.wrench.force.y,   // EE Y축: -X
                       filtered_force_right_.wrench.force.x;  // EE Z축: +Z

  Eigen::Matrix3d R_base_to_ee_left = ik_state_left_->getGlobalLinkTransform("ee_fixed_frame_left").linear();
  Eigen::Matrix3d R_base_to_ee_right = ik_state_right_->getGlobalLinkTransform("ee_fixed_frame_right").linear();

  Eigen::Vector3d force_in_world_left = R_base_to_ee_left * force_in_ee_left;
  Eigen::Vector3d force_in_world_right = R_base_to_ee_right * force_in_ee_right;


  // geometry_msgs::msg::Vector3 force_to_publish_right;

  // force_to_publish_right.x = filtered_force_right_.wrench.force.x;
  // force_to_publish_right.y = filtered_force_right_.wrench.force.y;
  // force_to_publish_right.z = filtered_force_right_.wrench.force.z;

  // // 데이터를 "/filtered_force_right" 토픽으로 발행
  // force_pub_right_->publish(force_to_publish_right);


  //  geometry_msgs::msg::Vector3 force_to_publish_left;

  // force_to_publish_left.x = filtered_force_left_.wrench.force.x;
  // force_to_publish_left.y = filtered_force_left_.wrench.force.y;
  // force_to_publish_left.z = filtered_force_left_.wrench.force.z;

  // // 데이터를 "/filtered_force_right" 토픽으로 발행
  // force_pub_left_->publish(force_to_publish_left);



  // 2. 어드미턴스 제어 계산
  Eigen::Vector3d desired_force = Eigen::Vector3d::Zero();
  Eigen::Vector3d force_error_left = force_in_world_left - desired_force;
  Eigen::Vector3d force_error_right = force_in_world_right - desired_force;

  if (start_admittance_requested_) {
    double force_norm_left = force_error_left.norm();
    double force_norm_right = force_error_right.norm();

    // 두 팔의 힘이 모두 임계값 이하일 때만 Admittance를 활성화합니다.
    if (force_norm_left < ADMITTANCE_START_FORCE_THRESHOLD && 
        force_norm_right < ADMITTANCE_START_FORCE_THRESHOLD) 
    {
        // 힘 안정화 확인 후 Admittance 활성화
        admittance_trigger = true;
        start_admittance_requested_ = false;
        
        // 첫 사이클 초기화 필터 로직 실행을 위해 ON
        is_initial_admittance_start_ = true; 

        RCLCPP_WARN(this->get_logger(), 
            "✅ Admittance Mode **ON**. Forces stabilized (L:%.2f, R:%.2f N).", 
            force_norm_left, force_norm_right);
    } else {
        // 아직 힘이 안정화되지 않았음. 제어 루프의 나머지 부분을 스킵하고 다음 틱을 기다립니다.
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            "Admittance Start Pending. Forces still high (L:%.2f, R:%.2f N). Threshold: %.2f N. Waiting...",
            force_norm_left, force_norm_right, ADMITTANCE_START_FORCE_THRESHOLD);
        return; 
    }
  }

  // Eigen::Isometry3d current_ee_pose_left = ik_state_left_->getGlobalLinkTransform("ee_fixed_frame_left");
  // Eigen::Isometry3d current_ee_pose_right = ik_state_right_->getGlobalLinkTransform("ee_fixed_frame_right");
  Eigen::Isometry3d current_ee_pose_left;
  Eigen::Isometry3d current_ee_pose_right;
  if (group_left) {
        // robot_state_에 현재 조인트 상태가 담겨있으므로, 이를 사용하여 FK를 수행합니다.
      // 이 함수는 해당 그룹의 End-Effector(Link)의 Pose를 계산합니다.
      robot_state_->setJointGroupPositions(group_left, current_joint_positions_left_); 
      current_ee_pose_left = robot_state_->getGlobalLinkTransform(group_left->getSolverInstance()->getTipFrames()[0]);
  } else {
      RCLCPP_ERROR(this->get_logger(), "Left JointModelGroup not found!");
      return;
  }

  if (group_right) {
      robot_state_->setJointGroupPositions(group_right, current_joint_positions_right_);
      current_ee_pose_right = robot_state_->getGlobalLinkTransform(group_right->getSolverInstance()->getTipFrames()[0]);
  } else {
      RCLCPP_ERROR(this->get_logger(), "Right JointModelGroup not found!");
      return;
  }


  current_pose_left = current_ee_pose_left.translation();
  current_pose_right = current_ee_pose_right.translation();
  Eigen::Matrix3d current_rotation_matrix_left = current_ee_pose_left.rotation();
  Eigen::Matrix3d current_rotation_matrix_right = current_ee_pose_right.rotation();
  
  
  

  
  // Eigen::Quaterniond current_orientation_left(current_ee_pose_left.rotation());
  // Eigen::Quaterniond current_orientation_right(current_ee_pose_right.rotation());

  // 어드미턴스 모델 업데이트: 가상 속도 및 위치 오프셋 계산
  compute_admittance(force_error_left, dt, velocity_virtual_left_, position_offset_left_, M, D, K);
  compute_admittance(force_error_right, dt, velocity_virtual_right_, position_offset_right_, M, D, K);

  Eigen::Matrix3d target_rotation_left;
  Eigen::Matrix3d target_rotation_right;

  if (!pose_lock_trigger) 
  {
      // Pose Lock이 OFF일 때만 현재 위치를 base target으로 업데이트합니다.
      // (이 위치가 어드미턴스 오프셋이 더해질 기준 위치가 됩니다.)
      base_target_pos_left = current_pose_left;
      base_target_pos_right = current_pose_right;  
      base_target_rot_left_ = current_rotation_matrix_left;
      base_target_rot_right_ = current_rotation_matrix_right;

      // target_rotation_right = base_target_rot_right_; 
      // target_rotation_left = base_target_rot_left_; 
      // Eigen::Quaterniond q_base_left(base_target_rot_left_);
      // RCLCPP_INFO(
      //     this->get_logger(),
      //     "🟡 [POSE_LOCK OFF] LEFT Base Q (Tracking): w:%.4f, x:%.4f, y:%.4f, z:%.4f",
      //     q_base_left.w(), q_base_left.x(), q_base_left.y(), q_base_left.z()
      // );
      
      // Eigen::Quaterniond q_base_right(base_target_rot_right_);
      // RCLCPP_INFO(
      //     this->get_logger(),
      //     "🟡 [POSE_LOCK OFF] RIGHT Base Q (Tracking): w:%.4f, x:%.4f, y:%.4f, z:%.4f",
      //     q_base_right.w(), q_base_right.x(), q_base_right.y(), q_base_right.z()
      // );

  }

  target_ee_pose_left = Eigen::Isometry3d::Identity();
  target_ee_pose_right = Eigen::Isometry3d::Identity();

  if (is_following_trajectory_left_ && trajectory_index_left_ < planned_trajectory_left_.points.size()) {
            
      // 1. 현재 인덱스의 궤적 목표 조인트 위치 사용
      target_joint_from_planner_left_ = planned_trajectory_left_.points[trajectory_index_left_].positions;
      
      // 💡 [로그 수정] 모든 주기 로깅
      size_t remaining_points_left = planned_trajectory_left_.points.size() - trajectory_index_left_;
      RCLCPP_INFO(
          this->get_logger(),
          "Left Arm Trajectory: Index=%zu, Remaining Points=%zu/%zu", 
          trajectory_index_left_, 
          remaining_points_left,
          planned_trajectory_left_.points.size()
      );

      // 2. Fwd Kinematics로 EE 목표 포즈 계산 및 저장
      const moveit::core::JointModelGroup* group_left = kinematic_model_->getJointModelGroup("fairino16_v6_group_left");
      if (group_left && target_joint_from_planner_left_.size() == group_left->getVariableCount()) {
          ik_state_left_->setJointGroupPositions(group_left, target_joint_from_planner_left_);
          target_ee_pose_planned_left_ = ik_state_left_->getGlobalLinkTransform("ee_fixed_frame_left");
          
          // Base Target 업데이트
          base_target_pos_left = target_ee_pose_planned_left_.translation();
          base_target_rot_left_ = target_ee_pose_planned_left_.rotation();
      }
      
      // 3. 인덱스 증가
      trajectory_index_left_++; 
      
      // 4. 모든 궤적을 소진했을 때 False 설정
      if (trajectory_index_left_ >= planned_trajectory_left_.points.size()) {
          RCLCPP_WARN(this->get_logger(), "Left Trajectory Finished. Switching to static Admittance.");
          is_following_trajectory_left_ = false; 
          traj_done_left = true;
          planned_trajectory_left_.points.clear(); // <-- 왼팔 추가
      }
    } 
  // --- 오른팔 궤적 추종 로직 (독립적) ---
  if (is_following_trajectory_right_ && trajectory_index_right_ < planned_trajectory_right_.points.size()) {
            
      // 1. 현재 인덱스의 궤적 목표 조인트 위치 사용
      target_joint_from_planner_right_ = planned_trajectory_right_.points[trajectory_index_right_].positions;
      
      // 💡 [로그 수정] 모든 주기 로깅
      size_t remaining_points_right = planned_trajectory_right_.points.size() - trajectory_index_right_;
      RCLCPP_INFO(
          this->get_logger(),
          "Right Arm Trajectory: Index=%zu, Remaining Points=%zu/%zu", 
          trajectory_index_right_, 
          remaining_points_right,
          planned_trajectory_right_.points.size()
      );

      // 2. Fwd Kinematics로 EE 목표 포즈 계산
      const moveit::core::JointModelGroup* group_right = kinematic_model_->getJointModelGroup("fairino16_v6_group_right");
      
      if (group_right && target_joint_from_planner_right_.size() == group_right->getVariableCount()) {
          ik_state_right_->setJointGroupPositions(group_right, target_joint_from_planner_right_);
          target_ee_pose_planned_right_ = ik_state_right_->getGlobalLinkTransform("ee_fixed_frame_right");
          
          // Base Target 업데이트
          base_target_pos_right = target_ee_pose_planned_right_.translation();
          base_target_rot_right_ = target_ee_pose_planned_right_.rotation();
          
      }

      // 3. 인덱스 증가
      trajectory_index_right_++; 
      
      // 4. 모든 궤적을 소진했을 때 False 설정
      if (trajectory_index_right_ >= planned_trajectory_right_.points.size()) {
          RCLCPP_WARN(this->get_logger(), "Right Trajectory Finished. Switching to static Admittance.");
          is_following_trajectory_right_ = false;
          traj_done_right = true;
          planned_trajectory_right_.points.clear(); // <-- 왼팔 추가
          
      }
    }
  if (traj_done_right && traj_done_left){
    std_msgs::msg::Bool trigger;
    trigger.data = true;
    get_traj_check_pub_->publish(trigger);
    traj_done_right = false;
    traj_done_left = false;

  }
  Eigen::Vector3d target_pos_left = base_target_pos_left + position_offset_left_;
  Eigen::Vector3d target_pos_right = base_target_pos_right + position_offset_right_;
  // Eigen::Vector3d target_pos_left = base_target_pos_left;
  // Eigen::Vector3d target_pos_right = base_target_pos_right;


  

  if (is_following_trajectory_left_ || planned_trajectory_left_.points.size() > 0) {
      // 궤적을 따라가거나 궤적 완료 후 최종 자세를 유지
      target_rotation_left = target_ee_pose_planned_left_.rotation();
  } else {
      // 궤적이 없는 경우: 현재 자세 유지 (순수 Static Admittance)
      target_rotation_left = base_target_rot_left_; 
  
  }

  // 2. 오른팔 자세 결정
  if (is_following_trajectory_right_ || planned_trajectory_right_.points.size() > 0) {
      target_rotation_right = target_ee_pose_planned_right_.rotation();
  } else {
      target_rotation_right = base_target_rot_right_;
   
  }

  target_ee_pose_left.translation() = target_pos_left;
  target_ee_pose_left.linear() = target_rotation_left;

  target_ee_pose_right.translation() = target_pos_right;
  target_ee_pose_right.linear() = target_rotation_right;

  // {
  //     Eigen::Quaterniond target_quat_left(target_rotation_left);
  //     RCLCPP_INFO(
  //         this->get_logger(),
  //         "🟢 LEFT Target EE Pose (IK Input): P[%.3f, %.3f, %.3f], Q[w:%.3f, x:%.3f, y:%.3f, z:%.3f]",
  //         target_pos_left.x(), target_pos_left.y(), target_pos_left.z(),
  //         target_quat_left.w(), target_quat_left.x(), target_quat_left.y(), target_quat_left.z()
  //     );

  //     Eigen::Quaterniond target_quat_right(target_rotation_right);
  //     RCLCPP_INFO(
  //         this->get_logger(),
  //         "🔵 RIGHT Target EE Pose (IK Input): P[%.3f, %.3f, %.3f], Q[w:%.3f, x:%.3f, y:%.3f, z:%.3f]",
  //         target_pos_right.x(), target_pos_right.y(), target_pos_right.z(),
  //         target_quat_right.w(), target_quat_right.x(), target_quat_right.y(), target_quat_right.z()
  //     );
  // }

  // 4. IK (역기구학) 풀이
  

  bool found_ik_left = false;
  bool found_ik_right = false;

  if (group_left) {
      found_ik_left = ik_state_left_->setFromIK(
          group_left,
          target_ee_pose_left,
          "ee_fixed_frame_left", 
          0.0025 // Timeout 
      );
  } else {
      RCLCPP_ERROR(this->get_logger(), "Left JointModelGroup not found!");
  }

  if (group_right) {
      found_ik_right = ik_state_right_->setFromIK(
          group_right,
          target_ee_pose_right,
          "ee_fixed_frame_right", 
          0.0025 // Timeout 
      );
  } else {
      RCLCPP_ERROR(this->get_logger(), "Right JointModelGroup not found!");
  }

  // 5. IK 결과 저장 및 실패 시 이전 값 사용
  if (found_ik_left) {
      ik_state_left_->copyJointGroupPositions(group_left, target_joint_positions_left_);
  } else {
      RCLCPP_WARN(this->get_logger(), "IK Solve Left Failed. Using previous positions.");
      if (previous_joint_positions_left_.size() == target_joint_positions_left_.size()) {
          // target_joint_positions_left_ = previous_joint_positions_left_;
      }
  }

  if (found_ik_right) {
      ik_state_right_->copyJointGroupPositions(group_right, target_joint_positions_right_);
  } else {
      RCLCPP_WARN(this->get_logger(), "IK Solve Right Failed. Using previous positions.");
      if (previous_joint_positions_right_.size() == target_joint_positions_right_.size()) {
          // target_joint_positions_right_ = previous_joint_positions_right_;
      }
  }

  // 6. 속도/가속도 제약 조건 적용 (Motion Smoothing)
  
  std::vector<double> output_positions_left;
  std::vector<double> next_output_positions_left;
  std::vector<double> output_positions_right;
  std::vector<double> next_output_positions_right;
  const double POS_TOLERANCE = 1e-4;
  const double VEL_CHANGE_THRESHOLD = 1e-4;


    // 6-1. 왼팔 제약 조건 적용
    {
      std::vector<double> deltas;
      std::vector<double> durations;

      for (size_t i = 0; i < target_joint_positions_left_.size(); ++i) {
        double delta = target_joint_positions_left_[i] - previous_joint_positions_left_[i];
        deltas.push_back(delta);
        double duration = std::sqrt(2.0 * std::fabs(delta) / (max_acceles_[i] + 1e-6));
        durations.push_back(duration);
      }

      double T_max = *std::max_element(durations.begin(), durations.end());

      output_positions_left.clear();
      next_output_positions_left.clear();

      for (size_t i = 0; i < target_joint_positions_left_.size(); ++i) {
        double delta = deltas[i];
        double target_vel;

        if (std::fabs(delta) < POS_TOLERANCE || T_max < 1e-4) {
          target_vel = 0.0;
        } else {
          target_vel = delta / T_max;
          target_vel = std::clamp(target_vel, -max_velocities_[i], max_velocities_[i]);
        }
        double dv = target_vel - current_velocities_left_[i];

        if (std::fabs(dv) < VEL_CHANGE_THRESHOLD) {
          target_velocities_[i] = current_velocities_left_[i];
        } else {
          target_velocities_[i] = applyConstraints(current_velocities_left_[i], target_vel, max_acceles_[i], -max_acceles_[i], 1.0, dt);
        }

        current_velocities_left_[i] = target_velocities_[i];

        // 1단계 스무딩 위치 계산
        double smoothed = previous_joint_positions_left_[i] + target_velocities_[i] * dt;
        output_positions_left.push_back(smoothed);

        if (fabs(target_joint_positions_left_[i] - smoothed) < POS_TOLERANCE ) {
          target_vel = 0.0;
        }
        // 2단계 스무딩 위치 계산 (다음 주기 예상)
        next_target_velocities_[i] = applyConstraints(current_velocities_left_[i], target_vel, max_acceles_[i], -max_acceles_[i], 1.0, dt);

        smoothed = smoothed + next_target_velocities_[i] * dt;
        next_output_positions_left.push_back(smoothed);
      }
      
      // 📢 로그: 왼팔 최종 크기 확인
      // RCLCPP_INFO(this->get_logger(), 
      //             "Left Trajectory Sizes: Output=%zu, NextOutput=%zu", 
      //             output_positions_left.size(), 
      //             next_output_positions_left.size());

    }

    // 6-2. 오른팔 제약 조건 적용
    {
      std::vector<double> deltas;
      std::vector<double> durations;

      for (size_t i = 0; i < target_joint_positions_right_.size(); ++i) {
        double delta = target_joint_positions_right_[i] - previous_joint_positions_right_[i];
        deltas.push_back(delta);
        double duration = std::sqrt(2.0 * std::fabs(delta) / (max_acceles_[i] + 1e-6));
        durations.push_back(duration);
      }

      double T_max = *std::max_element(durations.begin(), durations.end());

      output_positions_right.clear();
      next_output_positions_right.clear();

      for (size_t i = 0; i < target_joint_positions_right_.size(); ++i) {
        double delta = deltas[i];
        double target_vel;

        if (std::fabs(delta) < POS_TOLERANCE || T_max < 1e-4) {
          target_vel = 0.0;
        } else {
          target_vel = delta / T_max;
          target_vel = std::clamp(target_vel, -max_velocities_[i], max_velocities_[i]);
        }
        double dv = target_vel - current_velocities_right_[i];

        if (std::fabs(dv) < VEL_CHANGE_THRESHOLD) {
          target_velocities_[i] = current_velocities_right_[i];
        } else {
          target_velocities_[i] = applyConstraints(current_velocities_right_[i], target_vel, max_acceles_[i], -max_acceles_[i], 1.0, dt);
        }


        current_velocities_right_[i] = target_velocities_[i];

        // 1단계 스무딩 위치 계산
        double smoothed = previous_joint_positions_right_[i] + target_velocities_[i] * dt;
        output_positions_right.push_back(smoothed);

        if (fabs(target_joint_positions_right_[i] - smoothed) < POS_TOLERANCE ) {
          target_vel = 0.0;
        }
        // 2단계 스무딩 위치 계산 (다음 주기 예상)
        next_target_velocities_[i] = applyConstraints(current_velocities_right_[i], target_vel, max_acceles_[i], -max_acceles_[i], 1.0, dt);

        smoothed = smoothed + next_target_velocities_[i] * dt;
        next_output_positions_right.push_back(smoothed);
      }


    }

  // 7. 궤적 발행 및 상태 업데이트
  if (is_initial_admittance_start_) { 
      // 🚨 명령 필터링: 첫 주기에는 움직임 명령 대신 현재 위치 유지 명령 발행
      RCLCPP_WARN(this->get_logger(), "Admittance First Cycle: Applying Holding Command Filter.");

      // Base Target Pose 재설정 (가장 중요!)
      base_target_pos_left = current_pose_left;
      base_target_pos_right = current_pose_right;  
      base_target_rot_left_ = current_rotation_matrix_left;
      base_target_rot_right_ = current_rotation_matrix_right;

      // 왼팔 정지 명령
      publish_trajectory(joint_trajectory_publisher_left_, joint_names_left, 
                        current_joint_positions_left_, current_joint_positions_left_, dt);
      // 오른팔 정지 명령
      publish_trajectory(joint_trajectory_publisher_right_, joint_names_right, 
                        current_joint_positions_right_, current_joint_positions_right_, dt);
      
      // 상태 업데이트 및 플래그 리셋
      previous_joint_positions_left_ = current_joint_positions_left_; 
      previous_joint_positions_right_ = current_joint_positions_right_;
      is_initial_admittance_start_ = false;
      
      return;

    }
  

  if(admittance_trigger)
  {
    
      publish_trajectory(joint_trajectory_publisher_left_, joint_names_left, output_positions_left, next_output_positions_left, dt);
      publish_trajectory(joint_trajectory_publisher_right_, joint_names_right, output_positions_right, next_output_positions_right,dt);

  }
  previous_joint_positions_left_ = output_positions_left;
  previous_joint_positions_right_ = output_positions_right;

}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DualArmAdmittanceController>();
  node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor; 
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
