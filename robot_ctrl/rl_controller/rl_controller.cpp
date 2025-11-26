#include "rl_controller.h"

bool RLController::init() {
    // Initialize default joint positions
    // Initialize leg phase angles based on gait schedule
    // Clear all vectors to zero
    vel_commands.setZero();
    leg_theta.setZero();
    last_action.setZero();
    step_counter = 0;
    s_obs_base_lin_vel = 2.0;
    s_obs_base_ang_vel = 0.25;
    s_obs_projected_gravity = 1.0;
    s_obs_velocity_commands = 1.0;
    s_obs_joint_pos = 1.0;
    s_obs_joint_vel = 0.5;

    s_act_joint_position = 0.25;
    s_act_wheel_velocity=1.5;

    for(int i = 0; i < 4; i++) {
        leg_theta[i] = gait_schedule[i];
    }
    initialized_ = true;
    running_ = false;
    // loadPolicy("/home/ouge/Software/dg_fsm_whl/models/policy_09300233.onnx");
    // loadPolicy("/home/ouge/Software/dg_fsm_whl/models/policy_height_6.onnx");
    // loadPolicy("/home/ouge/Software/dg_fsm_whl/models/policy_leg_exp_1118.onnx");
    loadPolicy("/home/ouge/Software/dg_fsm_whl/models/policy_leg_exp_11252018.onnx");
    std::cout << "Policy Loaded" << std::endl;


    gru_hidden_state_.resize(gru_num_layers_ * 1 * gru_hidden_size_, 0.0f); // batch_size=1

    // ... 原有加载模型代码 ...
    std::cout << "Policy Loaded" << std::endl;

    return true;
}

// 文件: robot_ctrl/rl_controller/rl_controller.cpp

bool RLController::step(Vec23<double>* joint_q, Vec22<double>* joint_qd, Vec3<double>* accel, Vec3<double>* desired_vel_xyw, int* stand_flag) {
    
    if (step_counter % 4 == 0) { // 200Hz / 4 = 50Hz (请确保你已经改为 %4)
        
        // 1. 提取 Base 状态 (保持不变)
        Vec4<double> quat;
        quat[0] = (*joint_q)[3]; // w
        quat[1] = (*joint_q)[4]; // x  
        quat[2] = (*joint_q)[5]; // y
        quat[3] = (*joint_q)[6]; // z
        Vec3<double> body_ang_vel;
        body_ang_vel[0] = (*joint_qd)[3];
        body_ang_vel[1] = (*joint_qd)[4]; 
        body_ang_vel[2] = (*joint_qd)[5];

        Eigen::Quaterniond quat_eigen(quat[0], quat[1], quat[2], quat[3]);
        Vec3<double> projected_gravity = (quat_eigen.inverse() * Eigen::Vector3d(0, 0, -1)).cast<double>();
        
        // 2. 准备 Observation 数据
        // 目标顺序 (Policy Order): [LF, LH, RF, RH]
        // 源数据 (MuJoCo/Hardware Order): Base(7) + [RF(4), LF(4), RH(4), LH(4)]
        // 索引: RF=7, LF=11, RH=15, LH=19
        
        Vec12<double> obs_joint_pos;
        Vec16<double> obs_joint_vel; // 12 关节 + 4 轮子

        // --- 填充 Joint Position (减去 Default Pose) ---
        // LF (Source Index 11) - Default[0-2]
        obs_joint_pos.segment<3>(0) = joint_q->segment<3>(11) - default_dof_pos_obs.segment<3>(0);
        // LH (Source Index 19) - Default[3-5]
        obs_joint_pos.segment<3>(3) = joint_q->segment<3>(19) - default_dof_pos_obs.segment<3>(3);
        // RF (Source Index 7)  - Default[6-8]
        obs_joint_pos.segment<3>(6) = joint_q->segment<3>(7)  - default_dof_pos_obs.segment<3>(6);
        // RH (Source Index 15) - Default[9-11]
        obs_joint_pos.segment<3>(9) = joint_q->segment<3>(15) - default_dof_pos_obs.segment<3>(9);

        // --- 填充 Joint Velocity (顺序 LF, LH, RF, RH) ---
        // LF Leg (11)
        obs_joint_vel.segment<3>(0) = joint_qd->segment<3>(11);
        // LH Leg (19)
        obs_joint_vel.segment<3>(3) = joint_qd->segment<3>(19);
        // RF Leg (7)
        obs_joint_vel.segment<3>(6) = joint_qd->segment<3>(7);
        // RH Leg (15)
        obs_joint_vel.segment<3>(9) = joint_qd->segment<3>(15);

        // --- 填充 Wheel Velocity (顺序 LF, LH, RF, RH) ---
        // Source Indices: RF_w=10, LF_w=14, RH_w=18, LH_w=22 (Based on qd, Base=6)
        // Wait, joint_qd indices: Base(6) + RF(4) + LF(4)...
        // RF_w index = 6 + 3 = 9
        // LF_w index = 6 + 4 + 3 = 13
        // RH_w index = 6 + 8 + 3 = 17
        // LH_w index = 6 + 12 + 3 = 21
        
        obs_joint_vel[12] = (*joint_qd)[13]; // LF Wheel
        obs_joint_vel[13] = (*joint_qd)[21]; // LH Wheel
        obs_joint_vel[14] = (*joint_qd)[9];  // RF Wheel
        obs_joint_vel[15] = (*joint_qd)[17]; // RH Wheel

        // 3. 更新命令 (保持不变)
        vel_commands = *desired_vel_xyw;

        // 4. 构建 Observation Vector (53 dim)
        // 顺序: ang_vel(3), gravity(3), cmd(3), joint_pos(12), joint_vel(16), last_action(16)
        observation.segment<3>(0) = body_ang_vel * s_obs_base_ang_vel;
        observation.segment<3>(3) = projected_gravity * s_obs_projected_gravity;
        observation.segment<3>(6) = vel_commands * s_obs_velocity_commands;
        observation.segment<12>(9) = obs_joint_pos * s_obs_joint_pos;
        observation.segment<16>(21) = obs_joint_vel * s_obs_joint_vel;
        observation.segment<16>(37) = last_action;

        // 5. 运行推理 (ONNX / Thread Logic)
        // ... (这部分调用你的推理代码，保持不变) ...
        // 假设得到 output_data 存入 last_action
        // *注意*：如果是多线程，确保这里读到的是 computed last_action

        // 6. 动作映射 (Action Mapping)
        // Policy Output Order: [LF, LH, RF, RH] (Legs + Wheels)
        // Hardware Order:      [FR, FL, RR, RL] (Legs) + [FR_w, FL_w, RR_w, RL_w] (Wheels)
        // 注意：desired_positions 结构通常是 [Legs(12), Wheels(4)]

        Vec16<double> final_actions; // Still in Policy Order [LF, LH, RF, RH]
        
        // (A) 计算腿部目标位置 (Target = Default + Action * Scale)
        for(int i=0; i<12; ++i) {
            final_actions[i] = default_dof_pos_obs[i] + last_action[i] * s_act_joint_position;
        }
        // (B) 计算轮子目标速度 (Target = Action * Scale)
        for(int i=12; i<16; ++i) {
            final_actions[i] = last_action[i] * s_act_wheel_velocity;
        }

        // (C) 映射到 Hardware Order [FR, FL, RR, RL]
        
        // --- Legs ---
        // HW FR (0-2) <--- Policy RF (Indices 6-8)
        desired_positions.segment<3>(0) = final_actions.segment<3>(6);
        
        // HW FL (3-5) <--- Policy LF (Indices 0-2)
        desired_positions.segment<3>(3) = final_actions.segment<3>(0);
        
        // HW RR (6-8) <--- Policy RH (Indices 9-11)
        desired_positions.segment<3>(6) = final_actions.segment<3>(9);
        
        // HW RL (9-11) <--- Policy LH (Indices 3-5)
        desired_positions.segment<3>(9) = final_actions.segment<3>(3);

        // --- Wheels ---
        // Policy Indices: 12(LF), 13(LH), 14(RF), 15(RH)
        // Hardware Indices: 12(FR), 13(FL), 14(RR), 15(RL)
        
        desired_positions[12] = final_actions[14]; // HW FR <--- Policy RF
        desired_positions[13] = final_actions[12]; // HW FL <--- Policy LF
        desired_positions[14] = final_actions[15]; // HW RR <--- Policy RH
        desired_positions[15] = final_actions[13]; // HW RL <--- Policy LH
    }
    
    step_counter++;
    return true;
}

bool RLController::stop() {
    return true;
}

bool RLController::loadPolicy(const std::string& policy_path) {
    // 1. Initialize ONNX Runtime environment 
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "PolicyInference");
    session_options_ = std::make_unique<Ort::SessionOptions>();
    
    // (Optional) Enable CUDA if available
    // Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CUDA(*session_options_, 0));

    // 2. Load the ONNX model
    session_ = std::make_unique<Ort::Session>(*env_, policy_path.c_str(), *session_options_);
    return true;
}