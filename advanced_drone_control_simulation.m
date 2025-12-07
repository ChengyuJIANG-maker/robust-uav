function advanced_drone_control_simulation()
    % 清除工作区和命令窗口
    clear;
    clc;
    close all;
    
    % 初始化仿真参数
    dt = 0.01;          % 时间步长
    T = 20;             % 总仿真时间
    t = 0:dt:T;         % 时间向量
    n = length(t);      % 时间步数
    
    % 无人机参数
    mass = 1.0;         % 质量 (kg)
    g = 9.81;           % 重力加速度 (m/s^2)
    rho = 1.225;        % 空气密度 (kg/m^3)
    Cd = 0.5;           % 阻力系数
    A = 0.1;            % 特征面积 (m^2)
    I = diag([0.01, 0.01, 0.02]); % 惯性矩 
    
    % 控制器参数
    controller_type = 2; % 1: PID, 2: 自适应PID, 3: 滑模控制
    
    % 基础PID参数
    Kp_z0 = 20.0;
    Ki_z0 = 3.0;
    Kd_z0 = 10.0;
    
    Kp_xy0 = 8.0;
    Ki_xy0 = 1.0;
    Kd_xy0 = 5.0;
    
    % 自适应参数
    adaptive_rate = 0.1;
    
    % 滑模控制参数
    lambda = diag([5, 5, 5]); % 滑模面参数
    K_smc = diag([8, 8, 10]); % 滑模控制增益
    
    % 姿态控制器参数
    Kp_att = [8.0; 8.0; 5.0]; % 姿态环PID参数 [φ; θ; ψ]
    Ki_att = [1.0; 1.0; 1.0];
    Kd_att = [3.0; 3.0; 3.0];
    
    %% 环境参数设置
    % 风场参数
    wind_mean = [1.0; 0.5; 0];      % 平均风速 [x; y; z]
    wind_turbulence_intensity = 0.5; % 湍流强度
    wind_scale_length = 50;         % 湍流尺度长度
    
    % 涡流参数
    vortex_strength = 0.3;
    vortex_center = [5; 3; 2];
    vortex_radius = 3;
    
    % 传感器参数
    pos_noise_std = 0.01;   % 位置测量噪声标准差
    vel_noise_std = 0.05;   % 速度测量噪声标准差
    wind_noise_std = 0.1;   % 风速测量噪声标准差
    
    % 卡尔曼滤波器参数
    Q_kf = diag([0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.1, 0.1, 0.1]); % 过程噪声协方差
    R_kf = diag([0.01, 0.01, 0.01, 0.05, 0.05, 0.05]); % 测量噪声协方差
    
    % 初始状态
    pos = [0; 0; 0];        % 真实位置 [x; y; z]
    vel = [0; 0; 0];        % 真实速度
    acc = [0; 0; 0];        % 真实加速度
    quat = [1; 0; 0; 0];    % 四元数表示姿态 (w, x, y, z)
    omega = [0; 0; 0];      % 角速度 [p; q; r] (rad/s)
    
    % 估计状态 (用于卡尔曼滤波)
    x_est = [pos; vel; zeros(3,1)]; % [位置; 速度; 风扰动]
    P_est = eye(9);     % 估计误差协方差
    
    % 目标位置
    target_pos = [2; 1; 3];  % [x; y; z]
    target_vel = [0; 0; 0];  % 目标速度
    
    % 存储历史数据
    pos_history = zeros(3, n);
    vel_history = zeros(3, n);
    target_history = zeros(3, n);
    wind_history = zeros(3, n);
    control_history = zeros(3, n);
    estimated_wind_history = zeros(3, n);
    pos_measured_history = zeros(3, n);
    Kp_history = zeros(3, n);
    Ki_history = zeros(3, n);
    Kd_history = zeros(3, n);
    euler_history = zeros(3, n);
    attitude_target_history = zeros(3, n);
    thrust_history = zeros(1, n);
    torque_history = zeros(3, n);
    
    % 控制误差
    error_int = [0; 0; 0];          % 位置积分误差
    prev_error = [0; 0; 0];         % 上一次位置误差
    error_int_att = [0; 0; 0];      % 姿态误差积分
    prev_error_att = [0; 0; 0];     % 上一次姿态误差
    
    % 初始化风场
    wind_turbulence = zeros(3, 1);
    
    % 自适应控制器参数
    Kp = [Kp_xy0; Kp_xy0; Kp_z0];
    Ki = [Ki_xy0; Ki_xy0; Ki_z0];
    Kd = [Kd_xy0; Kd_xy0; Kd_z0];
    
    % 主仿真循环
    for i = 1:n
        % 计算当前风场 (Dryden湍流模型 + 涡流)
        if i > 1
            wind_turbulence = (1 - dt/wind_scale_length) * wind_turbulence + ...
                sqrt(2*dt/wind_scale_length) * wind_turbulence_intensity * randn(3, 1);
        end
        
        % 添加涡流效应
        vortex_vec = pos - vortex_center;
        vortex_dist = norm(vortex_vec);
        if vortex_dist > 0 && vortex_dist < vortex_radius
            % 涡流速度场 
            vortex_strength_current = vortex_strength * (1 - vortex_dist/vortex_radius);
            vortex_velocity = cross([0; 0; 1], vortex_vec) * vortex_strength_current / vortex_dist;
            wind_turbulence = wind_turbulence + vortex_velocity;
        end
        
        wind_velocity = wind_mean + wind_turbulence;
        wind_history(:, i) = wind_velocity;
        
        % 存储当前真实状态
        pos_history(:, i) = pos;
        vel_history(:, i) = vel;
        target_history(:, i) = target_pos;
        
        % 传感器测量 (添加噪声)
        pos_measured = pos + pos_noise_std * randn(3, 1);
        vel_measured = vel + vel_noise_std * randn(3, 1);
        wind_measured = wind_velocity + wind_noise_std * randn(3, 1);
        
        pos_measured_history(:, i) = pos_measured;
        
        % 卡尔曼滤波器 - 状态估计
        [x_est, P_est] = extended_kalman_filter(...
            x_est, P_est, acc, [pos_measured; vel_measured], dt, Q_kf, R_kf);
        
        estimated_wind = x_est(7:9);
        estimated_wind_history(:, i) = estimated_wind;
        
        % 计算误差 (使用估计的状态)
        pos_est = x_est(1:3);
        vel_est = x_est(4:6);
        
        error = target_pos - pos_est;
        error_int = error_int + error * dt;
        error_deriv = (error - prev_error) / dt;
        
        % 根据控制器类型选择控制策略
        switch controller_type
            case 1 % PID控制
                acc_desired = pid_controller(error, error_int, error_deriv, Kp, Ki, Kd, g);
                
            case 2 % 自适应PID控制
                [acc_desired, Kp, Ki, Kd] = adaptive_pid_controller(...
                    error, error_int, error_deriv, Kp, Ki, Kd, g, adaptive_rate, dt);
                
                % 存储自适应参数
                Kp_history(:, i) = Kp;
                Ki_history(:, i) = Ki;
                Kd_history(:, i) = Kd;
                
            case 3 % 滑模控制
                acc_desired = sliding_mode_controller(...
                    pos_est, vel_est, target_pos, target_vel, lambda, K_smc, g);
                
            otherwise
                acc_desired = pid_controller(error, error_int, error_deriv, Kp, Ki, Kd, g);
        end
        
        % 添加前馈风补偿
        acc_desired = acc_desired + 0.5 * estimated_wind;
        
        % 计算风阻力
        relative_vel = wind_velocity - vel;
        wind_force = 0.5 * rho * Cd * A * norm(relative_vel) * relative_vel;
        
        % ========== 新增部分：姿态控制环 ==========
        
        % 1. 中环：从加速度解算期望推力和期望姿态
        psi_des = 0; % 假设我们希望保持偏航角为0
        
        % 计算总推力指令 (考虑重力加速度和z轴加速度)
        T_des = mass * (g + acc_desired(3)); 
        
        % 从期望加速度解算期望横滚和俯仰角
        phi_des = (1/g) * (acc_desired(1) * sin(psi_des) - acc_desired(2) * cos(psi_des));
        theta_des = (1/g) * (acc_desired(1) * cos(psi_des) + acc_desired(2) * sin(psi_des));
        
        % 限制期望姿态角范围，防止过大
        phi_des = max(min(phi_des, pi/6), -pi/6);   % 限制在±30度内
        theta_des = max(min(theta_des, pi/6), -pi/6); 
        
        attitude_des = [phi_des; theta_des; psi_des];
        
        % 2. 内环：姿态控制器
        % 将当前四元数转换为欧拉角 [φ; θ; ψ]
        euler_angles = quat2euler(quat); 
        
        % 计算姿态误差 e_attitude
        e_att = attitude_des - euler_angles;
        error_int_att = error_int_att + e_att * dt;
        error_deriv_att = (e_att - prev_error_att) / dt;
        
        % 使用PID计算控制力矩 τ
        tau = zeros(3, 1);
        tau(1) = Kp_att(1) * e_att(1) + Ki_att(1) * error_int_att(1) + Kd_att(1) * error_deriv_att(1);
        tau(2) = Kp_att(2) * e_att(2) + Ki_att(2) * error_int_att(2) + Kd_att(2) * error_deriv_att(2);
        tau(3) = Kp_att(3) * e_att(3) + Ki_att(3) * error_int_att(3) + Kd_att(3) * error_deriv_att(3);
        
        % 存储数据
        euler_history(:, i) = euler_angles;
        attitude_target_history(:, i) = attitude_des;
        thrust_history(i) = T_des;
        torque_history(:, i) = tau;
        
        % 3. 动力学更新：使用刚体动力学方程
        % 将推力从机体坐标系转换到世界坐标系
        R = quat2rotm(quat); % 从四元数获取旋转矩阵
        thrust_vector_body = [0; 0; T_des]; % 在机体坐标系中，推力沿Z轴
        thrust_vector_world = R * thrust_vector_body;
        
        % 计算合力
        total_force = thrust_vector_world + wind_force - [0; 0; mass*g];
        
        % 更新线运动状态
        acc = total_force / mass;
        vel = vel + acc * dt;
        pos = pos + vel * dt;
        
        % 更新角运动状态 (欧拉方程)
        alpha = I \ (tau - cross(omega, I * omega)); % 角加速度
        omega = omega + alpha * dt;
        
        % 更新姿态 (四元数更新)
        omega_quat = [0; omega]; % 构建纯四元数
        quat_dot = 0.5 * quatmultiply(quat, omega_quat); 
        quat = quat + quat_dot * dt;
        quat = quat / norm(quat); % 单位化
        
        % 更新误差
        prev_error = error;
        prev_error_att = e_att; % 更新姿态误差
        
        % ========== 新增部分结束 ==========
        
        % 存储控制信号
        control_history(:, i) = acc_desired;
        
        % 确保无人机不穿过地面
        if pos(3) < 0
            pos(3) = 0;
            vel(3) = max(0, vel(3)); % 只允许向上反弹
        end
        
        % 随时间改变目标位置，展示追踪能力
        if i == round(n/6)
            target_pos = [1; -1; 2];
        elseif i == round(2*n/6)
            target_pos = [-1; 2; 4];
        elseif i == round(3*n/6)
            target_pos = [0; 0; 1];
        elseif i == round(4*n/6)
            target_pos = [3; -2; 2];
        elseif i == round(5*n/6)
            target_pos = [-2; 3; 3];
        end
    end
    
    % 绘制结果
    figure('Position', [50, 50, 1600, 1000]);
    
    % 3D轨迹图
    subplot(3, 4, [1, 5]);
    plot3(pos_history(1, :), pos_history(2, :), pos_history(3, :), 'b-', 'LineWidth', 2);
    hold on;
    plot3(target_history(1, :), target_history(2, :), target_history(3, :), 'r--', 'LineWidth', 2);
    plot3(pos_measured_history(1, :), pos_measured_history(2, :), pos_measured_history(3, :), 'g:', 'LineWidth', 1);
    scatter3(target_history(1, 1), target_history(2, 1), target_history(3, 1), 100, 'g', 'filled');
    scatter3(target_history(1, end), target_history(2, end), target_history(3, end), 100, 'r', 'filled');
    
    % 绘制涡流区域
    [x_vortex, y_vortex, z_vortex] = sphere(20);
    x_vortex = vortex_center(1) + vortex_radius * x_vortex;
    y_vortex = vortex_center(2) + vortex_radius * y_vortex;
    z_vortex = vortex_center(3) + vortex_radius * z_vortex;
    surf(x_vortex, y_vortex, z_vortex, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', 'm');
    
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('无人机3D轨迹 (含风扰动和涡流)');
    legend('真实轨迹', '目标轨迹', '测量轨迹', '起始点', '终点', '涡流区域');
    axis equal;
    
    % 位置随时间变化
    subplot(3, 4, 2);
    plot(t, pos_history(1, :), 'r-', 'LineWidth', 2);
    hold on;
    plot(t, pos_history(2, :), 'g-', 'LineWidth', 2);
    plot(t, pos_history(3, :), 'b-', 'LineWidth', 2);
    plot(t, target_history(1, :), 'r--', 'LineWidth', 1.5);
    plot(t, target_history(2, :), 'g--', 'LineWidth', 1.5);
    plot(t, target_history(3, :), 'b--', 'LineWidth', 1.5);
    
    grid on;
    xlabel('时间 (s)');
    ylabel('位置 (m)');
    title('位置 vs 时间');
    legend('X', 'Y', 'Z', '目标X', '目标Y', '目标Z');
    
    % 速度随时间变化
    subplot(3, 4, 6);
    plot(t, vel_history(1, :), 'r-', 'LineWidth', 2);
    hold on;
    plot(t, vel_history(2, :), 'g-', 'LineWidth', 2);
    plot(t, vel_history(3, :), 'b-', 'LineWidth', 2);
    
    grid on;
    xlabel('时间 (s)');
    ylabel('速度 (m/s)');
    title('速度 vs 时间');
    legend('V_x', 'V_y', 'V_z');
    
    % XY平面轨迹
    subplot(3, 4, 3);
    plot(pos_history(1, :), pos_history(2, :), 'b-', 'LineWidth', 2);
    hold on;
    plot(target_history(1, :), target_history(2, :), 'r--', 'LineWidth', 2);
    plot(pos_measured_history(1, :), pos_measured_history(2, :), 'g:', 'LineWidth', 1);
    
    % 绘制涡流区域在XY平面的投影
    theta = 0:0.1:2*pi;
    x_vortex_proj = vortex_center(1) + vortex_radius * cos(theta);
    y_vortex_proj = vortex_center(2) + vortex_radius * sin(theta);
    fill(x_vortex_proj, y_vortex_proj, 'm', 'FaceAlpha', 0.2, 'EdgeColor', 'm');
    
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('XY平面轨迹');
    legend('真实轨迹', '目标轨迹', '测量轨迹', '涡流区域');
    axis equal;
    
    % 高度随时间变化
    subplot(3, 4, 7);
    plot(t, pos_history(3, :), 'b-', 'LineWidth', 2);
    hold on;
    plot(t, target_history(3, :), 'r--', 'LineWidth', 2);
    plot(t, pos_measured_history(3, :), 'g:', 'LineWidth', 1);
    
    grid on;
    xlabel('时间 (s)');
    ylabel('高度 Z (m)');
    title('高度 vs 时间');
    legend('真实高度', '目标高度', '测量高度');
    
    % 控制信号
    subplot(3, 4, 11);
    plot(t, control_history(1, :), 'r-', 'LineWidth', 2);
    hold on;
    plot(t, control_history(2, :), 'g-', 'LineWidth', 2);
    plot(t, control_history(3, :), 'b-', 'LineWidth', 2);
    
    grid on;
    xlabel('时间 (s)');
    ylabel('控制加速度 (m/s^2)');
    title('控制信号');
    legend('a_x', 'a_y', 'a_z');
    
    % 姿态角随时间变化
    subplot(3, 4, 8);
    plot(t, euler_history(1, :)*180/pi, 'r-', 'LineWidth', 2);
    hold on;
    plot(t, euler_history(2, :)*180/pi, 'g-', 'LineWidth', 2);
    plot(t, euler_history(3, :)*180/pi, 'b-', 'LineWidth', 2);
    plot(t, attitude_target_history(1, :)*180/pi, 'r--', 'LineWidth', 1.5);
    plot(t, attitude_target_history(2, :)*180/pi, 'g--', 'LineWidth', 1.5);
    plot(t, attitude_target_history(3, :)*180/pi, 'b--', 'LineWidth', 1.5);
    
    grid on;
    xlabel('时间 (s)');
    ylabel('姿态角 (度)');
    title('姿态角 vs 时间');
    legend('φ', 'θ', 'ψ', 'φ_{目标}', 'θ_{目标}', 'ψ_{目标}');
    
    % 位置误差
    subplot(3, 4, 4);
    pos_error = sqrt(sum((pos_history - target_history).^2, 1));
    plot(t, pos_error, 'k-', 'LineWidth', 2);
    
    grid on;
    xlabel('时间 (s)');
    ylabel('位置误差 (m)');
    title('位置误差 vs 时间');
    
    % 推力与力矩
    subplot(3, 4, 12);
    yyaxis left;
    plot(t, thrust_history, 'k-', 'LineWidth', 2);
    ylabel('推力 (N)');
    
    yyaxis right;
    plot(t, torque_history(1, :), 'r-', 'LineWidth', 1.5);
    hold on;
    plot(t, torque_history(2, :), 'g-', 'LineWidth', 1.5);
    plot(t, torque_history(3, :), 'b-', 'LineWidth', 1.5);
    ylabel('力矩 (Nm)');
    
    grid on;
    xlabel('时间 (s)');
    title('推力与力矩');
    legend('推力', 'τ_x', 'τ_y', 'τ_z');
    
    % 添加整体标题
    controller_names = {'PID', '自适应PID', '滑模控制'};
    sgtitle(sprintf('高级无人机控制仿真 - %s控制器 (含姿态控制)', ...
        controller_names{controller_type}), 'FontSize', 16);
    
    % 显示无人机性能指标
    fprintf('无人机控制仿真完成!\n');
    fprintf('控制器类型: %s\n', controller_names{controller_type});
    fprintf('最终位置: [%.2f, %.2f, %.2f] m\n', pos(1), pos(2), pos(3));
    fprintf('目标位置: [%.2f, %.2f, %.2f] m\n', target_pos(1), target_pos(2), target_pos(3));
    fprintf('位置误差: %.2f m\n', norm(pos - target_pos));
    
    % 计算性能指标
    position_error = sqrt(sum((pos_history - target_history).^2, 1));
    mean_error = mean(position_error);
    max_error = max(position_error);
    fprintf('平均位置误差: %.2f m\n', mean_error);
    fprintf('最大位置误差: %.2f m\n', max_error);
    
    % 计算风估计误差
    wind_error = sqrt(sum((wind_history - estimated_wind_history).^2, 1));
    mean_wind_error = mean(wind_error);
    fprintf('平均风估计误差: %.2f m/s\n', mean_wind_error);
end

%% PID控制器
function acc_desired = pid_controller(error, error_int, error_deriv, Kp, Ki, Kd, g)
    acc_desired = zeros(3, 1);
    
    % XY平面控制
    acc_desired(1) = Kp(1) * error(1) + Ki(1) * error_int(1) + Kd(1) * error_deriv(1);
    acc_desired(2) = Kp(2) * error(2) + Ki(2) * error_int(2) + Kd(2) * error_deriv(2);
    
    % 高度控制 (Z方向)
    acc_desired(3) = Kp(3) * error(3) + Ki(3) * error_int(3) + Kd(3) * error_deriv(3) + g;
end

%% 自适应PID控制器
function [acc_desired, Kp_new, Ki_new, Kd_new] = adaptive_pid_controller(...
    error, error_int, error_deriv, Kp, Ki, Kd, g, adaptive_rate, dt)
    
    % 根据误差大小调整增益
    error_norm = norm(error);
    
    % 自适应调整规则
    Kp_new = Kp + adaptive_rate * error_norm * [1; 1; 1.5]; % Z方向增益调整更大
    Ki_new = Ki + adaptive_rate * norm(error_int) * [0.5; 0.5; 1];
    Kd_new = Kd + adaptive_rate * norm(error_deriv) * [1; 1; 2];
    
    % 限制增益范围
    Kp_new = min(max(Kp_new, [2; 2; 5]), [30; 30; 40]);
    Ki_new = min(max(Ki_new, [0.1; 0.1; 0.5]), [5; 5; 8]);
    Kd_new = min(max(Kd_new, [2; 2; 5]), [20; 20; 30]);
    
    % 计算控制输出
    acc_desired = pid_controller(error, error_int, error_deriv, Kp_new, Ki_new, Kd_new, g);
end

%% 滑模控制器
function acc_desired = sliding_mode_controller(...
    pos, vel, target_pos, target_vel, lambda, K, g)
    
    % 计算误差
    pos_error = target_pos - pos;
    vel_error = target_vel - vel;
    
    % 滑模面
    s = vel_error + lambda * pos_error;
    
    % 滑模控制律
    acc_desired = lambda * vel_error + K * sign(s) + [0; 0; g];
end

%% 扩展卡尔曼滤波器
function [x_est, P_est] = extended_kalman_filter(x_est, P_est, u, z, dt, Q, R)
    % 状态向量: [x; y; z; vx; vy; vz; wx; wy; wz]
    % 控制输入: 加速度 [ax; ay; az]
    % 测量: [x; y; z; vx; vy; vz]
    
    % 预测步骤
    % 状态转移函数
    f = @(x) [
        x(1) + x(4)*dt;
        x(2) + x(5)*dt;
        x(3) + x(6)*dt;
        x(4) + u(1)*dt;
        x(5) + u(2)*dt;
        x(6) + u(3)*dt;
        x(7); % 风速假设为常数
        x(8);
        x(9)
    ];
    
    % 状态转移雅可比矩阵
    F = [
        1, 0, 0, dt, 0, 0, 0, 0, 0;
        0, 1, 0, 0, dt, 0, 0, 0, 0;
        0, 0, 1, 0, 0, dt, 0, 0, 0;
        0, 0, 0, 1, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 1, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 1, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 1, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 1
    ];
    
    % 预测状态和协方差
    x_pred = f(x_est);
    P_pred = F * P_est * F' + Q;
    
    % 更新步骤
    % 测量函数
    h = @(x) [x(1:6)]; % 只测量位置和速度
    
    % 测量雅可比矩阵
    H = [
        1, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 1, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 1, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 1, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 1, 0, 0, 0
    ];
    
    % 卡尔曼增益
    K_gain = P_pred * H' / (H * P_pred * H' + R);
    
    % 更新状态和协方差
    x_est = x_pred + K_gain * (z - h(x_pred));
    P_est = (eye(9) - K_gain * H) * P_pred;
end

%% 四元数转欧拉角
function euler = quat2euler(q)
    % 将四元数 (w, x, y, z) 转换为欧拉角 (φ, θ, ψ)
    w = q(1); x = q(2); y = q(3); z = q(4);
    
    % 计算欧拉角
    phi = atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2));
    theta = asin(2*(w*y - z*x));
    psi = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2));
    
    euler = [phi; theta; psi];
end

%% 四元数乘法
function ab = quatmultiply(a, b)
    % 四元数乘法
    a0 = a(1); a1 = a(2); a2 = a(3); a3 = a(4);
    b0 = b(1); b1 = b(2); b2 = b(3); b3 = b(4);
    
    ab = [a0*b0 - a1*b1 - a2*b2 - a3*b3;
          a0*b1 + a1*b0 + a2*b3 - a3*b2;
          a0*b2 - a1*b3 + a2*b0 + a3*b1;
          a0*b3 + a1*b2 - a2*b1 + a3*b0];
end

%% 四元数转旋转矩阵
function R = quat2rotm(q)
    % 从四元数计算旋转矩阵
    w = q(1); x = q(2); y = q(3); z = q(4);
    
    R = [1-2*(y^2+z^2)   2*(x*y - w*z)   2*(x*z + w*y);
         2*(x*y + w*z)   1-2*(x^2+z^2)   2*(y*z - w*x);
         2*(x*z - w*y)   2*(y*z + w*x)   1-2*(x^2+y^2)];
end