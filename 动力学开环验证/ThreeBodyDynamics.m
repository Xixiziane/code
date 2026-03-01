function [X_dot, UAVstate1, UAVstate2, UAVstate3] = ThreeBodyDynamics(X, F_b1, M_b1, F_b2, M_b2, F_b3, M_b3)
% =========================================================================
% 多体组合式无人机非线性动力学求解器 (Multi-Body Dynamics Solver)
% 适用系统: 三机链翼布局 (左机-中机-右机)，翼尖球铰连接
% 求解方法: 牛顿-欧拉法 + Baumgarte 稳定化高斯约束 (KKT System)
%
% 输入:
%   X       : 系统增广状态向量 [36 x 1] 
%   F_bi    : 机体系下气动力与推力合力 [3 x 1] (N)
%   M_bi    : 机体系下气动与推力合力矩 [3 x 1] (N.m)
%
% 输出:
%   X_dot   : 系统状态导数 [36 x 1] (给 Simulink 积分器)
%   UAVstate: 各机独立状态 [12 x 1] (用于路由和反馈)
% =========================================================================

    %% 1. 物理参数初始化 (基于 InitDatactrl.m 推断或预设典型值)
    % 建议后续将其改为 Simulink Parameter 或从外部传入，目前在内部固化以保独立运行
    m = 1.9;                   % 单机质量 (kg)
    J = diag([0.0894, 0.144, 0.162]);  % 单机转动惯量 (kg.m^2)
    b = 1.2;                    % 单机翼展 (m)
    g = 9.80665;                % 重力加速度 (m/s^2)
    
    % Baumgarte 稳定化参数 (用于抑制数值积分导致的铰链断开漂移)
    alpha_b = 20.0;             
    beta_b = 20.0;

    %% 2. 状态向量解码 (State Decoding)
    % 定义每架机 12 个状态: [x1,euler1,x2,euler2,x3,euler3,v1,pqr1,v2,pqr2,v3,pqr3]^T
    % P_E: 地面系位置; V_B: 机体系速度; Att: 欧拉角; W_B: 机体系角速度
    P1 = X(1:3);   Att1 = X(4:6);   V1 = X(19:21);   W1 = X(22:24);
    P2 = X(7:9); Att2 = X(10:12); V2 = X(25:27); W2 = X(28:30);
    P3 = X(13:15); Att3 = X(16:18); V3 = X(31:33); W3 = X(34:36);

    %% 3. 坐标转换与几何学预结算 (Kinematics & Geometry)
    % 计算旋转矩阵 R (机体到地面 Body -> Earth)
    R1 = eul2rot(Att1);
    R2 = eul2rot(Att2);
    R3 = eul2rot(Att3);

    % 定义铰链点在机体系下的局部位置 (假设重心在中心，铰链在翼尖)
    % r_R: 右翼尖; r_L: 左翼尖
    r_R = [0; b/2; 0]; 
    r_L = [0; -b/2; 0];

    % 提取反对称矩阵用于叉乘运算计算
    r_R_x = skew(r_R);
    r_L_x = skew(r_L);

    %% 4. 动力学矩阵装配 (Dynamics Assembly)
    % 单机广义质量矩阵 (6x6)
    M_single = blkdiag(m * eye(3), J);
    % 系统总质量矩阵 (18x18)
    M_sys = blkdiag(M_single, M_single, M_single);

    % 重力在机体系下的投影 (Earth -> Body 需要 R^T)
    g_E = [0; 0; g];
    g_b1 = R1' * m * g_E;
    g_b2 = R2' * m * g_E;
    g_b3 = R3' * m * g_E;

    % 哥氏力与向心力项 (Coriolis & Centrifugal) C = -omega x (M v)
    C1_v = -m * cross(W1, V1);
    C1_w = -cross(W1, J * W1);
    C2_v = -m * cross(W2, V2);
    C2_w = -cross(W2, J * W2);
    C3_v = -m * cross(W3, V3);
    C3_w = -cross(W3, J * W3);

    % 装配系统右端力项 RHS_dyn (18x1): F_ext + F_gravity + F_coriolis
    RHS_dyn = [
        F_b1 + g_b1 + C1_v; M_b1 + C1_w;
        F_b2 + g_b2 + C2_v; M_b2 + C2_w;
        F_b3 + g_b3 + C3_v; M_b3 + C3_w
    ];

    %% 5. 约束雅可比与稳定化项 (Constraints & Baumgarte Stabilization)
    % 约束位置偏差 Phi (6x1): Hinge 1 (机1右接机2左), Hinge 2 (机2右接机3左)
    Phi_1 = (P1 + R1 * r_R) - (P2 + R2 * r_L);
    Phi_2 = (P2 + R2 * r_R) - (P3 + R3 * r_L);
    Phi = [Phi_1; Phi_2];

    % 约束雅可比矩阵 G (6x18) 
    % 由 d(Phi)/dt = G * [V1;W1; V2;W2; V3;W3] 推导而来
    Z3 = zeros(3,3);
    G = [
         R1, -R1*r_R_x, -R2,  R2*r_L_x, Z3,  Z3;
         Z3,  Z3,        R2, -R2*r_R_x, -R3, R3*r_L_x
    ];

    % 当前广义速度 V_sys (18x1)
    V_sys = [V1; W1; V2; W2; V3; W3];
    
    % 计算漂移加速度 dG_dt * V_sys
    % 推导公式: a_drift = R*(w x v) + R*(w x (w x r))
    drift_1R = R1 * (cross(W1, V1) + cross(W1, cross(W1, r_R)));
    drift_2L = R2 * (cross(W2, V2) + cross(W2, cross(W2, r_L)));
    drift_2R = R2 * (cross(W2, V2) + cross(W2, cross(W2, r_R)));
    drift_3L = R3 * (cross(W3, V3) + cross(W3, cross(W3, r_L)));
    
    dGV = [drift_1R - drift_2L; drift_2R - drift_3L];

    % Baumgarte 稳定化右端项 gamma (6x1)
    gamma = -dGV - 2 * alpha_b * (G * V_sys) - (beta_b^2) * Phi;

    %% 6. 核心 KKT 线性系统求解 (Solving the Linear System)
    % 构建 24x24 的增广矩阵: [M, -G^T; G, 0] * [dot_V; Lambda] = [RHS_dyn; gamma]
    KKT_LHS = [M_sys, -G'; 
               G,     zeros(6,6)];
    KKT_RHS = [RHS_dyn; 
               gamma];

    % 求解机体系加速度与拉格朗日乘子 (内力)
    sol = KKT_LHS \ KKT_RHS;
    dot_V_sys = sol(1:18);    % 提取加速度项
    % Lambda = sol(19:24);    % 约束内力 (如需监控铰链受力可提取此项)

    %% 7. 运动学导数结算 (Kinematic Derivatives)
    % 提取各机体加速度和角加速度
    dot_V1 = dot_V_sys(1:3);   dot_W1 = dot_V_sys(4:6);
    dot_V2 = dot_V_sys(7:9);   dot_W2 = dot_V_sys(10:12);
    dot_V3 = dot_V_sys(13:15); dot_W3 = dot_V_sys(16:18);

    % 位置导数: dot_P_E = R * V_B
    dot_P1 = R1 * V1;
    dot_P2 = R2 * V2;
    dot_P3 = R3 * V3;

    % 欧拉角导数: dot_Att = L * W_B
    dot_Att1 = eul_rate(Att1, W1);
    dot_Att2 = eul_rate(Att2, W2);
    dot_Att3 = eul_rate(Att3, W3);

    %% 8. 重新组装 15 维 UAVstate 输出 (用于外部模块交互)
    % 格式: [euler(3), pqr(3), pos_e(3), vel_b(3), accel_b(3)]
    UAVstate1 = [Att1; W1; P1; V1; dot_V1];
    UAVstate2 = [Att2; W2; P2; V2; dot_V2];
    UAVstate3 = [Att3; W3; P3; V3; dot_V3];

    %% 9. X_dot 重新装配 (严格遵循: [dot_pos, dot_att, dot_vel_b, dot_pqr])
    % 第一部分：位置导数 (地面系) 与 姿态导数 (欧拉角变化率)
    X_dot_kin = [dot_P1; dot_Att1;
                 dot_P2; dot_Att2;
                 dot_P3; dot_Att3];
    
    % 第二部分：机体系速度导数 (加速度) 与 角速度导数
    X_dot_dyn = [dot_V1; dot_W1;
                 dot_V2; dot_W2;
                 dot_V3; dot_W3];

    X_dot = [X_dot_kin; X_dot_dyn];
end

% =========================================================================
% 内部辅助函数 (Helper Functions)
% 保持在一个文件中，无需外部依赖，完美适配 Simulink MATLAB Function
% =========================================================================

function R = eul2rot(att)
    % 欧拉角转旋转矩阵 (Z-Y-X 顺序, 机体坐标系到地面坐标系)
    phi = att(1); theta = att(2); psi = att(3);
    s_phi = sin(phi); c_phi = cos(phi);
    s_the = sin(theta); c_the = cos(theta);
    s_psi = sin(psi); c_psi = cos(psi);
    
    R = [c_the*c_psi, s_phi*s_the*c_psi - c_phi*s_psi, c_phi*s_the*c_psi + s_phi*s_psi;
         c_the*s_psi, s_phi*s_the*s_psi + c_phi*c_psi, c_phi*s_the*s_psi - s_phi*c_psi;
        -s_the,       s_phi*c_the,                     c_phi*c_the];
end

function dot_att = eul_rate(att, w)
    % 机体角速度转欧拉角变化率
    phi = att(1); theta = att(2);
    s_phi = sin(phi); c_phi = cos(phi);
    t_the = tan(theta); c_the = cos(theta);
    
    % 防止万向节死锁 (Gimbal Lock) 附近的奇点
    if abs(c_the) < 1e-4
        c_the = sign(c_the) * 1e-4;
    end
    
    L = [1, s_phi*t_the, c_phi*t_the;
         0, c_phi,      -s_phi;
         0, s_phi/c_the, c_phi/c_the];
         
    dot_att = L * w;
end

function S = skew(v)
    % 向量的反对称矩阵操作
    S = [ 0,    -v(3),  v(2);
          v(3),  0,    -v(1);
         -v(2),  v(1),  0 ];
end