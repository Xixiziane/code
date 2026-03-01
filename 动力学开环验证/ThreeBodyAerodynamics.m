function[F_b1, M_b1, F_b2, M_b2, F_b3, M_b3] = ThreeBodyAerodynamics(X, Ctrl1, Ctrl2, Ctrl3)
% =========================================================================
% 多体组合式无人机气动力解算模块 
% 
% 输入:
%   X          : 积分器输出的 36 维系统总状态向量 (切断代数环的关键!)
%   Ctrl_i     : 舵面输入[delta_a, delta_e, delta_r, delta_t]
% =========================================================================

    %% 1. 环境与物理参数提取
    rho = 1.225;            % 空气密度 (kg/m^3)
    span = 1.2;             % 单机翼展 (m)
    chord = 0.3;            % 机翼弦长 (m)
    S_wing = span * chord;  % 单机机翼面积 (m^2)
    
    a0 = 2 * pi;            % 翼型升力线斜率 (rad^-1)
    alpha_0 = -0.05;        % 零升攻角 (rad)
    Cd0_airfoil = 0.01;     % 翼型零升阻力系数
    
    Cmq = -50.8;    Cnr = -0.411;   Clp = -0.414;
    CmDe = -1.13;   CnDr = -0.0345; ClDa = 0.0677;
% 添加纵向静稳定性参数
    Cm0 = 0.135;    Cma = -1.5;
    %% 2. 状态向量解码 (直接从积分器状态 X 中提取速度和角速度)
    % 严格对应 ThreeBodyDynamics 中定义的 X 顺序
    V1 = X(19:21); W1 = X(22:24);
    V2 = X(25:27); W2 = X(28:30);
    V3 = X(31:33); W3 = X(34:36);

    %% 3. 机翼离散化准备 (条带法/升力线法核心)
    N_seg = 6; 
    dy = span / N_seg; 
    y_stations = linspace(-span/2 + dy/2, span/2 - dy/2, N_seg);
    
    oswald_coupled = 0.85; 
    AR_coupled = (3*span)^2 / (3*S_wing);
    K_ind = 1 / (pi * oswald_coupled * AR_coupled);

    %% 4. 分别计算每架飞机的气动力
    % 1 号机 (左机)
    [F_wing1, M_wing1] = computeWingAero(V1, W1, y_stations, dy, chord, a0, alpha_0, Cd0_airfoil, rho, K_ind, Ctrl1(1));
    [F_tail1, M_tail1] = computeTailAero(V1, W1, Ctrl1, rho, S_wing, span, chord, Cmq, Cnr, Clp, CmDe, CnDr, Cm0, Cma);
    
    % 2 号机 (中机)
    [F_wing2, M_wing2] = computeWingAero(V2, W2, y_stations, dy, chord, a0, alpha_0, Cd0_airfoil, rho, K_ind, Ctrl2(1));[F_tail2, M_tail2] = computeTailAero(V2, W2, Ctrl2, rho, S_wing, span, chord, Cmq, Cnr, Clp, CmDe, CnDr, Cm0, Cma);

    % 3 号机 (右机)
    [F_wing3, M_wing3] = computeWingAero(V3, W3, y_stations, dy, chord, a0, alpha_0, Cd0_airfoil, rho, K_ind, Ctrl3(1));[F_tail3, M_tail3] = computeTailAero(V3, W3, Ctrl3, rho, S_wing, span, chord, Cmq, Cnr, Clp, CmDe, CnDr, Cm0, Cma);

    %% 5. 推力计算 
    T_max = 15.0; 
    Thrust1 =[T_max * Ctrl1(4); 0; 0];
    Thrust2 =[T_max * Ctrl2(4); 0; 0];
    Thrust3 =[T_max * Ctrl3(4); 0; 0];

    %% 6. 合成最终力和力矩
    F_b1 = F_wing1 + F_tail1 + Thrust1;
    M_b1 = M_wing1 + M_tail1;
    F_b2 = F_wing2 + F_tail2 + Thrust2;
    M_b2 = M_wing2 + M_tail2;
    F_b3 = F_wing3 + F_tail3 + Thrust3;
    M_b3 = M_wing3 + M_tail3;
end

% 内部辅助函数
function [F_wing, M_wing] = computeWingAero(V_b, W_b, y_stations, dy, chord, a0, alpha_0, Cd0, rho, K_ind, delta_a)
    F_wing = zeros(3,1); M_wing = zeros(3,1);
    u = V_b(1); w = V_b(3);
    for i = 1:length(y_stations)
        y_j = y_stations(i);
        r_local = [0; y_j; 0]; 
        V_local = V_b + cross(W_b, r_local);
        u_j = V_local(1); w_j = V_local(3);
        V_mag = norm([u_j, w_j]); 
        if V_mag < 0.1, continue; end
        alpha_j = atan2(w_j, u_j);
        da_eff = 0;
        if y_j > 0.3, da_eff = -0.5 * delta_a; 
        elseif y_j < -0.3, da_eff = 0.5 * delta_a; end
        Cl_j = a0 * (alpha_j - alpha_0 + da_eff) / (1 + a0 * K_ind); 
        Cd_j = Cd0 + K_ind * Cl_j^2;
        Q_j = 0.5 * rho * V_mag^2;
        L_j = Q_j * Cl_j * chord * dy; 
        D_j = Q_j * Cd_j * chord * dy; 
        Fx_j = -D_j * cos(alpha_j) + L_j * sin(alpha_j);
        Fz_j = -L_j * cos(alpha_j) - D_j * sin(alpha_j);
        Fy_j = 0; 
        dF =[Fx_j; Fy_j; Fz_j];
        dM = cross(r_local, dF);
        F_wing = F_wing + dF;
        M_wing = M_wing + dM;
    end
end


function [F_tail, M_tail] = computeTailAero(V_b, W_b, Ctrl, rho, S, b, c, Cmq, Cnr, Clp, CmDe, CnDr, Cm0, Cma)
    V_mag = max(norm(V_b), 0.1);
    Q = 0.5 * rho * V_mag^2;
    
    alpha = atan2(V_b(3), V_b(1));
    beta = asin(V_b(2)/V_mag);
    
    delta_e = Ctrl(2); delta_r = Ctrl(3);
    
    p_hat = W_b(1) * b / (2 * V_mag);
    q_hat = W_b(2) * c / (2 * V_mag);
    r_hat = W_b(3) * b / (2 * V_mag);
    
    dCl = Clp * p_hat;
    
    % 静稳定力矩 Cm0 + Cma*alpha
    dCm = Cm0 + Cma * alpha + Cmq * q_hat + CmDe * delta_e;
    
    dCn = Cnr * r_hat + CnDr * delta_r;
    
    CYb = -0.83; 
    FY = Q * S * CYb * beta;
    
    F_tail = [0; FY; 0];
    M_tail =[Q * S * b * dCl; Q * S * c * dCm; Q * S * b * dCn];
end