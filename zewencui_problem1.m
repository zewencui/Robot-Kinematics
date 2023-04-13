% 设置时间范围和初始状态
time_span = [0 2];
initial_state = [0.05; 0; 0.05; 0];

% 定义系统参数
m1 = 7.848;
m2 = 4.49;
L1 = 0.3;
Lc1 = 0.1554;
Lc2 = 0.0341;
I1 = 0.176;
I2 = 0.0411;
kp1 = 100;
kp2 = 100;
kd1 = 20;
kd2 = 20;

% 解ODE
[t, q] = ode45(@twoLinkPlanner, time_span, initial_state);

% 定义期望轨迹
qd1 = pi/2 .* (t >= 0) - pi/2 .* (t >= 1);
qd2 = pi/2 .* (t >= 0) - pi/2 .* (t >= 1);

% 计算跟踪误差
e1 = q(:, 1) - qd1;
e2 = q(:, 3) - qd2;

% 计算关节力矩
tau1 = max(min(kp1 .* (qd1 - q(:, 1)) - kd1 .* q(:, 2), 10), -10);
tau2 = max(min(kp2 .* (qd2 - q(:, 3)) - kd2 .* q(:, 4), 10), -10);

% 绘制关节角度随时间的变化曲线
figure(1);
plot(t, q(:, 1), t, q(:, 3), "--", t, qd1, t, qd2, "--")
title("Joint Responses with PD-Control")
legend("q1(t)", "q2(t)", "qd1(t)", "qd2(t)")
xlabel("Time (sec.)")

% 绘制跟踪误差随时间的变化曲线
figure(2);
plot(t, e1, t, e2, "--")
title("Tracking Errors with PD-control")
legend("e1(t)", "e2(t)")
xlabel("Time (sec.)")
ylabel("error")

% 绘制关节力矩随时间的变化曲线
figure(3);
plot(t, tau1, t, tau2, "--")
title("Joint Torques with PD-Control")
legend("tau1(t)", "tau2(t)")
xlabel("Time (sec.)")
ylabel("Torque")