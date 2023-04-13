% Define time span and initial joint positions and velocities
tspan = [0 2];
q0 = [0.05; 0; 0.05; 0];

% Define robot arm parameters
robot_arm.masses = [7.848, 4.49];
robot_arm.lengths = [0.3, 0.0341];
robot_arm.centers_of_mass = [0.1554, 0.0341];
robot_arm.moments_of_inertia = [0.176, 0.0411];
robot_arm.gravity = 9.81;

% Define PD controller gains
controller.kp = [100, 100];
controller.kd = [20, 20];
% Define PD controller gains
controller.kp1 = 100;
controller.kp2 = 100;
controller.kd1 = 20;
controller.kd2 = 20;
controller.qd1 = pi/2 .* (t >= 0) - pi/2 .* (t >= 1);
controller.qd2 = pi/2 .* (t >= 0) - pi/2 .* (t >= 1);


% Solve for joint positions and velocities using ode45
[t,q] = ode45(@(t, q) twoLinkPlanner(t, q, robot_arm, controller), tspan, q0);

% Define desired joint angles
qd1 = pi/2 .* (t >= 0) - pi/2 .* (t >= 1);
qd2 = pi/2 .* (t >= 0) - pi/2 .* (t >= 1);

% Calculate tracking errors and joint torques
e1 = q(:, 1) - qd1;
e2 = q(:, 3) - qd2;
tau1 = max(min(controller.kp(1) .* (qd1 - q(:, 1)) - controller.kd(1) .* q(:, 2), 10), -10);
tau2 = max(min(controller.kp(2) .* (qd2 - q(:, 3)) - controller.kd(2) .* q(:, 4), 10), -10);

% Plot joint responses with PD control
figure(1);
plot(t, q(:, 1), t, q(:, 3), "--", t, qd1, t, qd2, "--")
title("Joint Responses with PD-Control")
legend("q1(t)", "q2(t)", "qd1(t)", "qd2(t)")
xlabel("Time (sec.)")

% Plot tracking errors with PD control
figure(2);
plot(t, e1, t, e2, "--")
title("Tracking Errors with PD-control")
legend("e1(t)", "e2(t)")
xlabel("Time (sec.)")
ylabel("error")

% Plot joint torques with PD control
figure(3);
plot(t, tau1, t, tau2, "--")
title("Joint Torques with PD-Control")
legend("tau1(t)", "tau2(t)")
xlabel("Time (sec.)")
ylabel("Torque")

function qdot = twoLinkPlanner(t, q, robot_arm, controller)
% Compute qdot given t, q, robot_arm parameters and controller gains
qd1 = controller.qd1;
qd2 = controller.qd2;

% Extract joint positions and velocities from input q
q1 = q(1);
q2 = q(3);
dq1 = q(2);
dq2 = q(4);

% Compute M, C, and G matrices
[m1, m2] = deal(robot_arm.masses(1), robot_arm.masses(2));
[L1, L2] = deal(robot_arm.lengths(1), robot_arm.lengths(2));
[Lc1, Lc2] = deal(robot_arm.centers_of_mass(1), robot_arm.centers_of_mass(2));
[I1, I2] = deal(robot_arm.moments_of_inertia(1), robot_arm.moments_of_inertia(2));
g = robot_arm.gravity;

M = [I1 + I2 + m2*L1^2 + 2*m2*L1*Lc2*cos(q2), I2 + m2*L1*Lc2*cos(q2); ...     
    I2 + m2*L1*Lc2*cos(q2), I2];
C = [-m2*L1*Lc2*sin(q2)*dq2^2 - 2*m2*L1*Lc2*sin(q2)*dq1*dq2; ...     
    m2*L1*Lc2*sin(q2)*dq1^2];
G = [(m1*Lc1 + m2*L1)*g*cos(q1) + m2*Lc2*g*cos(q1+q2); ...     
    m2*Lc2*g*cos(q1+q2)];

% Compute joint torques using PD control
tau1 = controller.kp(1) * (qd1 - q1) - controller.kd(1) * dq1;
tau2 = controller.kp(2) * (qd2 - q2) - controller.kd(2) * dq2;

% Compute qdot
qdot = [dq1; M \ (tau1 - C - G); dq2; M \ (tau2 - C)];
end
