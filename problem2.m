tspan = [0, 2];
q0 = [0.05; 0; 0.05; 0];

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

% Solve the differential equation using ode45
[t, q] = ode45(@twoLinkPlannerP2, tspan, q0);

% Generate desired trajectories for the joints and their derivatives
T = cubicPolynomialTrajectory(t');
qd1 = T(1,:)';
qd2 = T(2,:)';
vd1 = T(3,:)';
vd2 = T(4,:)';
ad1 = T(5,:)';
ad2 = T(6,:)';

% Compute tracking errors
e1 = q(:,1) - qd1;
e2 = q(:,3) - qd2;

% Compute joint torques using PD-control
tau1 = max(min(ad1 + kp1 * (qd1 - q(:,1)) + kd1 * (vd1 - q(:,2)), 10), -10);
tau2 = max(min(ad2 + kp2 * (qd2 - q(:,3)) + kd2 * (vd2 - q(:,4)), 10), -10);

% Plot joint responses with PD-control
figure(1);
plot(t, q(:,1), t, q(:,3), "--", t, qd1, t, qd2, "--")
title("Joint Responses with PD-Control")
legend("q1(t)", "q2(t)", "qd1(t)", "qd2(t)")
xlabel("Time (sec.)")

% Plot tracking errors with PD-control
figure(2);
plot(t, e1, t, e2, "--")
title("Tracking Errors with PD-control")
legend("e1(t)", "e2(t)")
xlabel("Time (sec.)")
ylabel("error")

% Plot joint torques with PD-control
figure(3);
plot(t, tau1, t, tau2, "--")
title("Joint Torques with PD-Control")
legend("tau1(t)", "tau2(t)")
xlabel("Time (sec.)")
ylabel("Torque")



