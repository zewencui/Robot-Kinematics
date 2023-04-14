function xdot = twoLinkPlannerP1(t, q)
kp1 = 100;
kp2 = 100;
kd1 = 20;
kd2 = 20;

% Compute manipulator dynamics
DC = manipulator(q);

d11 = DC(1,1);
d12 = DC(1,2);
d21 = DC(1,3);
d22 = DC(1,4);

c121 = DC(2,1);
c211 = DC(2,2);
c221 = DC(2,3);
c112 = DC(2,4);

% Set desired trajectories for the joints
qd1 = pi/2 .* (t >= 0) - pi/2 .* (t >= 1);
qd2 = pi/2 .* (t >= 0) - pi/2 .* (t >= 1);

% Compute joint torques using PD-control
tau1 = max(min(kp1 * (qd1 - q(1)) - kd1 * q(2), 10), -10);
tau2 = max(min(kp2 * (qd2 - q(3)) - kd2 * q(4), 10), -10);

% Compute acceleration of each joint using computed torque control
a1 = tau1 - c121 * q(2) * q(4) - c211 * q(4) * q(2) - c221 * q(4)^2;
a2 = tau2 - c112 * q(2)^2;

% Compute the inverse of the manipulator inertia matrix
delta = d11 * d22 - d12 * d12;

% Compute the derivative of the state variables
xdot = [q(2);
1/delta * (d22 * a1 - d12 * a2);
q(4);
1/delta * (-d21 * a1 + d11 * a2)];
end