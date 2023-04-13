function xdot = twoLinkPlannerP2(t, q)
% Define PD-control gains
kp1 = 100;
kp2 = 100;
kd1 = 20;
kd2 = 20;

% Compute manipulator dynamics
DC = manipulator(q);

% Extract elements of the manipulator inertia matrix and Coriolis matrix
d11 = DC(1, 1);
d12 = DC(1, 2);
d21 = DC(1, 3);
d22 = DC(1, 4);
c121 = DC(2, 1);
c211 = DC(2, 2);
c221 = DC(2, 3);
c112 = DC(2, 4);

% Generate desired trajectories for the joints and their derivatives
T = cubicPolynomialTrajectory(t);

% Compute joint torques using PD-control
tau1 = max(min(T(5,:) + kp1 * (T(1,:) - q(1)) + kd1 * (T(3,:) - q(2)), 10), -10);
tau2 = max(min(T(6,:) + kp2 * (T(2,:) - q(3)) + kd2 * (T(4,:) - q(4)), 10), -10);

% Compute acceleration of each joint using computed torque control
a1 = tau1 - c121 * q(2) * q(4) - c211 * q(4) * q(2) - c221 * q(4)^2;
a2 = tau2 - c112 * q(2)^2;

% Compute the inverse of the manipulator inertia matrix
delta = d11 * d22 - d12 * d21;

% Compute the derivative of the state variables
xdot = [q(2);
1/delta * (d22 * a1 - d12 * a2);
q(4);
1/delta * (-d21 * a1 + d11 * a2)];
end



