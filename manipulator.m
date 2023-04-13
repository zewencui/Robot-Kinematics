function out = manipulator(q)
m1 = 7.848;
m2 = 4.49;
L1 = 0.3;
Lc1 = 0.1554;
Lc2 = 0.0341;
I1 = 0.176;
I2 = 0.0411;

% Compute the elements of the manipulator inertia matrix
d11 = m1 * Lc1^2 + m2 * (L1^2 + Lc2^2 + 2 * L1 * Lc2 * cos(q(3))) + I1 + I2;
d12 = m2 * (Lc2^2 + L1 * Lc2 * cos(q(3))) + I2;
d21 = d12;
d22 = m2 * Lc2^2 + I2;

% Compute the elements of the Coriolis matrix
c121 = -m2 * L1 * Lc2 * sin(q(3));
c211 = c121;
c221 = c121;
c112 = -c221;
C = [c121, c211, c221, c112];

% Assemble the manipulator inertia matrix and Coriolis matrix into a single matrix
D = [d11, d12, d21, d22];
out = [D; C];
end