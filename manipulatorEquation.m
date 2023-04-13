function [D, C] = manipulatorEquation(q, qd, m, lc, L, I)

m1 = m(1);
m2 = m(2);
L1 = L(1);
Lc1 = lc(1);
Lc2 = lc(2);
I1 = I(1);
I2 = I(2);

g = 9.81; % 重力加速度

% 计算机械臂的运动学参数
theta1 = q(1);
theta2 = q(2);
dtheta1 = qd(1);
dtheta2 = qd(2);

s1 = sin(theta1);
c1 = cos(theta1);
s2 = sin(theta2);
c2 = cos(theta2);

% 计算动力学系数矩阵D
d11 = m1*Lc1^2 + m2*(L1^2 + Lc2^2 + 2*L1*Lc2*c2) + I1 + I2;
d12 = m2*(Lc2^2 + L1*Lc2*c2) + I2;
d21 = d12;
d22 = m2*Lc2^2 + I2;

D = [d11, d12;     d21, d22];

% 计算科氏力矩向量C
c121 = -m2*L1*Lc2*s2*(2*dtheta1 + dtheta2)*(dtheta2 + dtheta1);
c211 = -m2*L1*Lc2*s2*dtheta1^2;
c221 = 0.5*m2*Lc2^2*s2*dtheta2^2;
c112 = m2*L1*Lc2*s2*dtheta2^2;
C = [c121+c211; c112+c221];

% 将科氏力矩向量加入动力学系数矩阵中
G = [m1*Lc1 + m2*L1*c1 + m2*Lc2*c1 + 0.5*m2*L1*c1*c2 + 0.5*m2*Lc2*c1*c2;     m2*Lc2 + m2*L1*c2 + 0.5*m2*Lc2*c2] * g;
DC = [C, [0; 0; 0]; zeros(1, 3), 1];

end
