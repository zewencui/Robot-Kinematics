function xdot = twoLinkPlannerP3(t, q)
% 保留原来的 kp1、kp2、kd1 和 kd2
kp1 = 100;
kp2 = 100;
kd1 = 20;
kd2 = 20;

% 计算控制相关的量
DC = manipulator(q);
d11 = DC(1,1);
d12 = DC(1,2);
d21 = DC(1,3);
d22 = DC(1,4);
c121 = DC(2,1);
c211 = DC(2,2);
c221 = DC(2,3);
c112 = DC(2,4);
T = cubicPolynomialTrajectory(t);
aq1 = T(5,:) + kp1*(T(1,:) - q(1)) + kd1*(T(3,:) - q(2));
aq2 = T(6,:) + kp2*(T(2,:) - q(3)) + kd2*(T(4,:) - q(4));
tau1 = max(min(d11*aq1 + d12*aq2 + c121*q(2)*q(4) + c211*q(4)*q(2) + c221*q(4)^2, 10), -10);
tau2 = max(min(d21*aq1 + d22*aq2 + c112*q(2)^2, 10), -10);
a1 = tau1 - c121*q(2)*q(4) - c211*q(4)*q(2) - c221*q(4)^2;
a2 = tau2 - c112*q(2)^2;
delta = d11*d22 - d12^2;
xdot = [q(2);
        1/delta*(d22*a1 - d12*a2);
        q(4);
        1/delta*(-d21*a1 + d11*a2)];
end
