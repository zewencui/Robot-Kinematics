function [t,q,DC,T] = simulation()
tspan = [0 2];
q0=[0.05;0;0.05;0];

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

[t,q]=ode45(@twoLinkPlannerP3,tspan,q0);
DC=manipulator(q);
T=cubicPolynomialTrajectory(t');

e1=q(:,1)-T(1,:)';
e2=q(:,3)-T(2,:)';

d11=DC(1,1);
d12=DC(1,2);
d21=DC(1,3);
d22=DC(1,4);

c121=DC(2,1);
c211=DC(2,2);
c221=DC(2,3);
c112=DC(2,4);

tau1=max(min(d11.*T(5,:)'+d12.*T(6,:)'+c121.*q(:,2).*q(:,4)+c211.*q(:,4).*q(:,2)+c221.*q(:,4).^2,10),-10);
tau2=max(min(d21.*T(5,:)'+d22.*T(6,:)'+c112.*q(:,2).^2,10),-10);

figure(1);
plot(t,q(:,1),t,q(:,3),"--",t,T(1,:),t,T(2,:),"--")
title("Joint Responses with PD-Control")
legend("q1(t)","q2(t)","qd1(t)","qd2(t)")
xlabel("Time (sec.)")

figure(2);
plot(t,e1,t,e2,"--")
title("Tracking Errors with PD-control")
legend("e1(t)","e2(t)")
xlabel("Time (sec.)")
ylabel("error")

figure(3);
plot(t,tau1,t,tau2,"--")
title("Joint Torques with PD-Control")
legend("tau1(t)","tau2(t)")
xlabel("Time (sec.)")
ylabel("Torque")

end
