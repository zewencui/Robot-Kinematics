function out=cubicPolynomialTrajectory(t)
    A1=[0,0,0,1;1,1,1,1;0,0,1,0;3,2,1,0];
    A2=[8,4,2,1;1,1,1,1;12,4,1,0;3,2,1,0];
    b1=[0,pi/2,0,0]';
    b2=[0,pi/2,0,0]';
    x1=A1\b1;
    x2=A2\b2;
    qd1 = zeros(size(t));
    qd2 = zeros(size(t));
    vd1 = zeros(size(t));
    vd2 = zeros(size(t));
    ad1 = zeros(size(t));
    ad2 = zeros(size(t));
    
    % Compute qd1, qd2, vd1, vd2, ad1, ad2 using a loop
    for i = 1:length(t)
        if t(i) < 1
            qd1(i) = x1(1)*t(i)^3 + x1(2)*t(i)^2 + x1(3)*t(i) + x1(4);
            qd2(i) = qd1(i);
            vd1(i) = 3*x1(1)*t(i)^2 + 2*x1(2)*t(i) + x1(3);
            vd2(i) = vd1(i);
            ad1(i) = 6*x1(1)*t(i) + 2*x1(2);
            ad2(i) = ad1(i);
        else
            qd1(i) = x2(1)*t(i)^3 + x2(2)*t(i)^2 + x2(3)*t(i) + x2(4);
            qd2(i) = qd1(i);
            vd1(i) = 3*x2(1)*t(i)^2 + 2*x2(2)*t(i) + x2(3);
            vd2(i) = vd1(i);
            ad1(i) = 6*x2(1)*t(i) + 2*x2(2);
            ad2(i) = ad1(i);
        end
    end
    out=[qd1;qd2;vd1;vd2;ad1;ad2];
end
