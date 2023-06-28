function [config1, config2] = ik()
    syms q1 q2 L0 L1 L2 q3 x y z L3 'real'
    x = L2*cos(q1 + q2) + L1*cos(q1);
    y = L2*sin(q1 + q2) + L1*sin(q1);
    z = L0 - L3/2 + q3;
    q1 = atan2(y, x) - atan2(L2*sin(q2), L1 + L2*cos(q2));
    q2 = -acos((x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2));
    q3 = z - L0 + L3/2;
    config1 = [q1,q2,q3];
    config2 = config1;
end

