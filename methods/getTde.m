function Tde = getTde(q, Td)
% get transform from desired frame to end effector frame
q1 = q(1); 
q2 = q(2); 
q3 = q(3);
L0 = 0.4; %base %lenght link
L1 = 0.3;
L2 = 0.3;
L3 = 0.4;

%mass link
m1 = 0.3;
m2 = 0.2;
m3 = 0.1;

%size
ra0 = 0.025; %base
ra1 = 0.02;
ra2 = 0.3;
s3 = 0.02;

%gravity
g=-9.81;

%correct
% Te = [
% [-cos(q1 + q2), -sin(q1 + q2),  0, L2*cos(q1 + q2) + L1*cos(q1)]
% [-sin(q1 + q2),  cos(q1 + q2),  0, L2*sin(q1 + q2) + L1*sin(q1)]
% [            0,             0, -1,               L0 - L3/2 + q3]
% [            0,             0,  0,                            1]
% ];

Te = [
[-cos(q1 + q2), -sin(q1 + q2),  0, (3*cos(q1 + q2))/10 + (3*cos(q1))/10]
[-sin(q1 + q2),  cos(q1 + q2),  0, (3*sin(q1 + q2))/10 + (3*sin(q1))/10]
[            0,             0, -1,                             q3 + 1/5]
[            0,             0,  0,                                    1]
];

%Tde = inv(Td)*Te;
Tde = Td/Te;

end 