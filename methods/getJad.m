function Jad = getJad(q, xTilde, Td)
%get transform from desired frame to end effector frame 

q1 = q(1);
q2 = q(2);
L0 = 0.4; %base %lenght link
L1 = 0.3;
L2 = 0.3;
L3 = 0.4;

Tde = getTde(q, Td);
%slide 29, robotics Kinematics
angles = tform2eul(Tde, 'ZYZ');
phi =   angles(1);
theta = angles(2);

% Rd = Td(1:3, 1:3);
Rd = eul2rotm(xTilde(4:6)');
%correct
Ta = [
[1, 0, 0, 0,         0,                   0]
[0, 1, 0, 0,         0,                   0]
[0, 0, 1, 0,         0,                   0]
[0, 0, 0, 0, -sin(phi), cos(phi)*sin(theta)]
[0, 0, 0, 0,  cos(phi), sin(phi)*sin(theta)]
[0, 0, 0, 1,         0,          cos(theta)]    
];

%correct
J = [
[- L1*sin(q1) - L2*cos(q1)*sin(q2) - L2*cos(q2)*sin(q1), - L2*cos(q1)*sin(q2) - L2*cos(q2)*sin(q1), 0]
[  L1*cos(q1) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2),   L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2), 0]
[                                                     0,                                         0, 1]
[                                                     1,                                         1, 0]
[                                                     0,                                         0, 0]
[                                                     0,                                         0, 0]
];

% R matrix slide 14 
R = [Rd' zeros(3,3)
    zeros(3,3) Rd'];

Jad = pinv(Ta)*R*J;