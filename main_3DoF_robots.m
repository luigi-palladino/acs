clear all;
close all;
addpath methods assignments classes simulink_models;
syms d0 d2 dB a1 a3 q1 q2 q3 m1 m2 m3 ra1 ra2 s3 L0 L1 L2 L3 real;

% DH table


DH = [  0 0 L0     0;
       L1 0  0    q1;
       L2 0  0    q2;
       0 -pi  -L3/2+q3 pi;
     ];

links = [
    Link("cyl", m1, ra1, 0, L1, [-L1/2;0;0]) 
    Link("cyl", m2, ra2, 0, L2, [-L2/2;0;0])
    Link("box", m3, s3, s3, L3, [0;0;-L3/2]) % pos o neg ?
];
myRobot = MyRobot('RRP.urdf', DH, links);

% for visual simulation it needs a variable
rbt = importrobot('RRP.urdf');

