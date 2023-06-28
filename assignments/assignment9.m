% Assignment 9
% Design the Operational Space PD control law with gravity compensation

% KD = [80;20;50;10;10;10];
% KP = [10;50;50;50;50;50];
KD = [80;80;80;10;10;10];
KP = [1000;1000;1000;50;50;50];

Value;
%gravity
g_q = [0;0;-g*m3];

% simulink trajectory
qi = [0 0 0]';
qf = [pi/2 -pi/2 -0.1]'; %not join positions!
dqi = [0 0 0]';
ti = 0;
tf = 10;
Ts = 0.001;

xi = getK(qi);
xd = getK(qf);

open('simulink_models\operational_space_pd_w_g_comp.slx');
sim('simulink_models\operational_space_pd_w_g_comp.slx');
