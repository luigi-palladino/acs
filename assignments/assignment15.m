% Assignement 15
% Implement the Parallel Force/Position Control.

KD = [50;50;30;10;10;10];
KP = [100;100;50;20;20;20];
Md = diag([0.3;0.2;0.1;1;1;1]);
KI = 10;
KF = 5;

invMd = inv(Md);
Value;
g_q = [0;0;-g*m3];

K = diag([1 1 10 1 1 1]);
fd = [0 0 -0.1 0 0 0]';

qi = [0 0 0]';
dqi = [0;0;0];
qf = [0 0 -0.2]';
qr = [0 0 -0.1];

Ts = 0.001;

xi = getK(qi);
xd = getK(qf);
xr = getK(qr);

open('simulink_models\parallel_force_pos_control.slx');
sim('simulink_models\parallel_force_pos_control.slx');
%%
KI = 5;
KF = 5;
%open('simulink_models\parallel_force_pos_control.slx');
sim('simulink_models\parallel_force_pos_control.slx');