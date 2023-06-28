function dJa = getdJa(q, dq)
q=q';
dq=dq';
q1 = q(1);
q2 = q(2);

L0 = 0.4; %base %lenght link
L1 = 0.3;
L2 = 0.3;
L3 = 0.4;

dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);

% dJa = [
% [ L2*sin(q1 + q2),  L2*sin(q1 + q2), 0]
% [-L2*cos(q1 + q2), -L2*cos(q1 + q2), 0]
% [               0,                0, 0]
% [               0,                0, 0]
% [               0,                0, 0]
% [               0,                0, 0]
% ];
dJa = [
[-(L2*cos(q1 + q2)*(dq1 + dq2)) - L1*cos(q1)*(dq1), -(L2*cos(q1 + q2)*(dq1 + dq2)), 0]
[L2*sin(q1 + q2)*(dq1 + dq2), L2*sin(q1 + q2)*(dq1 + dq2), 0]
[0, 0, 0]
[0, 0, 0]
[0, 0, 0]
[0, 0, 0]
];




end