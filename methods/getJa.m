function Ja = getJa(q)
%q2 = q(2);
q=q';
q1 = q(1);
q2 = q(2);
L0 = 0.4; %base %lenght link
L1 = 0.3;
L2 = 0.3;
L3 = 0.4;

Ja = [
[- L1*sin(q1) - L2*cos(q1)*sin(q2) - L2*cos(q2)*sin(q1), - L2*cos(q1)*sin(q2) - L2*cos(q2)*sin(q1), 0]
[  L1*cos(q1) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2),   L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2), 0]
[                                                     0,                                         0, 1]
[                                                     1,                                         1, 0]
[                                                     0,                                         0, 0]
[                                                     0,                                         0, 0]
];



end

