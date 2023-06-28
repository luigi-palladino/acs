function j = getDiffJa(q, dq)
q1 = q(1);
q2 = q(2);
dq1 = dq(1);
dq2 = dq(2);

L1 = 0.3;
L2 = 0.3;


j = [
[-(L2*cos(q1 + q2)*(dq1 + dq2)) - L1*cos(q1)*(dq1), -(L2*cos(q1 + q2)*(dq1 + dq2)), 0]
[L2*sin(q1 + q2)*(dq1 + dq2), L2*sin(q1 + q2)*(dq1 + dq2), 0]
[0, 0, 0]
[0, 0, 0]
[0, 0, 0]
[0, 0, 0]
];  

end

