function j = getInvJ(q)
%myRobot.setValues(pinv(myRobot.J))
L1 = 0.3;
L2 = 0.3;

q1 = q(1);
q2 = q(2);

j=[
[                                                      -(sin(conj(q1)) - sin(conj(q2))*abs(L2)^2*cos(q1)*cos(q2) + sin(conj(q2))*abs(L2)^2*sin(q1)*sin(q2))/(L1*sin(conj(q2))*sin(q2)*abs(L2)^2 + L1*cos(conj(q1))*cos(q1) + L1*sin(conj(q1))*sin(q1)),                                                         (cos(conj(q1)) + sin(conj(q2))*abs(L2)^2*cos(q1)*sin(q2) + sin(conj(q2))*abs(L2)^2*cos(q2)*sin(q1))/(L1*sin(conj(q2))*sin(q2)*abs(L2)^2 + L1*cos(conj(q1))*cos(q1) + L1*sin(conj(q1))*sin(q1)), 0, 0, 0,                        -(L2*cos(q1 + q2 - conj(q1)))/(L1*sin(conj(q2))*sin(q2)*abs(L2)^2 + L1*cos(q1 - conj(q1)))]
[(L2*sin(conj(q1)) - L1*sin(conj(q2))*abs(L2)^2*cos(q1) - L2*sin(conj(q2))*abs(L2)^2*cos(q1)*cos(q2) + L2*sin(conj(q2))*abs(L2)^2*sin(q1)*sin(q2))/(L1*L2*sin(conj(q2))*sin(q2)*abs(L2)^2 + L1*L2*cos(conj(q1))*cos(q1) + L1*L2*sin(conj(q1))*sin(q1)), -(L2*cos(conj(q1)) + L1*sin(conj(q2))*abs(L2)^2*sin(q1) + L2*sin(conj(q2))*abs(L2)^2*cos(q1)*sin(q2) + L2*sin(conj(q2))*abs(L2)^2*cos(q2)*sin(q1))/(L1*L2*sin(conj(q2))*sin(q2)*abs(L2)^2 + L1*L2*cos(conj(q1))*cos(q1) + L1*L2*sin(conj(q1))*sin(q1)), 0, 0, 0, (L1*cos(q1 - conj(q1)) + L2*cos(q1 + q2 - conj(q1)))/(L1*sin(conj(q2))*sin(q2)*abs(L2)^2 + L1*cos(q1 - conj(q1)))]
[                                                                                                                                                                                                                                                    0,                                                                                                                                                                                                                                                      0, 1, 0, 0,                                                                                                                 0]
];

end