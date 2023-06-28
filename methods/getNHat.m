function n = getNHat(q, dq)
%myRobot.setValues(myRobot.C*myRobot.dq+myRobot.G) 
q1 = q(1);
q2 = q(2);
dq1 = dq(1);
dq2 = dq(2);
%lenght link
L1 = 0.3;
L2 = 0.3;
L3 = 0.4;

%mass link
m1 = 0.3;
m2 = 0.2+0.5;
m3 = 0.1+0.1;

%gravity
g=-9.81;

n=[ 
-(dq2*(m2 + 2*m3)*((L1*dq1*sin(conj(q1) - q1 + conj(q2))*abs(L2)^2)/L2 + (L2*dq1*abs(L1)^2*sin(q1 + q2 - conj(q1)))/L1 + (L2*dq2*abs(L1)^2*sin(q1 + q2 - conj(q1)))/L1))/2
                                                   (dq1*dq2*(m2 + 2*m3)*(L2^2*abs(L1)^2*sin(q1 + q2 - conj(q1)) - L1^2*sin(conj(q1) - q1 + conj(q2))*abs(L2)^2))/(4*L1*L2)
                                                                                                                                                                     -g*m3
];    
 
 

end

