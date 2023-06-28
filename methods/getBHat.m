function B = getBHat(q)
%myRobot.B_RNE
q1=q(1);
q2=q(2);
%lenght link
L1 = 0.3;
L2 = 0.3;
L3 = 0.4;

%mass link
m1 = 0.3;
m2 = 0.3+0.2;
m3 = 0.2+0.1;

%size
ra0 = 0.025; %base
ra1 = 0.02;
ra2 = 0.3;
s3 = 0.02;

%gravity
g=-9.81;

B = [
[(m1*(L1^2 + 3*ra1^2))/2 + (m2*(L2^2 + 3*ra2^2))/2 + m3*(((L2*cos(q1 + q2) + L1*cos(q1))*(L2*cos(conj(q1))*abs(L1)^2 + L1*cos(conj(q1) + conj(q2))*abs(L2)^2))/(L1*L2) + ((L2*sin(q1 + q2) + L1*sin(q1))*(L2*sin(conj(q1))*abs(L1)^2 + L1*sin(conj(q1) + conj(q2))*abs(L2)^2))/(L1*L2)) + m2*((((L2*cos(q1 + q2))/2 + L1*cos(q1))*(L1*cos(conj(q1) + conj(q2))*abs(L2)^2 + 2*L2*cos(conj(q1))*abs(L1)^2))/(2*L1*L2) + (((L2*sin(q1 + q2))/2 + L1*sin(q1))*(2*L2*sin(conj(q1))*abs(L1)^2 + L1*sin(conj(q1) + conj(q2))*abs(L2)^2))/(2*L1*L2)) + (m3*(L3^2 + s3^2))/12 + (m1*abs(L1)^2)/4 + (m1*cos(q1 - conj(q1))*abs(L1)^2)/4 + (m2*cos(q1 - conj(q1))*abs(L2)^2)/4, (L2^2*m2)/2 + (L3^2*m3)/12 + (3*m2*ra2^2)/2 + (m3*s3^2)/12 + (m2*cos(q1 - conj(q1))*abs(L2)^2)/4 + (m2*cos(q1 + q2 - conj(q1) - conj(q2))*abs(L2)^2)/4 + m3*cos(q1 + q2 - conj(q1) - conj(q2))*abs(L2)^2 + (L2*m2*conj(L1)*cos(q1 + q2 - conj(q1)))/2 + L2*m3*conj(L1)*cos(q1 + q2 - conj(q1)),  0]
[                                                                                                                                                                                                                                               (m2*(L2^2 + 3*ra2^2))/2 + (m3*(L3^2 + s3^2))/12 + m3*((cos(conj(q1) + conj(q2))*abs(L2)^2*(L2*cos(q1 + q2) + L1*cos(q1)))/L2 + (sin(conj(q1) + conj(q2))*abs(L2)^2*(L2*sin(q1 + q2) + L1*sin(q1)))/L2) + m2*((cos(conj(q1) + conj(q2))*abs(L2)^2*((L2*cos(q1 + q2))/2 + L1*cos(q1)))/(2*L2) + (sin(conj(q1) + conj(q2))*abs(L2)^2*((L2*sin(q1 + q2))/2 + L1*sin(q1)))/(2*L2)) + (m2*cos(q1 - conj(q1))*abs(L2)^2)/4,                                                                                       (L2^2*m2)/2 + (L3^2*m3)/12 + (3*m2*ra2^2)/2 + (m3*s3^2)/12 + (m2*cos(q1 - conj(q1))*abs(L2)^2)/4 + (m2*cos(q1 + q2 - conj(q1) - conj(q2))*abs(L2)^2)/4 + m3*cos(q1 + q2 - conj(q1) - conj(q2))*abs(L2)^2,  0]
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                                                              0, m3]
]; 
end

