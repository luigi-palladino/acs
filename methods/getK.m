function k = getK(q)
%myRobot.setValues(myRobot.allT(:,:,end))
q1 = q(1); q2 = q(2); q3 = q(3);

L0 = 0.4; %base %lenght link
L1 = 0.3;
L2 = 0.3;
L3 = 0.4;

%mass link
m1 = 0.3;
m2 = 0.2;
m3 = 0.1;

%size
ra0 = 0.025; %base
ra1 = 0.02;
ra2 = 0.3;
s3 = 0.02;

%gravity
g=-9.81;


 T = [
   sin(q1)*sin(q2) - cos(q1)*cos(q2), - cos(q1)*sin(q2) - cos(q2)*sin(q1),  0, L1*cos(q1) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2);
 - cos(q1)*sin(q2) - cos(q2)*sin(q1),   cos(q1)*cos(q2) - sin(q1)*sin(q2),  0, L1*sin(q1) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1);
                                   0,                                   0, -1,                                       L0 - L3/2 + q3;
                                  0,                                   0,  0,                                                    1
];

% T = [
% [  sin(q1)*sin(q2) - cos(q1)*cos(q2), - cos(q1)*sin(q2) - cos(q2)*sin(q1),  0, (3*cos(q1))/10 + (3*cos(q1)*cos(q2))/10 - (3*sin(q1)*sin(q2))/10]
% [- cos(q1)*sin(q2) - cos(q2)*sin(q1),   cos(q1)*cos(q2) - sin(q1)*sin(q2),  0, (3*sin(q1))/10 + (3*cos(q1)*sin(q2))/10 + (3*cos(q2)*sin(q1))/10]
% [                                  0,                                   0, -1,                                                         q3 + 1/5]
% [                                  0,                                   0,  0,                                                                1]
% ];
 
p = T(1:3, 4);
% rotation R to euler angles
 phi = [
     atan2(T(2,3), T(1,3));
     atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));
     atan2(T(3,2), -T(3,1));
 ];
% x = L2*cos(q1 + q2) + L1*cos(q1);
% y = L2*sin(q1 + q2) + L1*sin(q1);
% z = L0 - L3/2 + q3;
% q1 = atan2(y, x) - atan2(L2*sin(q2), L1 + L2*cos(q2));
% q2 = -acos(max(min((x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2),1),-1));
% q3 = z - L0 + L3/2;
% phi = [q1;q2;q3];
% p = [x;y;z];
k = [p;phi];
end

