% Assignment12
% Implement the impedance control in the operational space.

KD = [40;40;40;10;10;10];
KP = [50;50;50;50;50;50];
Md = diag([0.3;0.2;0.1;1;1;1]);
invMd = inv(Md);
Value;
g_q = [0;0;-g*m3];
K = diag([1 1 100 1 1 1]);

qi = [0 0 0]';
dqi = [0;0;0];
%qf = [0 -0.2 0]';
qr = [0 0 -0.1]';

xr = getK(qr);

ti = 0;
tf = 5;
Ts = 0.001;

tvec = ti:Ts:tf;
tpts = 0:2;
DimValues = 3;

DataPositions = [];
DataVelocities = [];
DataAccelerations = [];


for i=1:DimValues
    if i==1
      %waypoints for joint 1
      wpts = [0 0 0; 0 0 0];  
    end
    if i==2
      %waypoints for joint 2
      wpts = [0 0 0; 0 0 0];  
    end
    if i==3
      %waypoints for joint 3
      wpts = [0 -0.2 -0.2; 0 -0.2 -0.2];  
    end
    
    [q, dq, ddq, pp] = cubicpolytraj(wpts, tpts, tvec);
    % x = K(q)
    % dx = J(q)dq
    % ddx = dJ(q,dq)dq + J(q)ddq

    q=q(1,:);
    dq=dq(1,:);
    ddq=ddq(1,:);
    

    DataPositions(i, :) = q;
    DataVelocities(i, :) = dq;
    DataAccelerations(i, :) = ddq;
end

sz = size(DataPositions);

x = [];
dx = [];
ddx = [];
for i=1:sz(2) 
    x(:,i) = getK(DataPositions(:,i));
    dx(:,i) = getJa(DataPositions(:,i))*DataVelocities(:, i);
    ddx(:,i) = getdJa(DataPositions(:,i),DataVelocities(:, i))*DataVelocities(:, i) + getJa(DataPositions(:,i))*DataAccelerations(:, i);
end 


xd = timeseries(x', tvec);
% 
dxd = timeseries(dx', tvec);
% 
ddxd = timeseries(ddx', tvec);
%%
open('simulink_models\impedance_control.slx');
sim('simulink_models\impedance_control.slx');
