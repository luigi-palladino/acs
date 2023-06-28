% Assignment 10
% Design the Operational Space Inverse Dynamics Control law

KD = [250;150;250;10;10;10];
KP = [1000;500;500;50;50;50];
clear qd dqd ddqd
wrongModel = 0;

% simulink trajectory
qi = [0 0 0]';
%qf = [pi -0.2 pi]';
dqi = [0 0 0]';

ti = 0;
tf = 5;
Ts = 0.001;

DimValues = 3;

DataPositions = [];
DataVelocities = [];
DataAccelerations = [];

tpts = 0:2;
tvec = ti:Ts:tf;

Value;
for i=1:DimValues
    if i==1
      %waypoints for joint 1
      wpts = [0 pi/2 pi/2; 0 pi/2 pi/2];  
    end
    if i==2
      %waypoints for joint 2
      wpts = [0 pi/2 pi/2; 0 pi/2 pi/2];  
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

dxd = timeseries(dx', tvec);

ddxd = timeseries(ddx', tvec);

%%
open('simulink_models\operational_space_inv_dyn.slx');
sim('simulink_models\operational_space_inv_dyn.slx',10);
