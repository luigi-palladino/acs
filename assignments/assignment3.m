%Assignment3 
%Equation of motion
myRobot.computeLagrangian();
disp('B');
disp(myRobot.B);
disp('C');
disp(myRobot.C*myRobot.dq);
disp('G');
disp(myRobot.G);
disp('TAU');
disp(myRobot.TAU);

%%
myRobot.setTrajectoryPoint([pi/2;pi/2;-0.05], [0.1;0.1;0.1]);

disp('Set configuration [pi/2;pi/2;-0.5] to see some values');
disp('B');
disp(myRobot.setValueTrajectoryPoint(myRobot.B));
disp('C');
disp(myRobot.setValueTrajectoryPoint(myRobot.C*myRobot.dq));
disp('G');
disp(myRobot.setValueTrajectoryPoint(myRobot.G));
disp('TAU');
disp(myRobot.setValueTrajectoryPoint(myRobot.TAU));


