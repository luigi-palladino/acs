% Assignment 5
% Dynamic model in the operational space

disp('B op');
disp(myRobot.Ba);
disp('C op * xd');
disp(myRobot.Ca);
disp('G op');
disp(myRobot.Ga);

%%
disp("simplified")
disp('B op');
disp(simplify(myRobot.Ba));
disp('C op * xd');
disp(simplify(myRobot.Ca));
disp('G op');
disp(simplify(myRobot.Ga));

%%

myRobot.setTrajectoryPoint([pi/2;pi/2;-0.05], [0.1;0.1;0.1]);
disp('Set configuration [pi/2;pi/2;-0.05] to see some values');
disp('B op');
disp(myRobot.setValueTrajectoryPoint(myRobot.Ba));
disp('C op * xd');
disp(myRobot.setValueTrajectoryPoint(myRobot.Ca));
disp('G op');
disp(myRobot.setValueTrajectoryPoint(myRobot.Ga));


