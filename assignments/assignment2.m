% Assignment 2
% Compute the kinetic energy

disp('Kinetic energy');
disp(simplify(myRobot.kineticEnergy()));

disp('Potential Energy');
disp(myRobot.potentialEnergy);

%%
myRobot.setTrajectoryPoint([pi/2;pi/2;-0.05], [0.1;0.1;0.1]);
disp('Set configuration [pi/2;pi/2;-0.05] to see some values');
disp('Kinetic energy');
disp(myRobot.setValueTrajectoryPoint(myRobot.kineticEnergy()));
disp('Potential Energy');
disp(myRobot.setValues(myRobot.potentialEnergy()));