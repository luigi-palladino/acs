% Assignment 1
% DH table
% direct kinematics
% inverse kinematics
% Jacobians (geometric and analytical)
% By hand, and cross-checking with Robotics toolbox

%%
%showdetails(myRobot.robot);
%config = homeConfiguration(myRobot.robot);
%myRobot.setAllConfig(config);
%myRobot.setAllConfig([pi/2;pi/2;-0.05]);
%show(myRobot.robot,config);
%myRobot.show
%%
close all;
disp('DH Table');
disp(myRobot.DH);
disp('Direct Kinematics');
disp(simplify(myRobot.allT(:,:,end)));
%%
disp('Inverse Kinematics');
disp(ik());

disp('Geometric Jacobian');
disp(simplify(myRobot.computeJ));
disp('Analytical Jacobian');
disp(myRobot.Ja);


myRobot.setAllConfig([pi/2;-pi/2;-0.05]);

myRobot.show
disp('Set configuration [pi/2;-pi/2;-0.05] to cross check with toolbox');
disp('Direct kinematics toolbox');
disp(myRobot.toolboxT);
disp('Direct kinematics computed');
disp(myRobot.setValues(myRobot.allT(:,:,end)));
disp('Inverse kinematics toolbox');
disp(myRobot.toolboxIk);
disp('Inverse kinematics computed')
disp(myRobot.setValues(myRobot.inverseKinematics'));
disp('Geometric Jacobian toolbox');
disp(myRobot.toolboxJg);
disp('Geometric Jacobian computed')
disp(myRobot.setValues(myRobot.J));
disp('Analitical Jacobian toolbox');
disp(myRobot.toolboxJa);
disp('Analytical Jacobian');
disp(simplify(myRobot.Ja));
disp('Analytical Jacobian computed');
disp(myRobot.setValues(myRobot.Ja));

