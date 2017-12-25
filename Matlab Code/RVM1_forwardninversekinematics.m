%RVM1_forwardninversekinematics()
theta_1 = 0;
theta_2 = pi/6;
theta_3 = pi/2;
theta_4 = pi/3;
theta_5  = 0;

% robot = RVM1Frames(theta_1, theta_2, theta_3, theta_4, theta_5);
% %show(robot);
% hold on;
pos = RVM1_fk(theta_1, theta_2, theta_3, theta_4, theta_5);
plot_robot_using_DH(pos);
grid on;
%err = compare_DH_toolbox(robot, pos);
hold on

