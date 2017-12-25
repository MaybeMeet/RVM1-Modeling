function err = compare_DH_toolbox(robot, pos)
% function compare.m takes the rigid body tree robot and 11x3 position
% 
theta_1 = 0;
theta_2 = pi/4;
theta_3 = pi/3;
theta_4 = pi/4;
theta_5  = 0;
pos = RVM1_fk(theta_1, theta_2, theta_3, theta_4, theta_5);

robot = RVM1Frames(theta_1, theta_2, theta_3, theta_4, theta_5);
configuration = randomConfiguration(robot);


configuration(1).JointPosition = theta_1;
configuration(2).JointPosition = theta_2;
configuration(3).JointPosition = theta_3;
configuration(4).JointPosition = theta_4;
configuration(5).JointPosition = theta_5;

transform = getTransform(robot,configuration,'endeffector');
err=norm(transform(1:3,4)'-pos(6,:))

%err = 0;

end