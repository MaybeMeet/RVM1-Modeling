%This program will run all the codes and follow the line as per 
%starting and end point given to it 

theta_1 = 0;
theta_2 = pi/4;
theta_3 = -pi/3;
theta_4 = 0;
theta_5  = pi/4;
theta_6 = 0;

startPoint = [1.1,0,1.3];
endPoint = [2.3,0,3];

robot = RVM1Frames(theta_1, theta_2, theta_3, theta_4, theta_5);

robotConfig = robot.homeConfiguration();
robotConfig(1).JointPosition = theta_1;
robotConfig(2).JointPosition = theta_2;
robotConfig(3).JointPosition = theta_3;
robotConfig(4).JointPosition = theta_4;
robotConfig(5).JointPosition = theta_5;
endEffectorPose = getTransform(robot, robotConfig, 'endeffector');

[posArray, time] = RVM1_iksolve(robot, startPoint, endPoint, endEffectorPose);

plot_inv(posArray, startPoint, endPoint);


