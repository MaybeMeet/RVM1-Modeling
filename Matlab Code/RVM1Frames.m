function [robot] = RVM1Frames(theta_1, theta_2, theta_3, theta_4, theta_5)
% Function plot_manip will plot the manipulator based of DH parameters
% using functions from Robotics System toolbox. 
% The function essentially helps in the visualization of the frames

robot = robotics.RigidBodyTree;

%% DH Params
dhparams = [ 0      pi/2    1.52        theta_1;
             2.50      0          0     theta_2;
             1.60     0        0        theta_3;
             0      pi/2        0      theta_4+(pi/2);
             0      0         .72       theta_5-(pi/2)];


%% Create the Rigid body chain
%used matlab official site for reference

body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');
setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;

body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
bodyEndEffector = robotics.RigidBody('endeffector');


setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');


body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
bodyEndEffector.Joint = jnt5;

addBody(robot,body1,'base')
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,bodyEndEffector,'body4')

% Display
figure
h1 = axes;
% Create the configuration for the Robot
config = homeConfiguration(robot);

% Assign the theta parameters to the config struct
config(1).JointPosition = dhparams(1,4);
config(2).JointPosition = dhparams(2,4);
config(3).JointPosition = dhparams(3,4);
config(4).JointPosition = dhparams(4,4);
config(5).JointPosition = dhparams(5,4);

show(robot, config);
hold on 
% Flip the axes to make our frame assignment consistent with the MATLAB
% convention
set(h1, 'Ydir', 'reverse')
set(h1, 'Xdir', 'reverse')
hold on
