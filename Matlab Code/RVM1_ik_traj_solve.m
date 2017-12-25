function [posArray, time] = RVM1_ik_traj_solve(robot, startPoint, endPoint, referenceEndPose)

x=[13 14 6  2  5 20 42 49 39 25 22 30];
y=[48 31 7 13 19 12 16 39 61 66 40  2];
n=length(x);
t=1:n;
tt=linspace(t(1),t(n),50);
xx=spline(t,x,tt)/40;
yy=spline(t,y,tt)/40;

H = trvec2tform([0 0 0]) * eul2tform([0 0 pi], 'ZYX');
index = 1;

initialguess = robot.homeConfiguration;
field1 = 'MaxIterations';  value1 = 1500;
field2 = 'SolutionTolerance'; value2 = 0.01;
s = struct(field1, value1, field2, value2);

ik = robotics.InverseKinematics('RigidBodyTree',robot, 'SolverParameters', s,'SolverAlgorithm','LevenbergMarquardt');
%ik = robotics.InverseKinematics('RigidBodyTree',robot, 'SolverParameters', s);
%we can use either of above commands depending on algorithm needed

%points = traj_line(startPoint, endPoint, resolution); 
%define points fromthe line equation


%plot(xx,yy,'b')
z=zeros(size(xx));
plot3(xx,z,yy,'b')
hold on;
points =[xx'+ 1,z',yy'+3];

%points =[ xx',yy',tt'];

posArray = zeros(6,3,size(points,1));
time_each = zeros(size(points,1),1);

m = size(points,1);

for i = 1:m
    point = points(i,:);
    H(1:3,4) = point';
    %weights = ones(1,6);
    weights = [0.1 0.1 0.1 1.2 1.2 1];
    tform = H;
    
   [configSol, time_each(index)] = solve_ik(ik, tform, weights, initialguess);
   
   jointValues = [configSol(1).JointPosition, configSol(2).JointPosition, configSol(3).JointPosition,...
                           configSol(4).JointPosition, configSol(5).JointPosition];
    pos = RVM1_fk( jointValues(1), jointValues(2), jointValues(3), jointValues(4), jointValues(5)); 
  
    posArray(:,:,index) = pos(1:6,:);
    index = index + 1;
    initialguess = configSol;
end
time = sum(time_each);

posArray(6,1,:)= posArray(6,1,:)-0.72;
posArray(6,3,:)= posArray(6,3,:)-0.72;
% 
end
function [configSol, time] = solve_ik(ik, tform, weights, initialguess)
    tic
    [configSol,~] = step(ik,'endeffector',tform,weights,initialguess);
    time = toc;
    
%     jointValues = [configSol(1).JointPosition, configSol(2).JointPosition, configSol(3).JointPosition,...
%                                configSol(4).JointPosition, configSol(5).JointPosition, configSol(6).JointPosition];
end