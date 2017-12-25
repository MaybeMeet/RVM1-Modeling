function [posArray, time] = RVM1_iksolve(robot, startPoint, endPoint, referenceEndPose)

p1 = startPoint;
p2 = endPoint;
line_dir = (p2-p1);
H = trvec2tform([0 0 0]) * eul2tform([0 0 pi], 'ZYX');

resolution = 0.1;
index = 1;

initialguess = robot.homeConfiguration;
field1 = 'MaxIterations';  value1 = 1500;
field2 = 'SolutionTolerance'; value2 = 0.01;
s = struct(field1, value1, field2, value2);

ik = robotics.InverseKinematics('RigidBodyTree',robot, 'SolverParameters', s,'SolverAlgorithm','LevenbergMarquardt');
%ik = robotics.InverseKinematics('RigidBodyTree',robot, 'SolverParameters', s);
%we can use either of above commands depending on algorithm needed

points = traj_line(startPoint, endPoint, resolution); 
%define points from the line equation

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
    
 % sometimes it takes lot of time , for observation purposes we can use
 % time as calculated above. we can find time needed for different
 % algorithm
%     jointValues = [configSol(1).JointPosition, configSol(2).JointPosition, configSol(3).JointPosition,...
%                                configSol(4).JointPosition, configSol(5).JointPosition, configSol(6).JointPosition];
end