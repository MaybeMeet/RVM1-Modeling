function [points] = traj_line(startPoint, endPoint, resolution)
% Function traj_line_generator.m generates points on a 
% straight line between start and end points at a specific resolution 
% Resolution is a fraction value between 0 and 1
%n is number of points

% startPoint = [7, 15, 25];
% endPoint = [27, 0, 0];
% resolution = 0.1;

p1 = startPoint;
p2 = endPoint;
line_dir = (p2-p1);
points = zeros(floor(1 / resolution),3);
index = 1;

for t = 0 : resolution : 1
    point = p1 + t * line_dir;
    points(index,:) = point;
    index = index + 1;
    
end