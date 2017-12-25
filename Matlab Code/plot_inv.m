function plot_inv(posArray, startPoint, endPoint)
% function plot_inverse_output displays the performance of inverse
% kinematics solution through animation
% startPoint - first point on the line
% endPoint - last point on the line
% posArray - 7x3xn matrix containing position of all the seven points on
% the manipulator across all the n steps

[~, ~, m] = size(posArray);
pause on;  % Set this to on if you want to watch the animation
GraphingTimeDelay = .51; % The length of time that Matlab should pause between positions when graphing, if at all, in seconds.

% Setup plot
figure(2)
scale_f = 1;
axis vis3d
axis(scale_f*[-4 4 -4 4 -4 4])
grid on
view(70,10)
xlabel('X (m.)')
ylabel('Y (m.)')
zlabel('Z (m.)')

% Plot robot initially
hold on
hrobot = plot3([posArray(1, 1,1) posArray(2, 1,1) posArray(3, 1,1) posArray(4, 1,1) posArray(5, 1,1) posArray(6, 1,1)]',...
      [posArray(1, 2, 1) posArray(2, 2,1) posArray(3, 2,1) posArray(4, 2,1) posArray(5, 2,1) posArray(6, 2,1)]',... 
      [posArray(1, 3, 1) posArray(2, 3, 1) posArray(3, 3, 1) posArray(4, 3, 1) posArray(5, 3, 1) posArray(6, 3, 1)]', 'k.-','linewidth',1,'markersize',20);
  hold on
plot3([startPoint(1), endPoint(1)]', [startPoint(2), endPoint(2)]', [startPoint(3), endPoint(3)]', 'g', 'linewidth', 1, 'LineStyle', '--')
pause(GraphingTimeDelay);

actualPose = posArray(6,:,1);

for i = 2 : m
    cla(hrobot)
    set(hrobot,'xdata',[posArray(1, 1,i) posArray(2, 1, i) posArray(3, 1, i) posArray(4, 1, i) posArray(5, 1, i) posArray(6, 1, i)]',...
        'ydata',[posArray(1, 2, i) posArray(2, 2, i) posArray(3, 2, i) posArray(4, 2, i) posArray(5, 2, i) posArray(6, 2, i) ]',...
        'zdata',[posArray(1, 3, i) posArray(2, 3, i) posArray(3, 3) posArray(4, 3, i) posArray(5, 3, i) posArray(6, 3, i)]');
    actualPose = [actualPose;posArray(6,:,i)];
    hold on
    plot3([startPoint(1), endPoint(1)]', [startPoint(2), endPoint(2)]', [startPoint(3), endPoint(3)]', 'g', 'linewidth', 1, 'LineStyle', '--')
    plot3(actualPose(:,1), actualPose(:,2), actualPose(:,3))
    pause(GraphingTimeDelay);
end