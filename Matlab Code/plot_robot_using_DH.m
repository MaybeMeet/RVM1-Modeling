function plot_robot_using_DH(pos,theta_1, theta_2, theta_3, theta_4, theta_5)
% Function plot_fk takes the Forward Kinematics solution(pos - 12 x 3 matrix) computed and plots
% these positions it like a stick figure for the manipulator using plot3
% function

plot3([pos(1, 1) pos(2, 1) pos(3, 1) pos(4, 1) pos(5, 1) pos(6, 1)]',...
      [pos(1, 2) pos(2, 2) pos(3, 2) pos(4, 2) pos(5, 2) pos(6, 2)]',... 
      [pos(1, 3) pos(2, 3) pos(3, 3) pos(4, 3) pos(5, 3) pos(6, 3)]', 'k.-','linewidth',1,'markersize',20);         

hold on
plot3([pos(7, 1), pos(8, 1), pos(9, 1), pos(10, 1), pos(11, 1)]',...
      [pos(7, 2), pos(8, 2), pos(9, 2), pos(10, 2), pos(11, 2)]',...
      [pos(7, 3), pos(8, 3), pos(9, 3), pos(10, 3), pos(11, 3)]','g.-','linewidth',2,'markersize',20)
end