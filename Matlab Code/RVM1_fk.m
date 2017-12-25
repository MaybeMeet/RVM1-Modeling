function [pos] = RVM1_fk( theta_1, theta_2, theta_3, theta_4, theta_5)
% The input to the function will be the joint
%angles of the robot in radians, and the distance between the gripper pads in inches.
%The output  contain 10 positions of various points along the robot arm 

%% DH Parameters assuming homeposition as zero i.e, theta=0

dhparams = [ 0      pi/2    1.52        theta_1;
             2.50      0          0     theta_2;
             1.60     0        0        theta_3;
             0      pi/2        0      theta_4+pi/2;
             0      0         .72       theta_5-pi/2];

% Extract the ai, alphai, di and thetai from the dhparams matrix

l = .70;
w = .60;
% 
grip_1 = [-l/2 0 0];
grip_2 = [-l/2 0 -w];
grip_3 = [0 0 -w] ;
grip_4 = [l/2 0 -w]; 
grip_5 = [l/2 0 0];


link_1 = dhparams(1,:);
link_2 = dhparams(2,:);
link_3 = dhparams(3,:);
link_4 = dhparams(4,:);
link_5 = dhparams(5,:);


a_1 = link_1(1); alpha_1 = link_1(2); d_1 = link_1(3); theta1 = link_1(4);
a_2 = link_2(1); alpha_2 = link_2(2); d_2 = link_2(3); theta2 = link_2(4);
a_3 = link_3(1); alpha_3 = link_3(2); d_3 = link_3(3); theta3 = link_3(4);
a_4 = link_4(1); alpha_4 = link_4(2); d_4 = link_4(3); theta4 = link_4(4);
a_5 = link_5(1); alpha_5 = link_5(2); d_5 = link_5(3); theta5 = link_5(4);

% Compute the Ai realtive frames from one joint to other
A01 = compute_dh_matrix(a_1, alpha_1, d_1, theta1);
A12 = compute_dh_matrix(a_2, alpha_2, d_2, theta2);
A23 = compute_dh_matrix(a_3, alpha_3, d_3, theta3);
A34 = compute_dh_matrix(a_4, alpha_4, d_4, theta4);
A45 = compute_dh_matrix(a_5, alpha_5, d_5, theta5);


% Compute position of each frame w.r.t the ground frame
pos = zeros(6, 3);
A02 = A01 * A12;
A03 = A02 * A23;
A04 = A03 * A34;
A05 = A04 * A45;

%calculating positions of all points 
pos(1,:) = [0;0;0];
pos(2,:) = A01(1:3,4);
pos(3,:) = A02(1:3,4);
pos(4,:) = A03(1:3,4);
pos(5,:) = A04(1:3,4);
pos(6,:) = A05(1:3,4);

grip_1_pos = (A05 * [grip_1'; 1])';
grip_2_pos = (A05 * [grip_2'; 1])';
grip_3_pos = (A05 * [grip_3'; 1])';
grip_4_pos = (A05 * [grip_4'; 1])';
grip_5_pos = (A05 * [grip_5'; 1])';

%
pos(7,:) = grip_1_pos(1:3);
pos(8,:) = grip_2_pos(1:3);
pos(9,:) = grip_3_pos(1:3);
pos(10,:) = grip_4_pos(1:3);
pos(11,:) = grip_5_pos(1:3);


end