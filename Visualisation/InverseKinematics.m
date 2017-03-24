%{
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% 
%   Copyright (C) 2017 Trinity Robotics Group.
%
%
%   FILENAME:   InverseKinematics.m 
%
%
%   DATE: 27/02/2017
%
%
%   DESCRIPTION: program to compute the inverse kinematics of a 3 d.o.f
%                RRR manipulator.
%
%
%   AUTHOR: Andrew Murtagh, 
%           Trinity Robotics Group, Trinity College Dublin.
%
%
%   NOTES: -Jacobian transposition.
%          -orientation is not considered, only position.
%          -terminates once threshold distance to goal is reached.
%          -forward kinematics are also computed for visualising positions.
%
%
%   VERSION: v1
%
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%}


close all; clear; clc;
O = [1  0   0   0;
     0  1   0   0;
     0  0   1   0;
     0  0   0   1];
figure('units','normalized','outerposition',[0 0 1 1]);
title('Inverse Kinematics'); 
 
 
 
 
%--------------------------------------------------------------------------
%Define parameters
%--------------------------------------------------------------------------
%DH parameters
la_a_1 = 0;
la_alpha_1 = -90;
la_d_1 = 0.1; 
LA_THETA_1 = 45;                                % base angle

la_a_2 = 0.1;
la_alpha_2 = 0; 
la_d_2 = 0; 
LA_THETA_2 = -45;                               % shoulder angle

la_a_3 = 0.1;
la_alpha_3 = 0; 
la_d_3 = 0;
LA_THETA_3 = 45;                                % elbow angle

%IK parameters
goal = [0.08 -0.05 0.02]';                
currX =0; currY=0; currZ=0;
alpha = 15;
distance_threshold = 0.01;
%--------------------------------------------------------------------------



%do-while loop not in matlab so have a flag for first iteration.
First = true;
while sqrt(power(goal(1) - currX,2) + power(goal(2) - currY,2) + power(goal(3) - currZ,2))> distance_threshold || First
    %----------------------------------------------------------------------
    %Inverse Kinematics
    %----------------------------------------------------------------------
    %rename joints for convenience
    q1 = LA_THETA_1;
    q2 = LA_THETA_2;
    q3 = LA_THETA_3;

    %compute current position according to overall transformation matrix
    currX = 0.1*cosd(q1)*cosd(q2)*cosd(q3) - 0.1*cosd(q1)*sind(q2)*sind(q3) + 0.1*cosd(q1)*cosd(q2);
    currY = 0.1*sind(q1)*cosd(q2)*cosd(q3) - 0.1*sind(q1)*sind(q2)*sind(q3) + 0.1*sind(q1)*cosd(q2);
    currZ = 0.1 -0.1*cosd(q3)*sind(q2) - 0.1*cosd(q2)*sind(q3) - 0.1*sind(q2);

    %plot current position
    plot3(currX, currY, currZ, 'co');
    xlim([-0.02 0.15]); 
    ylim([-0.14 0.14]);
    zlim([-0.02 0.2]);
    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis manual; 
    hold on;
    %plot goal point
    plot3(goal(1), goal(2), goal(3), 'ro');
    
    

    %Compute Jacobian
    J = [];
    J(1,1) = -0.1*sind(q1)*cosd(q2)*cosd(q3) + 0.1*sind(q1)*sind(q2)*sind(q3) - 0.1*sind(q1)*cosd(q2);
    J(1,2) = -0.1*cosd(q1)*sind(q2)*cosd(q3) - 0.1*cosd(q1)*cosd(q2)*sind(q3) - 0.1*cosd(q1)*sind(q2);
    J(1,3) = -0.1*cosd(q1)*cosd(q2)*sind(q3) - 0.1*cosd(q1)*sind(q2)*cosd(q3);
 
    J(2,1) = 0.1*cosd(q1)*cosd(q2)*cosd(q3) - 0.1*cosd(q1)*sind(q2)*sind(q3) + 0.1*cosd(q1)*cosd(q2);
    J(2,2) = -0.1*sind(q1)*sind(q2)*cosd(q3) - 0.1*cosd(q1)*sind(q2)*sind(q3) + 0.1*cosd(q1)*cosd(q2);
    J(2,3) = -0.1*sind(q1)*cosd(q2)*sind(q3) - 0.1*sind(q1)*sind(q2)*cosd(q3);
 
    J(3,1) = 0;
    J(3,2) = -0.1*cosd(q3)*cosd(q2) + 0.1*sind(q2)*sind(q3) - 0.1*cosd(q2);
    J(3,3) = 0.1*sind(q3)*sind(q2) - 0.1*cosd(q2)*cosd(q3);
 
    %Get delta e
    delta_e = [goal(1) - currX, goal(2) - currY, goal(3) - currZ]';
    %theta_vec = (J'*inv(J*J'))*goal;
    delta_q = alpha*J'*delta_e;
    
    %update joint postions
    LA_THETA_1 = LA_THETA_1 + delta_q(1);
    LA_THETA_2 = LA_THETA_2 + delta_q(2);
    LA_THETA_3 = LA_THETA_3 + delta_q(3);
    
    




    %----------------------------------------------------------------------
    %Visualise manipulator with forward kinematics
    %----------------------------------------------------------------------
    %transformation matrices from DH
    la_A1 = DH(la_a_1, la_alpha_1, la_d_1, LA_THETA_1);
    la_A2 = DH(la_a_2, la_alpha_2, la_d_2, LA_THETA_2);
    la_A3 = DH(la_a_3, la_alpha_3, la_d_3, LA_THETA_3);
    
    %coordinates from transformation matrices
    la_0 = O*[0 0 0 1]';
    la_1 = O*la_A1*[0 0 0 1]';
    la_2 = O*la_A1*la_A2*[0 0 0 1]';
    la_3 = O*la_A1*la_A2*la_A3*[0 0 0 1]';

    la_coord = zeros(4,3);
    la_coord(2,1:3) = la_1(1:3);
    la_coord(3,1:3) = la_2(1:3);
    la_coord(4,1:3) = la_3(1:3);

    %draw linkages and coordinate frames
    plot3(la_coord(1:4,1), la_coord(1:4,2), la_coord(1:4,3), '-y', 'LineWidth', 2);
    drawCoordFrame(O,'O'); 
    drawCoordFrame(O*la_A1,'A1'); 
    drawCoordFrame(O*la_A1*la_A2,'A2'); 
    drawCoordFrame(O*la_A1*la_A2*la_A3,'A3'); 
    %----------------------------------------------------------------------

    
    
    hold off;
    First = false;
    pause(0.01);
    

end







%--------------------------------------------------------------------------
%functions
%--------------------------------------------------------------------------
function A = Rx(theta)
    A = [1    0                 0           0
         0    cosd(theta)     -sind(theta)	0;
         0    sind(theta)     cosd(theta)   0;
         0    0               0             1];

end

function A = Ry(theta)
    A = [cosd(theta)    0	sind(theta)     0
         0              1   0           	0;
         -sind(theta)   0   cosd(theta)     0;
         0              0   0               1];

end

function A = Rz(theta)
    A = [cosd(theta)    -sind(theta)	0   0
         sind(theta)    cosd(theta)     0	0;
         0              0               1   0;
         0              0               0   1];

end

function A = DH(a, alpha, d, theta)
    A = [cosd(theta)    -sind(theta)*cosd(alpha)    sind(theta)*sind(alpha)     a*cosd(theta);
         sind(theta)    cosd(theta)*cosd(alpha)     -cosd(theta)*sind(alpha)	a*sind(theta);
         0              sind(alpha)                 cosd(alpha)                 d;
         0              0                           0                           1];

end

function drawCoordFrame(A, stringsd) 
    scale = 0.02;
    plot3(   [A(1,4) A(1,4)+scale*A(1,1)],  [A(2,4) A(2,4)+scale*A(2,1)],  [A(3,4) A(3,4)+scale*A(3,1)],'r', ...
            [A(1,4) A(1,4)+scale*A(1,2)],  [A(2,4) A(2,4)+scale*A(2,2)],  [A(3,4) A(3,4)+scale*A(3,2)],'g', ... 
            [A(1,4) A(1,4)+scale*A(1,3)],  [A(2,4) A(2,4)+scale*A(2,3)],  [A(3,4) A(3,4)+scale*A(3,3)],'b');
    text(A(1,4)+scale*A(1,1), A(2,4)+scale*A(2,1), A(3,4)+scale*A(3,1), stringsd, 'FontSize', 8);
    text(A(1,4)+scale*A(1,2), A(2,4)+scale*A(2,2), A(3,4)+scale*A(3,2), stringsd, 'FontSize', 8);
    text(A(1,4)+scale*A(1,3), A(2,4)+scale*A(2,3), A(3,4)+scale*A(3,3), stringsd, 'FontSize', 8);
end

    
