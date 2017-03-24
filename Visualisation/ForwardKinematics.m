%{
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% 
%   FILENAME:   ForwardKinematics.m 
%
%
%   DESCRIPTION: program to compute the forward kinematics of a 24
%   DOF humanoid robot.
%
%
%   AUTHOR: Andrew Murtagh, 
%           Trinity Robotics Group, Trinity College Dublin.
%
%
%   NOTES: -DH parameters in BOLD represent joints that can be moved.
%          -updates several joint positions over 20 iterations.
%          -yaw is about z, pitch is about y and roll is about x.
%          -right and left are from robot's point of view.
%          -movements are reversed between left and right limbs.
%
%
%   VERSION: v1
%
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%}


close all; clear; clc;
%origin
O = [1  0   0   0;
     0  1   0   0;
     0  0   1   0;
     0  0   0   1];
figure('units','normalized','outerposition',[0 0 1 1]);
title('Forward Kinematics'); 
 
 
 
 
%--------------------------------------------------------------------------
%Define DH parameters
%--------------------------------------------------------------------------

%head
head_a_1 = 0;
head_alpha_1 = -90; 
head_d_1 = 0;
HEAD_THETA_1 = -20;                             % HEAD YAW, -(ve) is right, +(ve) is left

head_a_2 = 0;
head_alpha_2 = 90;
head_d_2 = 0;
HEAD_THETA_2 = 10;                              % HEAD PITCH, (-ve) is up, (+ve) is down

NeckOffset =[1  0   0   0;
             0  1   0   0;
             0  0   1   0.126;
             0  0   0   1];
NeckO = O*NeckOffset;



%right arm
ra_a_1 = 0;
ra_alpha_1 = -90;
ra_d_1 = 0;
RA_THETA_1 = -120;                              % SHOULDER PITCH, -120 is backwards, -70 is forwards

ra_a_2 = 0;
ra_alpha_2 = 90; 
ra_d_2 = 0; 
RA_THETA_2 = 90;                                % SHOULDER ROLL, 70 is outwards, 100 is inwards

ra_a_3 = 0;
ra_alpha_3 = 90; 
ra_d_3 = 0.11;
RA_THETA_3 = -10;                               % ARM TWIST, (+ve) is outwards, (-ve) is inwards

ra_a_4 = 0;
RA_ALPHA_4 = -120;                              % ELBOW BEND, -120 is upwards -70 is backwards
ra_d_4 = 0; 
ra_theta_4 = 0; 

ra_a_5 = 0;
ra_alpha_5 = 90;
ra_d_5 = 0.09;
ra_theta_5 = 0;

RightArmOffset =[1  0   0   0;
                 0  1   0   -0.098;
                 0  0   1   0.1;
                 0  0   0   1];
RightArmOrigin = O*RightArmOffset*Rx(90);




%left arm
la_a_1 = 0;
la_alpha_1 = -90;
la_d_1 = 0; 
LA_THETA_1 = -90;                               % SHOULDER PITCH

la_a_2 = 0;
la_alpha_2 = 90; 
la_d_2 = 0; 
LA_THETA_2 =90;                                 % SHOULDER ROLL

la_a_3 = 0;
la_alpha_3 = 90; 
la_d_3 = 0.11;
LA_THETA_3 = -10;                               % ARM TWIST

la_a_4 = 0;
LA_ALPHA_4 = -90;                               % ELBOW BEND
la_d_4 = 0; 
la_theta_4 = 0; 

la_a_5 = 0;
la_alpha_5 = 90;
la_d_5 = 0.09;
la_theta_5 = 0;

LeftArmOffset =[1  0   0   0;
                 0  1   0   0.098;
                 0  0   1   0.1;
                 0  0   0   1];
LeftArmOrigin = O*LeftArmOffset*Rx(90);





%left leg
ll_a_1 = 0;
LL_ALPHA_1 = 0;                                 % HIP ROLL, (+ve) is outwards, (-ve) is inwards
ll_d_1 = -0.0; 
ll_theta_1 = 0; 

ll_a_2 = -0.0; 
LL_ALPHA_2 = 0;                                 % HIP PITCH, (+ve) is forward, (-ve) is backwards
ll_d_2 = 0; 
ll_theta_2 = -90;

ll_a_3 = 0;
LL_ALPHA_3 = 0;                                 % KNEE PITCH, (-ve) is backward, (+ve) is fowards
ll_d_3 = -0.1;
ll_theta_3 = 0; 

ll_a_4 = 0;
LL_ALPHA_4 = 0;                                 % ANKLE PITCH, (+ve) is forward, (-ve) is backward
ll_d_4 = -0.12;  
ll_theta_4 = 0; 

ll_a_5 = 0;
LL_ALPHA_5 = 0;                                 % ANKLE ROLL, (-ve) is inward, (+ve) is outward
ll_d_5 = -0.0; 
ll_theta_5 = 90; 

ll_a_6 = 0; 
ll_alpha_6 = 0;
ll_d_6 = -0.02;
ll_theta_6 = 0; 

LeftLegOffset =[1  0   0   0;
                 0  1   0   0.05;
                 0  0   1   -0.085;
                 0  0   0   1];
LeftLegOrigin = O*LeftLegOffset;



%right leg
rl_a_1 = 0;
RL_ALPHA_1 = -10;                                 % HIP ROLL
rl_d_1 = -0.0;
rl_theta_1 = 0; 

rl_a_2 = -0.0; 
RL_ALPHA_2 = -10;                                 % HIP PITCH
rl_d_2 = 0; 
rl_theta_2 = -90;

rl_a_3 = 0;
RL_ALPHA_3 = -10;                                 % KNEE PITCH 
rl_d_3 = -0.1;
rl_theta_3 = 0; 

rl_a_4 = 0;
RL_ALPHA_4 = 0;                                   % ANKLE PITCH 
rl_d_4 = -0.12;  
rl_theta_4 = 0; 

rl_a_5 = 0;
RL_ALPHA_5 = 10;                                 % ANKLE ROLL
rl_d_5 = -0.0; 
rl_theta_5 = 90; 

rl_a_6 = 0; 
rl_alpha_6 = 0;
rl_d_6 = -0.02;
rl_theta_6 = 0; 

RightLegOffset =[1  0   0   0;
                 0  1   0   -0.05;
                 0  0   1   -0.085;
                 0  0   0   1];
RightLegOrigin = O*RightLegOffset;





 
for i=0:1:20
    pause(0.5);
    
    %----------------------------------------------------------------------
    %compute transformation matrices using DH parameters
    %----------------------------------------------------------------------
    ra_A1 = DH(ra_a_1, ra_alpha_1, ra_d_1, RA_THETA_1);
    ra_A2 = DH(ra_a_2, ra_alpha_2, ra_d_2, RA_THETA_2);
    ra_A3 = DH(ra_a_3, ra_alpha_3, ra_d_3, RA_THETA_3);
    ra_A4 = DH(ra_a_4, RA_ALPHA_4, ra_d_4, ra_theta_4);
    ra_A5 = DH(ra_a_5, ra_alpha_5, ra_d_5, ra_theta_5);
    
    head_A1 = DH(head_a_1, head_alpha_1, head_d_1, HEAD_THETA_1);
    head_A2 = DH(head_a_2, head_alpha_2, head_d_2, HEAD_THETA_2);
    
    
    la_A1 = DH(la_a_1, la_alpha_1, la_d_1, LA_THETA_1);
    la_A2 = DH(la_a_2, la_alpha_2, la_d_2, LA_THETA_2);
    la_A3 = DH(la_a_3, la_alpha_3, la_d_3, LA_THETA_3);
    la_A4 = DH(la_a_4, LA_ALPHA_4, la_d_4, la_theta_4);
    la_A5 = DH(la_a_5, la_alpha_5, la_d_5, la_theta_5);
    
    
    ll_A1 = DH(ll_a_1, LL_ALPHA_1, ll_d_1, ll_theta_1);
    ll_A2 = DH(ll_a_2, LL_ALPHA_2, ll_d_2, ll_theta_2);
    ll_A3 = DH(ll_a_3, LL_ALPHA_3, ll_d_3, ll_theta_3);
    ll_A4 = DH(ll_a_4, LL_ALPHA_4, ll_d_4, ll_theta_4);
    ll_A5 = DH(ll_a_5, LL_ALPHA_5, ll_d_5, ll_theta_5);
    ll_A6 = DH(ll_a_6, ll_alpha_6, ll_d_6, ll_theta_6);
    
    
    
    rl_A1 = DH(rl_a_1, RL_ALPHA_1, rl_d_1, rl_theta_1);
    rl_A2 = DH(rl_a_2, RL_ALPHA_2, rl_d_2, rl_theta_2);
    rl_A3 = DH(rl_a_3, RL_ALPHA_3, rl_d_3, rl_theta_3);
    rl_A4 = DH(rl_a_4, RL_ALPHA_4, rl_d_4, rl_theta_4);
    rl_A5 = DH(rl_a_5, RL_ALPHA_5, rl_d_5, rl_theta_5);
    rl_A6 = DH(rl_a_6, rl_alpha_6, rl_d_6, rl_theta_6);
    
    
    
    %----------------------------------------------------------------------
    %compute link positions
    %----------------------------------------------------------------------
    %right arm
    ra_0 = RightArmOrigin*[0 0 0 1]';
    ra_1 = RightArmOrigin*ra_A1*[0 0 0 1]';
    ra_2 = RightArmOrigin*ra_A1*ra_A2*[0 0 0 1]';
    ra_3 = RightArmOrigin*ra_A1*ra_A2*ra_A3*[0 0 0 1]';
    ra_4 = RightArmOrigin*ra_A1*ra_A2*ra_A3*ra_A4*[0 0 0 1]';
    ra_5 = RightArmOrigin*ra_A1*ra_A2*ra_A3*ra_A4*ra_A5*[0 0 0 1]';
    
    ra_coord = zeros(6,3);
    ra_coord(1,1:3) = ra_0(1:3);
    ra_coord(2,1:3) = ra_1(1:3);
    ra_coord(3,1:3) = ra_2(1:3);
    ra_coord(4,1:3) = ra_3(1:3);
    ra_coord(5,1:3) = ra_4(1:3);
    ra_coord(6,1:3) = ra_5(1:3);
  
    plot3(ra_coord(1:6,1), ra_coord(1:6,2), ra_coord(1:6,3), '-y', 'LineWidth', 2);
    hold on;
    
    %left arm
    la_0 = LeftArmOrigin*[0 0 0 1]';
    la_1 = LeftArmOrigin*la_A1*[0 0 0 1]';
    la_2 = LeftArmOrigin*la_A1*la_A2*[0 0 0 1]';
    la_3 = LeftArmOrigin*la_A1*la_A2*la_A3*[0 0 0 1]';
    la_4 = LeftArmOrigin*la_A1*la_A2*la_A3*la_A4*[0 0 0 1]';
    la_5 = LeftArmOrigin*la_A1*la_A2*la_A3*la_A4*la_A5*[0 0 0 1]';
    
    la_coord = zeros(6,3);
    la_coord(1,1:3) = la_0(1:3);
    la_coord(2,1:3) = la_1(1:3);
    la_coord(3,1:3) = la_2(1:3);
    la_coord(4,1:3) = la_3(1:3);
    la_coord(5,1:3) = la_4(1:3);
    la_coord(6,1:3) = la_5(1:3);
  
    plot3(la_coord(1:6,1), la_coord(1:6,2), la_coord(1:6,3), '-y', 'LineWidth', 2);
    
    
    %left leg
    ll_0 = LeftLegOrigin*[0 0 0 1]';
    ll_1 = LeftLegOrigin*ll_A1*[0 0 0 1]';
    ll_2 = LeftLegOrigin*ll_A1*ll_A2*[0 0 0 1]';
    ll_3 = LeftLegOrigin*ll_A1*ll_A2*ll_A3*[0 0 0 1]';
    ll_4 = LeftLegOrigin*ll_A1*ll_A2*ll_A3*ll_A4*[0 0 0 1]';
    ll_5 = LeftLegOrigin*ll_A1*ll_A2*ll_A3*ll_A4*ll_A5*[0 0 0 1]';
    ll_6 = LeftLegOrigin*ll_A1*ll_A2*ll_A3*ll_A4*ll_A5*ll_A6*[0 0 0 1]';
    
    ll_coord = zeros(7,3);
    ll_coord(1,1:3) = ll_0(1:3);
    ll_coord(2,1:3) = ll_1(1:3);
    ll_coord(3,1:3) = ll_2(1:3);
    ll_coord(4,1:3) = ll_3(1:3);
    ll_coord(5,1:3) = ll_4(1:3);
    ll_coord(6,1:3) = ll_5(1:3);
    ll_coord(7,1:3) = ll_6(1:3);
  
    plot3(ll_coord(1:7,1), ll_coord(1:7,2), ll_coord(1:7,3), '-y', 'LineWidth', 2);
    
    %right leg
    rl_0 = RightLegOrigin*[0 0 0 1]';
    rl_1 = RightLegOrigin*rl_A1*[0 0 0 1]';
    rl_2 = RightLegOrigin*rl_A1*rl_A2*[0 0 0 1]';
    rl_3 = RightLegOrigin*rl_A1*rl_A2*rl_A3*[0 0 0 1]';
    rl_4 = RightLegOrigin*rl_A1*rl_A2*rl_A3*rl_A4*[0 0 0 1]';
    rl_5 = RightLegOrigin*rl_A1*rl_A2*rl_A3*rl_A4*rl_A5*[0 0 0 1]';
    rl_6 = RightLegOrigin*rl_A1*rl_A2*rl_A3*rl_A4*rl_A5*rl_A6*[0 0 0 1]';
    
    rl_coord = zeros(7,3);
    rl_coord(1,1:3) = rl_0(1:3);
    rl_coord(2,1:3) = rl_1(1:3);
    rl_coord(3,1:3) = rl_2(1:3);
    rl_coord(4,1:3) = rl_3(1:3);
    rl_coord(5,1:3) = rl_4(1:3);
    rl_coord(6,1:3) = rl_5(1:3);
    rl_coord(7,1:3) = rl_6(1:3);
  
    plot3(rl_coord(1:7,1), rl_coord(1:7,2), rl_coord(1:7,3), '-y', 'LineWidth', 2);
    

    %set up figure and plot a ground plane
    xlim([-0.4 0.4]); 
    ylim([-0.4 0.4]);
    zlim([-0.4 0.4]);
    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis manual; 
    view(60, 20);
    fill3([0.2 0.2 -0.2 -0.2],[0.2 -0.2 -0.2 0.2],[-0.3186 -0.3186 -0.3186 -0.3186 ],'r')
    alpha(0.3)
    
    
   
    %----------------------------------------------------------------------
    %draw coordinate frames
    %----------------------------------------------------------------------
    drawCoordFrame(O,'O'); 
    
    %right arm
    drawCoordFrame(RightArmOrigin*ra_A1,'A1'); 
    drawCoordFrame(RightArmOrigin*ra_A1*ra_A2,'A2'); 
    drawCoordFrame(RightArmOrigin*ra_A1*ra_A2*ra_A3,'A3'); 
    drawCoordFrame(RightArmOrigin*ra_A1*ra_A2*ra_A3*ra_A4,'A4');
    drawCoordFrame(RightArmOrigin*ra_A1*ra_A2*ra_A3*ra_A4*ra_A5,'A5');
    
    %head
    drawCoordFrame(NeckO*head_A1,'A1'); 
    drawCoordFrame(NeckO*head_A1*head_A2,'A2'); 
    
    %draw an ellipsoid head
    [x, y, z] = ellipsoid(0,0,0.028, 0.012,0.02,0.06, 30);
    t = hgtransform;
    S1 = surf(x, y, z,'Parent',t);
    rotate(S1,[0 0 1], HEAD_THETA_1);
    rotate(S1,[0 1 0], HEAD_THETA_2);
    Tx2 = makehgtform('translate',[0 0 0.156]);
    t.Matrix = Tx2;
    
    
    %left arm
    drawCoordFrame(LeftArmOrigin*la_A1,'A1'); 
    drawCoordFrame(LeftArmOrigin*la_A1*la_A2,'A2'); 
    drawCoordFrame(LeftArmOrigin*la_A1*la_A2*la_A3,'A3'); 
    drawCoordFrame(LeftArmOrigin*la_A1*la_A2*la_A3*la_A4,'A4');
    drawCoordFrame(LeftArmOrigin*la_A1*la_A2*la_A3*la_A4*la_A5,'A5');
    
    
    %left leg
    drawCoordFrame(O,'O'); 
    drawCoordFrame(LeftLegOrigin,'LO'); 
    drawCoordFrame(LeftLegOrigin*ll_A1,'A1'); 
    drawCoordFrame(LeftLegOrigin*ll_A1*ll_A2,'A2'); 
    drawCoordFrame(LeftLegOrigin*ll_A1*ll_A2*ll_A3,'A3'); 
    drawCoordFrame(LeftLegOrigin*ll_A1*ll_A2*ll_A3*ll_A4,'A4');
    drawCoordFrame(LeftLegOrigin*ll_A1*ll_A2*ll_A3*ll_A4*ll_A5,'A5');
    drawCoordFrame(LeftLegOrigin*ll_A1*ll_A2*ll_A3*ll_A4*ll_A5*ll_A6,'A6');
    
    %right leg
    drawCoordFrame(O,'O'); 
    drawCoordFrame(RightLegOrigin,'LO'); 
    drawCoordFrame(RightLegOrigin*rl_A1,'A1'); 
    drawCoordFrame(RightLegOrigin*rl_A1*rl_A2,'A2'); 
    drawCoordFrame(RightLegOrigin*rl_A1*rl_A2*rl_A3,'A3'); 
    drawCoordFrame(RightLegOrigin*rl_A1*rl_A2*rl_A3*rl_A4,'A4');
    drawCoordFrame(RightLegOrigin*rl_A1*rl_A2*rl_A3*rl_A4*rl_A5,'A5');
    drawCoordFrame(RightLegOrigin*rl_A1*rl_A2*rl_A3*rl_A4*rl_A5*rl_A6,'A6');
    
    hold off;
    
    %----------------------------------------------------------------------
    %update joint positions
    %----------------------------------------------------------------------
    
    %right arm
    RA_THETA_1 = RA_THETA_1 + 2;                
    RA_ALPHA_4 = RA_ALPHA_4-2;                    
   
    %head
    HEAD_THETA_1 = HEAD_THETA_1  + 2;    
    HEAD_THETA_2 = HEAD_THETA_2 - 1;      
    
    %left arm
    LA_THETA_2 = LA_THETA_2+2;                  
    
    %left leg
    LL_ALPHA_2 = LL_ALPHA_2 +2;
    LL_ALPHA_3 = LL_ALPHA_3 - 2;
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
 %   text(A(1,4)+scale*A(1,1), A(2,4)+scale*A(2,1), A(3,4)+scale*A(3,1), stringsd, 'FontSize', 8);
 %   text(A(1,4)+scale*A(1,2), A(2,4)+scale*A(2,2), A(3,4)+scale*A(3,2), stringsd, 'FontSize', 8);
  %  text(A(1,4)+scale*A(1,3), A(2,4)+scale*A(2,3), A(3,4)+scale*A(3,3), stringsd, 'FontSize', 8);
end

    