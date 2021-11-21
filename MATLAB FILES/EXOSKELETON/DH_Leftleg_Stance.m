clear all; clc;
a0 = 0.1; 
a1 = 0.3; a2 = 0.3; d0 = 0.05;
% L = Link([theta d a alpha], 'standard' or 'modified'); 
% Standard is for standard transformation matrix 
% Modified is the transpose of standard transformation matrix

L(1) = Link([pi/2 d0 -a0 pi/2], 'modified');

L(2) = Link([0 0 a1 0], 'modified');

L(3) = Link([0 0 a2 0], 'modified');

L(4) = Link([0 0 a2 0], 'modified');

L(5) = Link([pi/2 0 a1 0], 'modified');

% L(6) = Link([-pi/2 -d0 a0 -pi/2], 'modified');

Exo = SerialLink(L);
Exo.name = 'Exo';

Exo.plot([pi/2 pi/6 (130*pi/180) -pi/10 150*pi/180]) 

% for th1 = pi/9:0.01:5*(pi/9)
% %ExoL.plot([0 -pi/2 th1 0]);
% ExoR.plot([0 -pi/2+th1 0 0]);
% ExoR.plot([0 -pi/2 th1+pi/2 0]);
% pause(0.25)
% end


% syms q1 q2 q3  
% q = [q1 q2 q3];
% q = [0 0 0];
% q_ready = [0 -pi/12 pi/6 pi/4 -pi/3 pi/12];
% t = [0:0.05:2];
% Transformation_matrix = Exo.fkine(q);
% q_reach = Exo.ikine(Transformation_matrix);
% [q,q_d,q_dd] = jtraj(q_ready, q_reach, t);


% offset = [-pi/2, 0, pi/2];
% Change the offset values in Exo variable to get the correct DH parameters
% ExoL.fkine([0 0 0]) gives the transformation matrix with all the
% joint inputs as zero for left leg
% ExoR.fkine([0 0 0]) gives the transformation matrix with all the
% joint inputs as zero for right leg
% ExoL.plot([0 0 0]) gives the line diagram for left leg
% ExoR.plot([0 0 0]) gives the line diagram for right leg

% vpa(Transformation_matrix) simplifies the matrix
