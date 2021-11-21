clear all; clc;
d2 = 900; a2 = 1000; d4 = 1400; d5 = 1200; d6 = 800; a6 = 1000;

% L = Link([theta d a alpha], 'standard' or 'modified'); 
% Standard is for standard transformation matrix 
% Modified is the transpose of standard transformation matrix

L(1) = Link([0 0 0 -pi/2], 'modified');

L(2) = Link([pi/2 -a2 d2 pi/2], 'modified');

L(3) = Link([pi/2 0 0 pi/2], 'modified');

L(4) = Link([0 0 -d4 0], 'modified');

L(5) = Link([-pi/2 0 -d5 0], 'modified');

L(6) = Link([-pi/2 -d6 a6 -pi/2], 'modified');

L(7) = Link([0 0 0 -pi/2], 'modified');

Exo = SerialLink(L);
Exo.name = 'exo';

syms q1 q2 q3 q4 q5 q6 q7
q = [q1 q2 q3 q4 q5 q6 q7];
Transformation_matrix = Exo.fkine(q);



% offset = [0, pi/2, pi/2, 0, -pi/2, -pi/2, 0];
% Change the offset values in Exo variable to get the correct DH parameters
% Exo.fkine([0 0 0 0 0 0 0]) gives the transformation matrix with all the
% joint inputs as zero
% Exo.plot([0 0 0 0 0 0 0]) gives the line diagram
