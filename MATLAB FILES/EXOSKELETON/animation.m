clear all;
close all;
clc;

syms a0 a1 a2 d0 q1 q2 q3 q4 q5 qg real ;
a0 = 0.075; 
a1 = 0.3; a2 = 0.3; d0 = 0.25;

dh1 = [pi/2, -a0, 0, q1+pi/2];
dh2 = [0, a1, 0, q2];
dh3 = [0, a2, 0, q3];
dh4 = [0, a2, 0, q4];
dh5 = [0, a1, 0, (q5+pi/2)];
dh6 = [-pi/2, a0, 0, 0];

T01 = (transformation(dh1));
T12 = (transformation(dh2));
T23 = (transformation(dh3));
T34 = (transformation(dh4));
T45 = (transformation(dh5));
T56 = (transformation(dh6));

T02 = vpa(simplify(T01*T12));
T03 = vpa(simplify(T01*T12*T23));
T04 = vpa(simplify(T01*T12*T23*T34));
T05 = vpa(simplify(T01*T12*T23*T34*T45));
T06 = vpa(simplify(T01*T12*T23*T34*T45*T56));
   
P06 = vpa(simplify(T06(1:3,4)));
    
J06 = [diff(P06,q1),diff(P06,q2),diff(P06,q3),diff(P06,q4),diff(P06,q5)];


L = 0.2; % Step Length = 2.5*foot length

t = (0:360)*pi/180;
r = L/(2*pi);

q = [-0.0239; 0.2465; 2.7995; -0.3459; 0.1927]; %Initial Configuration

for j = 1:length(t)
    
mu = [-0.2331334779 ; 0 ; -0.00038442603]+[r*(t(j)-sin(t(j))) ; 0 ; r*(1-cos(t(j))) ]; %Expected Position w.r.t stance leg

[qf(:,j), prof_bp(:,j)] = inverse_kinematic(T06,J06,mu,q); % Inverse kinematic function

% Returns joint angles and position of end-effector

end 



% for i = 1:10:length(t)
%    
% q = [q1 q2 q3 q4 q5];
% q_new = [qf(1,i) qf(2,i) qf(3,i) qf(4,i) qf(5,i)];
%     
% biped_c = animation_plot(q,q_new,dh1,dh2,dh3,dh4,dh5,dh6);
% 
% plot([0,0.08,biped_c(1,1),0],[0,0,biped_c(2,1),0],'r-','linewidth',3)
% hold on
% plot([biped_c(1,6),biped_c(1,6)+0.08,biped_c(1,5),biped_c(1,6)],[biped_c(2,6),biped_c(2,6),biped_c(2,5),biped_c(2,6)],'r-','linewidth',3)
% plot(biped_c(1,1:5),biped_c(2,1:5),'g--','linewidth',1.5,'MarkerEdgeColor','g',...
%                        'MarkerFaceColor','g',...
%                        'MarkerSize',1);
% plot(prof_bp(1,:),prof_bp(2,:),'k-')
% set(gca,'fontsize',16)
% xlabel('x,[m]');
% ylabel('z,[m]');`
% axis([-0.5 0.5 -0.1 0.9])
% axis square
% grid on
% pause(0.1)
% hold off
%        
%     
% end
