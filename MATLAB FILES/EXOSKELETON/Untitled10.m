clear all; close all; clc;
L = 0.2; 
t = (0:360)*pi/180;
r = L/(2*pi);
for j =1:length(t)
a0 = 0.05; a5 = 0.05;
a1 = 0.4; a2 = 0.35; a3 = 0.35; a4 = 0.4;
th1 = 330*pi/180; th2 = 20*pi/180;
th3 = 200*pi/180;
th4 = -20 * pi/180; th5 = 10*pi/180;
  
x = a0 + a4*cos(th1 + th2 + th3 + th4) + a2*cos(th1 + th2) + a1*cos(th1) + a5*cos(th1 + th2 + th3 + th4 + th5) + a3*cos(th1 + th2 + th3);
y = a4*sin(th1 + th2 + th3 + th4) + a2*sin(th1 + th2) + a1*sin(th1) + a5*sin(th1 + th2 + th3 + th4 + th5) + a3*sin(th1 + th2 + th3);
psi = th1 + th2 + th3 + th4 + th5;

mu_a = [x;y;psi]+[0.05+0.05*sin(0.5*t(j));0.1*t(j);0];
mu_a = [0;0.4;psi]+[r*(1-cos(t(j)));-r*(t(j)-sin(t(j)));0];
if j>1
    q =q_r(:,j-1);
else
    q = [th1;th2;th3;th4;th5] ;
end
for i = 1:10
    th1 = q(1); th2 = q(2); th3 = q(3); th4 = q(4); th5 = q(5);
J = [- a4*sin(th1 + th2 + th3 + th4) - a2*sin(th1 + th2) - a1*sin(th1) - a5*sin(th1 + th2 + th3 + th4 + th5) - a3*sin(th1 + th2 + th3), - a4*sin(th1 + th2 + th3 + th4) - a2*sin(th1 + th2) - a5*sin(th1 + th2 + th3 + th4 + th5) - a3*sin(th1 + th2 + th3), - a4*sin(th1 + th2 + th3 + th4) - a5*sin(th1 + th2 + th3 + th4 + th5) - a3*sin(th1 + th2 + th3), - a4*sin(th1 + th2 + th3 + th4) - a5*sin(th1 + th2 + th3 + th4 + th5), -a5*sin(th1 + th2 + th3 + th4 + th5); 
       a4*cos(th1 + th2 + th3 + th4) + a2*cos(th1 + th2) + a1*cos(th1) + a5*cos(th1 + th2 + th3 + th4 + th5) + a3*cos(th1 + th2 + th3), a4*cos(th1 + th2 + th3 + th4) + a2*cos(th1 + th2) + a5*cos(th1 + th2 + th3 + th4 + th5) + a3*cos(th1 + th2 + th3), a4*cos(th1 + th2 + th3 + th4) + a5*cos(th1 + th2 + th3 + th4 + th5) + a3*cos(th1 + th2 + th3), a4*cos(th1 + th2 + th3 + th4) + a5*cos(th1 + th2 + th3 + th4 + th5), a5*cos(th1 + th2 + th3 + th4 + th5); 
       1, 1, 1, 1, 1];
x = a0 + a4*cos(th1 + th2 + th3 + th4) + a2*cos(th1 + th2) + a1*cos(th1) + a5*cos(th1 + th2 + th3 + th4 + th5) + a3*cos(th1 + th2 + th3);
y = a4*sin(th1 + th2 + th3 + th4) + a2*sin(th1 + th2) + a1*sin(th1) + a5*sin(th1 + th2 + th3 + th4 + th5) + a3*sin(th1 + th2 + th3);
psi = th1 + th2 + th3 + th4 + th5;

mu_e = [x;y;psi];
    if abs(mu_a-mu_e)<1e-5
        break;
    end
q = q + pinv(J) * (mu_a-mu_e);
end
q_r(:,j) = q;
R = [cosd(90),-sind(90);
     sind(90), cosd(90)];
prof_bp(:,j) = R*[x;y];
end

for i = 1:10:length(t)
    th1 = q_r(1,i); th2 = q_r(2,i); th3 = q_r(3,i); th4 = q_r(4,i); th5 = q_r(5,i);
x1 = a0;
y1 = 0;
x2 = a0 + a1*cos(th1);
y2 = a1*sin(th1);
x3 = a0 + a2*cos(th1 + th2) + a1*cos(th1);
y3 = a2*sin(th1 + th2) + a1*sin(th1);
x4 = a0 + a2*cos(th1 + th2) + a1*cos(th1) + a3*cos(th1 + th2 + th3);
y4 = a2*sin(th1 + th2) + a1*sin(th1) + a3*sin(th1 + th2 + th3);
x5 = a0 + a4*cos(th1 + th2 + th3 + th4) + a2*cos(th1 + th2) + a1*cos(th1) + a3*cos(th1 + th2 + th3);
y5 = a4*sin(th1 + th2 + th3 + th4) + a2*sin(th1 + th2) + a1*sin(th1) + a3*sin(th1 + th2 + th3);
x = a0 + a4*cos(th1 + th2 + th3 + th4) + a2*cos(th1 + th2) + a1*cos(th1) + a5*cos(th1 + th2 + th3 + th4 + th5) + a3*cos(th1 + th2 + th3);
y = a4*sin(th1 + th2 + th3 + th4) + a2*sin(th1 + th2) + a1*sin(th1) + a5*sin(th1 + th2 + th3 + th4 + th5) + a3*sin(th1 + th2 + th3);
psi = th1 + th2 + th3 + th4 + th5;
R = [cosd(90),-sind(90);
     sind(90), cosd(90)];
biped_c = R* [x1,x2,x3,x4,x5,x;
              y1,y2,y3,y4,y5,y];
plot([0,0.08,biped_c(1,1),0],[0,0,biped_c(2,1),0],'r-','linewidth',3)
hold on
plot([biped_c(1,6),biped_c(1,6)+0.08,biped_c(1,5),biped_c(1,6)],[biped_c(2,6),biped_c(2,6),biped_c(2,5),biped_c(2,6)],'r-','linewidth',3)
plot(biped_c(1,1:5),biped_c(2,1:5),'r-o','linewidth',3,'MarkerEdgeColor','k',...
                       'MarkerFaceColor','g',...
                       'MarkerSize',10)
plot(prof_bp(1,:),prof_bp(2,:),'k-')
set(gca,'fontsize',16)
xlabel('x,[m]');
ylabel('z,[m]');
axis([-0.5 0.5 -0.1 0.9])
axis square
grid on
pause(0.1)
hold off
end

L = 0.2; 
t = (0:360)*pi/180;
r = L/(2*pi);
for j =1:length(t)
a0 = 0.05; a5 = 0.05;
a1 = 0.4; a2 = 0.35; a3 = 0.4; a4 = 0.35;
th1 = 330*pi/180; th2 = 20*pi/180;
th3 = 200*pi/180;
th4 = -20 * pi/180; th5 = 10*pi/180;
  
x = a0 + a4*cos(th1 + th2 + th3 + th4) + a2*cos(th1 + th2) + a1*cos(th1) + a5*cos(th1 + th2 + th3 + th4 + th5) + a3*cos(th1 + th2 + th3);
y = a4*sin(th1 + th2 + th3 + th4) + a2*sin(th1 + th2) + a1*sin(th1) + a5*sin(th1 + th2 + th3 + th4 + th5) + a3*sin(th1 + th2 + th3);
psi = th1 + th2 + th3 + th4 + th5;

mu_a = [x;y;psi]+[0.05+0.05*sin(0.5*t(j));0.1*t(j);0];
mu_a = [0;-0.2;psi]+[r*(1-cos(t(j)));-r*(t(j)-sin(t(j)));0];
if j>1
    q =q_r(:,j-1);
else
    q = [th1;th2;th3;th4;th5] ;
end
for i = 1:10
    th1 = q(1); th2 = q(2); th3 = q(3); th4 = q(4); th5 = q(5);
J = [- a4*sin(th1 + th2 + th3 + th4) - a2*sin(th1 + th2) - a1*sin(th1) - a5*sin(th1 + th2 + th3 + th4 + th5) - a3*sin(th1 + th2 + th3), - a4*sin(th1 + th2 + th3 + th4) - a2*sin(th1 + th2) - a5*sin(th1 + th2 + th3 + th4 + th5) - a3*sin(th1 + th2 + th3), - a4*sin(th1 + th2 + th3 + th4) - a5*sin(th1 + th2 + th3 + th4 + th5) - a3*sin(th1 + th2 + th3), - a4*sin(th1 + th2 + th3 + th4) - a5*sin(th1 + th2 + th3 + th4 + th5), -a5*sin(th1 + th2 + th3 + th4 + th5); 
       a4*cos(th1 + th2 + th3 + th4) + a2*cos(th1 + th2) + a1*cos(th1) + a5*cos(th1 + th2 + th3 + th4 + th5) + a3*cos(th1 + th2 + th3), a4*cos(th1 + th2 + th3 + th4) + a2*cos(th1 + th2) + a5*cos(th1 + th2 + th3 + th4 + th5) + a3*cos(th1 + th2 + th3), a4*cos(th1 + th2 + th3 + th4) + a5*cos(th1 + th2 + th3 + th4 + th5) + a3*cos(th1 + th2 + th3), a4*cos(th1 + th2 + th3 + th4) + a5*cos(th1 + th2 + th3 + th4 + th5), a5*cos(th1 + th2 + th3 + th4 + th5); 
       1, 1, 1, 1, 1];
x = a0 + a4*cos(th1 + th2 + th3 + th4) + a2*cos(th1 + th2) + a1*cos(th1) + a5*cos(th1 + th2 + th3 + th4 + th5) + a3*cos(th1 + th2 + th3);
y = a4*sin(th1 + th2 + th3 + th4) + a2*sin(th1 + th2) + a1*sin(th1) + a5*sin(th1 + th2 + th3 + th4 + th5) + a3*sin(th1 + th2 + th3);
psi = th1 + th2 + th3 + th4 + th5;

mu_e = [x;y;psi];
    if abs(mu_a-mu_e)<1e-5
        break;
    end
q = q + pinv(J) * (mu_a-mu_e);
end
q_r(:,j) = q;
R = [cosd(90),-sind(90);
     sind(90), cosd(90)];
prof_bp(:,j) = R*[x;y];
end

for i = 1:10:length(t)
    th1 = q_r(1,i); th2 = q_r(2,i); th3 = q_r(3,i); th4 = q_r(4,i); th5 = q_r(5,i);
x1 = a0;
y1 = 0;
x2 = a0 + a1*cos(th1);
y2 = a1*sin(th1);
x3 = a0 + a2*cos(th1 + th2) + a1*cos(th1);
y3 = a2*sin(th1 + th2) + a1*sin(th1);
x4 = a0 + a2*cos(th1 + th2) + a1*cos(th1) + a3*cos(th1 + th2 + th3);
y4 = a2*sin(th1 + th2) + a1*sin(th1) + a3*sin(th1 + th2 + th3);
x5 = a0 + a4*cos(th1 + th2 + th3 + th4) + a2*cos(th1 + th2) + a1*cos(th1) + a3*cos(th1 + th2 + th3);
y5 = a4*sin(th1 + th2 + th3 + th4) + a2*sin(th1 + th2) + a1*sin(th1) + a3*sin(th1 + th2 + th3);
x = a0 + a4*cos(th1 + th2 + th3 + th4) + a2*cos(th1 + th2) + a1*cos(th1) + a5*cos(th1 + th2 + th3 + th4 + th5) + a3*cos(th1 + th2 + th3);
y = a4*sin(th1 + th2 + th3 + th4) + a2*sin(th1 + th2) + a1*sin(th1) + a5*sin(th1 + th2 + th3 + th4 + th5) + a3*sin(th1 + th2 + th3);
psi = th1 + th2 + th3 + th4 + th5;
R = [cosd(90),-sind(90);
     sind(90), cosd(90)];
biped_c = R* [x1,x2,x3,x4,x5,x;
              y1,y2,y3,y4,y5,y];
plot([0,0.08,biped_c(1,1),0]-0.2,[0,0,biped_c(2,1),0],'r-','linewidth',3)
hold on
plot([biped_c(1,6),biped_c(1,6)+0.08,biped_c(1,5),biped_c(1,6)]-0.2,[biped_c(2,6),biped_c(2,6),biped_c(2,5),biped_c(2,6)],'r-','linewidth',3)
plot(biped_c(1,1:5)-0.2,biped_c(2,1:5),'r-o','linewidth',3,'MarkerEdgeColor','k',...
                       'MarkerFaceColor','g',...
                       'MarkerSize',10)
plot(prof_bp(1,:)-0.2,prof_bp(2,:),'k-')
axis([-0.5 0.5 -0.1 0.9])
axis square
set(gca,'fontsize',16)
xlabel('x,[m]');
ylabel('z,[m]');
grid on
pause(0.1)
hold off
end
