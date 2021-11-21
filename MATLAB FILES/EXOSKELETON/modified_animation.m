clear all; close all; clc;

l1 = 35 ; % hip to knee length in cm
l2 = 35; % knee to ankle length
l3 = 20; %upper body length
f = 0; % foot length
xi = 0;
xf = 35; % step length
L = 35;
v_s = 2.5; % hip velocity in x-direction
v_e = 2.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Stance Leg

syms q1 q2 q3

digits(3);

dh1 = [pi/2, -f, 0, q1];
dh2 = [0, l1, 0, q2];
dh3 = [0, l2, 0, q3];
dh4 = [-pi/2, 0, 0, 0];

T01 = simplify((transformation(dh1)));
T12 = (transformation(dh2));
T23 = (transformation(dh3));
T34 = (transformation(dh4));

T02 = vpa(simplify(T01*T12));
T03 = vpa(simplify(T01*T12*T23));
T04 = vpa(simplify(T01*T12*T23*T34));

P04 = vpa(simplify(T04(1:3,4)));

J04 = [diff(P04,q1),diff(P04,q2),diff(P04,q3)];

t = 0:0.1:3;

r = - 2*(xf/(2*(t(end))^3) - (v_s+v_e)/(2*(t(end))^2));

%q = [112*pi/180; 0; -112*pi/180];

for j=1:length(t)
    
    x4(j) = xf/4 + v_s*t(j) + ((v_e-v_s)/(2*t(end)) - r*1.5*t(end))*t(j)*t(j) + r*(t(j))^3 - (xi+xf/2) ;
    
    z4(j) = sqrt((l1+l2)^2-(x4(j)+ (xi+xf/2) -(xi+0.5*xf))^2);
    
    mu = [x4(j);0;z4(j)]; % Expected position w.r.t base of stance leg
    
    if x4(j)<0
    q(1,j) = pi-atan(abs(z4(j)/x4(j)));
    else
    q(1,j) = atan(z4(j)/x4(j));
    end
    
    q(2,j) = 0;
    
    q(3,j) = -q(1,j);
    
    mu_e = simplify([T04(1,4) ; T04(2,4) ; T04(3,4)]); %planar
    vars = [q1, q2, q3];
    newVars = [q(1,j), q(2,j), q(3,j)];
    mu_e = simplify(subs(mu_e,vars, newVars));
    
    x2(j) = simplify(subs(T02(1,4),vars, newVars));
    z2(j) = simplify(subs(T02(3,4),vars, newVars));
    
    prof_bp(:,j) = [mu_e(1);mu_e(3)];  
    
end

for j = 1:length(t)
plotx4(j) = prof_bp(1,j);
plotz4(j) = prof_bp(2,j);
plotx0(j) = 0;
plotz0(j) = 0;
plotx2(j) = x2(j);
plotz2(j) = z2(j);

plot([plotx0(j),plotx2(j)],[plotz0(j),plotz2(j)],'r-')
hold on
plot([plotx2(j),plotx4(j)],[plotz2(j),plotz4(j)],'b-')
xlim([-20,20])

xlabel('x-axis in cm');
ylabel('z-axis in cm');
title('Inverse Kinematics of Stance leg');

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Swing Leg

xw0 = x4; % End-effector of stance leg is same as the base of swing leg (hip joint)
zw0 = z4;

syms qw1 qw2 qw3

dhw1 = [pi/2, 0, 0, qw1];
dhw2 = [0, l2, 0, qw2];
dhw3 = [0, l1, 0, qw3];
dhw4 = [-pi/2, f, 0, 0];

Tw01 = simplify((transformation(dhw1)));
Tw12 = (transformation(dhw2));
Tw23 = (transformation(dhw3));
Tw34 = (transformation(dhw4));

Tw02 = vpa(simplify(Tw01*Tw12));
Tw03 = vpa(simplify(Tw01*Tw12*Tw23));
Tw04 = vpa(simplify(Tw01*Tw12*Tw23*Tw34));

Pw04 = vpa(simplify(Tw04(1:3,4)));

Jw04 = [diff(Pw04,qw1),diff(Pw04,qw2),diff(Pw04,qw3)];

t = 0:0.1:3;

h = 6.25; % Step height in cm

qw = [1.4455;0;-1.4455];

[val,idx] = max(z4);

xm = x4(idx)+(xi+xf/2);

k = h/((xm-xi)*((xm-xi-xf)^2));

for j=1:length(t)
    
    xw4(j) = xi + (3*xf*(t(j))^2)/(t(end))^2 - (2*xf*(t(j))^3)/(t(end))^3 - (xi+xf/2);
    
    zw4(j) = -k*xi*((xf+xi)^2) + k*(xf+xi)*(xf+3*xi)*(xw4(j)+(xi+xf/2)) - k*((2*xf+3*xi)*((xw4(j)+(xi+xf/2))^2)-(xw4(j)+(xi+xf/2))^3);
    
    muw = [xw4(j)-xw0(j);0;zw4(j)-zw0(j)]; % Expected position w.r.t hip of swing leg
    
    for i = 1:10
        % qw1 = qw(1,i)
        % qw2 = qw(2,i); 
        % qw3 = qw(3,i);  
   
        if j>1
         qw = q_w(:,j-1);
        end
   
        % Forward kinematic model
        muw_e = [simplify(Tw04(1,4)) ; simplify(Tw04(2,4)) ; simplify(Tw04(3,4))]; %planar
        varsw = [qw1, qw2, qw3];
        newVarsw = [qw(1), qw(2), qw(3)];
        muw_e = subs(muw_e,varsw, newVarsw);
    
        % Error estimation
        del_muw = vpa(muw - muw_e);
    
        % Jacobain matrix
        Jw04_1= subs(Jw04,varsw, newVarsw);
   
        % Newton method
        qw = qw + pinv(Jw04_1)*(del_muw);
    
        % Termination condition
        if abs(del_muw)<1e-5
            break;
        end
    end
    
    q_w(:,j) = vpa(qw);
  
    xw2(j) = simplify(subs(Tw02(1,4),varsw, newVarsw));
    zw2(j) = simplify(subs(Tw02(3,4),varsw, newVarsw));
    
    prof_bpw(:,j) = [muw_e(1);muw_e(3)];
    
end
% hold off
for j = 1:length(t)
plotxw4(j) = prof_bpw(1,j)+xw0(j);
plotzw4(j) = prof_bpw(2,j)+zw0(j);
plotxw0(j) = xw0(j);
plotzw0(j) = zw0(j);
plotxw2(j) = xw2(j)+xw0(j);
plotzw2(j) = zw2(j)+zw0(j);

plot([plotxw0(j),plotxw2(j)],[plotzw0(j),plotzw2(j)],'g-')
hold on
plot([plotxw2(j),plotxw4(j)],[plotzw2(j),plotzw4(j)],'k-')
xlim([-30,30])

xlabel('x-axis in cm');
ylabel('z-axis in cm');
title('Inverse Kinematics of Swing leg');

end

v = 0:0.1:1.5;

for j = 1:length(v)

plotx5(j) = x4(j) + l3*sind(10*v(j));
plotz5(j) = z4(j) + l3*cosd(10*v(j));

end

v = 1.5:-0.1:0;

for j = length(v):(2*length(v)-1)

plotx5(j) = x4(j) + l3*sind(10*v(j+1-length(v)));
plotz5(j) = z4(j) + l3*cosd(10*v(j+1-length(v)));

end

hold off

v = VideoWriter('biped.avi');
open(v);

for j = 1:length(t)
    
plotx4(j) = prof_bp(1,j);
plotz4(j) = prof_bp(2,j);
plotx0(j) = 0;
plotz0(j) = 0;
plotx2(j) = x2(j);
plotz2(j) = z2(j);

plot([plotx0(j),plotx2(j)],[plotz0(j),plotz2(j)],'r-o','linewidth',3,'MarkerEdgeColor','k',...
                        'MarkerFaceColor','k',...
                        'MarkerSize',5)
hold on
plot([plotx0(j),plotx0(j)+5],[plotz0(j),plotz0(j)],'r-o','linewidth',3,'MarkerEdgeColor','k',...
                        'MarkerFaceColor','k',...
                        'MarkerSize',5)
plot([plotx2(j),plotx4(j)],[plotz2(j),plotz4(j)],'b-o','linewidth',3,'MarkerEdgeColor','k',...
                        'MarkerFaceColor','k',...
                        'MarkerSize',5)

plot([plotx4(j),plotx5(j)],[plotz4(j),plotz5(j)],'m-o','linewidth',3,'MarkerEdgeColor','k',...
                        'MarkerFaceColor','k',...
                        'MarkerSize',5)
                    
plotxw4(j) = prof_bpw(1,j)+xw0(j);
plotzw4(j) = prof_bpw(2,j)+zw0(j);
plotxw0(j) = xw0(j);
plotzw0(j) = zw0(j);
plotxw2(j) = xw2(j)+xw0(j);
plotzw2(j) = zw2(j)+zw0(j);

plot([plotxw0(j),plotxw2(j)],[plotzw0(j),plotzw2(j)],'g-o','linewidth',3,'MarkerEdgeColor','k',...
                        'MarkerFaceColor','k',...
                        'MarkerSize',5)
plot([plotxw2(j),plotxw4(j)],[plotzw2(j),plotzw4(j)],'k-o','linewidth',3,'MarkerEdgeColor','k',...
                        'MarkerFaceColor','k',...
                        'MarkerSize',5)
plot([plotxw4(j),plotxw4(j)+5],[plotzw4(j),plotzw4(j)],'k-o','linewidth',3,'MarkerEdgeColor','k',...
                        'MarkerFaceColor','k',...
                        'MarkerSize',5)

xlim([-50,50])

xlabel('x-axis in cm');
ylabel('z-axis in cm');
title('Inverse Kinematics of Biped');

grid on
pause(0.1)
hold off

frame = getframe(gcf);
writeVideo(v,frame);

end

close(v);


