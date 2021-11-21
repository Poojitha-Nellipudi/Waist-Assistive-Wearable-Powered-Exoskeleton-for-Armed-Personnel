v = VideoWriter('exo2.avi');
open(v);
for i = 1:10:length(t)
   

q = [q1 q2 q3 q4 q5];
q_new = [qf(1,i) qf(2,i) qf(3,i) qf(4,i) qf(5,i)];
    
biped_c = animation_plot(q,q_new,T01,T02,T03,T04,T05,T06);


plot(biped_c(1,1:7),biped_c(2,1:7),'r-o','linewidth',3,'MarkerEdgeColor','k',...
                        'MarkerFaceColor','k',...
                        'MarkerSize',5)
hold on
plot(prof_bp(1,:),prof_bp(2,:),'k-')
set(gca,'fontsize',16)
xlabel('x,[m]');
ylabel('z,[m]');
axis([-0.5 0.5 -0.1 0.9])
axis square
grid on
pause(0.1)
hold off
    
frame = getframe(gcf);
writeVideo(v,frame);
end

close(v);