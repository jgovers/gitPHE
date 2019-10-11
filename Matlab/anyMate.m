%% Animation of the vessels
figure(7); clf
plot(x1(1,:),x1(2,:),'r'); hold on; grid on
plot(x2(1,:),x2(2,:),'b'); 
plot(par.x01(1),par.x01(2),'o','LineWidth',2); plot(par.x02(1),par.x02(2),'o','LineWidth',2); 
plot(par.CW1(1),par.CW1(2),'o','LineWidth',2); plot(par.CW2(1),par.CW2(2),'o','LineWidth',2);
plot(par.CW3(1),par.CW3(2),'o','LineWidth',2);
plot(par.D1(1),par.D1(2),'o','LineWidth',2); plot(par.D2(1),par.D2(2),'o','LineWidth',2); 
plot([par.x01(1),par.CW1(1)],[par.x01(2),par.CW1(2)],'k-.'); plot([par.x02(1),par.CW1(1)],[par.x02(2),par.CW1(2)],'k-.');
plot([par.CW1(1),par.CW3(1)],[par.CW1(2),par.CW3(2)],'k-.'); plot([par.CW3(1),par.D1(1)],[par.CW3(2),par.D1(2)],'k-.');
plot([par.CW3(1),par.D2(1)],[par.CW3(2),par.D2(2)],'k-.');
rectangle('Position',[par.CW1(1)-par.rangeG,par.CW1(2)-par.rangeG,2*par.rangeG,2*par.rangeG],'Curvature',[1 1],'LineStyle',':');
rectangle('Position',[par.CW2(1)-par.rangeG,par.CW2(2)-par.rangeG,2*par.rangeG,2*par.rangeG],'Curvature',[1 1],'LineStyle',':');
rectangle('Position',[par.CW3(1)-par.rangeG,par.CW3(2)-par.rangeG,2*par.rangeG,2*par.rangeG],'Curvature',[1 1],'LineStyle',':');


xlabel('x'); ylabel('y')
xlim([0 80]); ylim([-20 20]);
pbaspect([80 40 1])

for i = 1:Tf/T+1
    h1 = plot(x1(1,i),x1(2,i),'or');
    h2 = plot(x2(1,i),x2(2,i),'ob');
    h3 = rectangle('Position',[x1(1,i)-par.dSafe/2,x1(2,i)-par.dSafe/2,par.dSafe,par.dSafe],'Curvature',[1 1],'LineStyle',':');
    h4 = rectangle('Position',[x2(1,i)-par.dSafe/2,x2(2,i)-par.dSafe/2,par.dSafe,par.dSafe],'Curvature',[1 1],'LineStyle',':');
    h5 = rectangle('Position',[x1(1,i)-par.rangeD/2,x1(2,i)-par.rangeD/2,par.rangeD,par.rangeD],'Curvature',[1 1],'LineStyle',':');
    h6 = rectangle('Position',[x2(1,i)-par.rangeD/2,x2(2,i)-par.rangeD/2,par.rangeD,par.rangeD],'Curvature',[1 1],'LineStyle',':');
    
    pause(T)
    delete(h1); delete(h2); delete(h3); delete(h4); delete(h5); delete(h6);
end
