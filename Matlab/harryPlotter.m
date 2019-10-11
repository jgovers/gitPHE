%% Figures
% Layout of the waypoints
figure(1); clf
plot(par.x01(1),par.x01(2),'o','LineWidth',2); hold on; grid on
plot(par.x02(1),par.x02(2),'o','LineWidth',2); plot(par.CW1(1),par.CW1(2),'o','LineWidth',2); 
plot(par.CW2(1),par.CW2(2),'o','LineWidth',2); plot(par.CW3(1),par.CW3(2),'o','LineWidth',2);
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
legend('O_1','O_2','CW_1','CW_2','CW_3','D1','D_2')

%% Position and velocity trace of vessel 1
figure(2); clf
yyaxis left
plot(t,y1(1,:)); hold on; grid on
plot(t,y1(2,:)); ylabel('Position')
yyaxis right
stairs(t,v1); ylabel('Velocity')
legend('x_1','y_1','v_1')
xlabel('t'); 

%% Position and velocity trace of vessel 2
figure(3); clf
yyaxis left
plot(t,y2(1,:)); hold on; grid on
plot(t,y2(2,:)); ylabel('Position')
yyaxis right
stairs(t,v2); ylabel('Velocity')
legend('x_2','y_2','v_2')
xlabel('t'); 

%% Relative distance between the vessels
figure(4); clf
stairs(t,d); hold on; grid on;
plot([0,Tf],[par.dSafe,par.dSafe],'-.');
plot([0,Tf],[par.rangeD,par.rangeD],'-.');
stairs(t,flag1(1:end-1)+12.5); stairs(t,flag2(1:end-1)+15); stairs(t,flagd+10);
ylim([0 35])
xlabel('t')
legend('d','d_{max}','d_{range}','flag_1','flag_2','flag_d')

%% Value of the optimization function
figure(5); clf
stairs(t,V0N); hold on; grid on
stairs(t,flag1(2:end)*10^4)
stairs(t,flag2(2:end)*10^4)
stairs(t,flagd*10^4)
legend('V^0_N','flag_1','flag_2','flag_d')
xlabel('k')

%% Paths of the vessels and waypoints
figure(6); clf
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
legend('path_1','path_2','O_1','O_2','CW_1','CW_2','CW_3','D1','D_2')