%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A model predictive control approach for vessel train formations of      %
% cooperative multi-vessel systems.                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Matlab code for the final MPC design project of the course:
% SC42125 Model Predictive Control (2018/2019)
% 
% Made by:
% Jurriaan Govers - 4163753
%

yalmip('clear')
clear all
clc

doPlots = 1;        % Set this value to 1 if plots are wanted, set to 0 otherwise
doAnimation = 1;    % Set this value to 1 if an animation is wanted, set to 0 otherwise

tic

%% Horizons
T = .1;     % Sample time
Tf = 30;    % Simulation horizon
dim.N = 3;  % Control horizon

%% State space representations 
LTI.A1 = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1];  % State space formulation of vessel 1
LTI.B1 = [0 0;0 0;1 0;0 1];
LTI.C1 = [1 0 0 0;0 1 0 0];
LTI.D1 = [0 0;0 0];
LTI.G1 = ss(LTI.A1,LTI.B1,LTI.C1,LTI.D1,1);

LTI.A2 = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1];  % State space formulation of vessle 2
LTI.B2 = [0 0; 0 0; 1 0; 0 1];
LTI.C2 = LTI.C1;
LTI.D2 = LTI.D1;
LTI.G2 = ss(LTI.A2,LTI.B2,LTI.C2,LTI.D2,1);

dim.x = 4;  % Number of states
dim.u = 2;  % Number of inputs
dim.y = 2;  % Number of outputs

%% Parameters
par.x01 = [ 0;20;0;0];  % Starting point of vessel 1
par.x02 = [10;-10;0;0]; % Starting point of vessel 2

par.CW1 = [20;0;0;0];   % First common waypoint
par.CW2 = [40;0;0;0];   % Second common waypoint
par.CW3 = [60;0;0;0];   % Third common waypoint

par.D1 = [80;-20;0;0];  % Destination of vessel 1
par.D2 = [80;15;0;0];   % Destination of vessel 2

par.rangeG = 5;     % The range wherein a vessel changes to the next waypoint
par.rangeD = 10;    % The range wherein the vessels try to aggregate

par.vMax1 = 5;      % Maximum velocity of vessel 1
par.vDes1 = 4.5;    % Desired velocity of vessel 1
par.vMax2 = 3;      % Maximum velocity of vessel 2
par.vDes2 = 2.5;    % Desired velocity of vessel 2
par.dSafe = 2;      % Minimum safety distance

%% Trajectory generation


%% Weights and cost matrices 
weight.Q = [5 0 0 0;0 5 0 0;0 0 1 0;0 0 0 1];            % Weight on the state
weight.R = eye(dim.u);                                   % Weight on the input
[~,weight.P1,~] = dlqr(LTI.A1,LTI.B1,weight.Q,weight.R); % Weight on the terminal cost of vessel 1
[~,weight.P2,~] = dlqr(LTI.A2,LTI.B2,weight.Q,weight.R); % Weight on the terminal cost of vessel 2
weight.Qd = 10;                                          % Weight on the aggregation

[predMat1] = predMatGen(LTI.G1,dim);            % Prediction matrices for vessel 1
[costMat1] = costMatGen(predMat1,weight,dim);   % Cost matrices for vessel 1

[predMat2] = predMatGen(LTI.G2,dim);            % Prediction matrices for vessel 2
[costMat2] = costMatGen(predMat2,weight,dim);   % Cost matrices for vessel 1

[costMatd] = costMatdGen(predMat1,predMat2,weight,dim); % Cost matrices for the aggregation

%% Optimization initialization
uOpt1 = sdpvar(dim.u*dim.N,1);  % Optimization variables, control inputs of vessel 1
uOpt2 = sdpvar(dim.u*dim.N,1);  % Optimization variables, control inputs of vessel 2

opts = sdpsettings('verbose',0,'solver','FMINCON'); % Optimization settings, the problem is QCQP so FMINCON is used as a solver

cons = [];  % The constraints are defined in the loop

%% Optimization and simulation
% Set all initial variables
t = 0:T:Tf;

x1 = zeros(dim.x,Tf/T+2);
x1(:,1) = par.x01;
x2 = zeros(dim.x,Tf/T+2);
x2(:,1) = par.x02;

y1 = zeros(dim.y,Tf/T+1);
y2 = zeros(dim.y,Tf/T+1);

v1 = zeros(1,Tf/T+1);
v2 = zeros(1,Tf/T+1);

u1 = zeros(dim.u,Tf/T+1);
u2 = zeros(dim.u,Tf/T+1);
uN1 = zeros(dim.u*dim.N,Tf/T+1);
uN2 = zeros(dim.u*dim.N,Tf/T+1);

d = zeros(1,Tf/T+1);

V0N = zeros(1,Tf/T+1);

flag1 = zeros(1,Tf/T+2);
flag2 = zeros(1,Tf/T+2);
flagd = zeros(1,Tf/T+1);

CG1 = par.CW1;
CG2 = par.CW1;

% The actual simulation loop
for k = 1:Tf/T+1
    clc
    disp([num2str(k),'/',num2str(Tf/T+1)])
    y1(:,k) = LTI.C1*x1(:,k); 
    y2(:,k) = LTI.C2*x2(:,k);    
    v1(k) = sqrt(x1(3,k)^2 + x1(4,k)^2);    % Velocity of vessel 1
    v2(k) = sqrt(x2(3,k)^2 + x2(4,k)^2);    % Velocity of vessel 2
    
    d(k) = norm(y1(:,k)-y2(:,k));   % Relative distance between the vessels
    
    % Waypoint selection of vessel 1
    if norm(y1(:,k)-LTI.C1*par.CW1) >= par.rangeG && flag1(k) == 0
        CG1 = par.CW1;
    elseif norm(y1(:,k)-LTI.C1*par.CW2) >= par.rangeG && flag1(k) <= 1
        CG1 = par.CW2;
        flag1(k+1) = 1;
    elseif norm(y1(:,k)-LTI.C1*par.CW3) >= par.rangeG && flag1(k) <= 2
        CG1 = par.CW3;
        flag1(k+1) = 2;
    else
        CG1 = par.D1;
        flag1(k+1) = 3;
    end
    
    % Waypoint selection of vessel 2
    if norm(y2(:,k)-LTI.C2*par.CW1) >= par.rangeG && flag2(k) == 0
        CG2 = par.CW1;
    elseif norm(y2(:,k)-LTI.C2*par.CW2) >= par.rangeG && flag2(k) <= 1
        CG2 = par.CW2;
        flag2(k+1) = 1;
    elseif norm(y2(:,k)-LTI.C2*par.CW3) >= par.rangeG && flag2(k) <= 2
        CG2 = par.CW3;
        flag2(k+1) = 2;
    else
        CG2 = par.D2;
        flag2(k+1) = 3;
    end
    
    % Range check for aggregation 
    if d(k)<=par.rangeD && norm(y1(:,k)-LTI.C1*CG1)>=par.rangeG && norm(y2(:,k)-LTI.C2*CG2)>=par.rangeG
        flagd(k) = 1;
    else
        flagd(k) = 0;
    end
    
    % The cost functions
    V1 = 0.5*uOpt1'*costMat1.H*uOpt1 + (costMat1.h*(x1(:,k)-CG1))'*uOpt1;
    V2 = 0.5*uOpt2'*costMat2.H*uOpt2 + (costMat2.h*(x2(:,k)-CG2))'*uOpt2;
    Vd = 0.5*[uOpt1;uOpt2]'*costMatd.H*[uOpt1;uOpt2] + (costMatd.h*[(x1(:,k));(x2(:,k))])'*[uOpt1;uOpt2];
    
    V = V1+V2+flagd(k)*Vd;
    
    % The constraints
    cons = [];
    cons = consGen(uOpt1,uOpt2,x1(:,k),x2(:,k),cons,LTI,predMat1,predMat2,par,dim);
    
    % The optimization 
    info = optimize(cons,V,opts);
    
    V0N(k) = value(V) + 0.5*(x1(:,k)-CG1)'*costMat1.Hx*(x1(:,k)-CG1) ...
        + 0.5*(x2(:,k)-CG2)'*costMat2.Hx*(x2(:,k)-CG2) + flagd(k)*0.5*[(x1(:,k));(x2(:,k))]'*costMatd.Hx*[(x1(:,k));(x2(:,k))];
    
    if info.problem >=1
        disp(info);
        break
    end
    
    % Updating the state space evolution
    uN1(:,k) = value(uOpt1);
    uN2(:,k) = value(uOpt2);
    
    u1(:,k) = uN1(1:dim.u,k);
    x1(:,k+1) = LTI.A1*x1(:,k) + LTI.B1*u1(:,k);   
    
    u2(:,k) = uN2(1:dim.u,k);
    x2(:,k+1) = LTI.A2*x2(:,k) + LTI.B2*u2(:,k);
end

toc

%% Figures
if doPlots
    run harryPlotter
    drawnow
end

if doAnimation
    run anyMate
end