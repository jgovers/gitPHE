function [cons] = consGen(uOpt1,uOpt2,x1,x2,cons,LTI,predMat1,predMat2,par,dim)
    
xN1 = predMat1.P*x1 + predMat1.S*uOpt1;
xN2 = predMat2.P*x2 + predMat2.S*uOpt2;

Cv = kron(eye(dim.N+1),[0 0 1 0;0 0 0 1]);

vSbar1 = Cv*predMat1.S;
vSbar2 = Cv*predMat2.S;
vPbar1 = Cv*predMat1.P*x1;
vPbar2 = Cv*predMat2.P*x2;

for i = 2:dim.N+1
    vS1 = vSbar1((i-1)*dim.y+1:i*dim.y,:);
    vP1 = vPbar1((i-1)*dim.y+1:i*dim.y);
    cons = [cons,uOpt1'*(vS1'*vS1)*uOpt1 + 2*(vP1'*vS1)*uOpt1<=par.vMax1^2-vP1'*vP1];

    vS2 = vSbar2((i-1)*dim.y+1:i*dim.y,:);
    vP2 = vPbar2((i-1)*dim.y+1:i*dim.y);
    cons = [cons,uOpt2'*(vS2'*vS2)*uOpt2 + 2*(vP2'*vS2)*uOpt2<=par.vMax2^2-vP2'*vP2];

end

Cy1 = kron(eye(dim.N+1),LTI.C1);
Cy2 = kron(eye(dim.N+1),LTI.C2);

% ySbar1 = Cy1*predMat1.S;
% ySbar2 = Cy2*predMat2.S;
% yPbar1 = Cy1*predMat1.P*x1;
% yPbar2 = Cy2*predMat2.P*x2;

yN1 = Cy1*xN1;
yN2 = Cy2*xN2;

for i = 3:dim.N+1
    y1 = yN1((i-1)*dim.y+1:i*dim.y);
    y2 = yN2((i-1)*dim.y+1:i*dim.y);
    d = y1-y2;

    cons = [cons,d'*d>=par.dSafe^2];
end    