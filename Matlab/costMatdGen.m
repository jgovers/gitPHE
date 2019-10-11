function [costMat] = costMatdGen(predMat1,predMat2,weight,dim)

Dbar = 2*weight.Qd.*kron([1,-1;-1,1],eye(dim.y*(dim.N+1)));
Sbar = blkdiag(predMat1.S,predMat2.S);
Pbar = blkdiag(predMat1.P,predMat2.P);
Cbar = blkdiag(predMat1.C,predMat2.C);

costMat.H = Sbar'*Cbar'*Dbar*Cbar*Sbar;
costMat.h = Sbar'*Cbar'*Dbar*Cbar*Pbar;
costMat.Hx = Pbar'*Cbar'*Dbar*Cbar*Pbar;