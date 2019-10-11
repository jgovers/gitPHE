function [costMat] = costMatGen(predMat,weight,dim)

costMat.H = predMat.S'*blkdiag(kron(eye(dim.N),weight.Q),weight.P1)*predMat.S + kron(eye(dim.N),weight.R);
costMat.h = predMat.S'*blkdiag(kron(eye(dim.N),weight.Q),weight.P1)*predMat.P;
costMat.Hx = predMat.P'*blkdiag(kron(eye(dim.N),weight.Q),weight.P1)*predMat.P;