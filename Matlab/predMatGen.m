function [predMat] = predMatGen(G,dim)

P = zeros(dim.x*(dim.N+1),dim.x);
S = zeros(dim.x*(dim.N+1),dim.u*(dim.N));

for k = 0:dim.N
    P(k*dim.x+1:(k+1)*dim.x,:) = G.A^k;
end

for k = 1:dim.N
    for i = 0:k-1
        S(k*dim.x+1:(k+1)*dim.x,i*dim.u+1:(i+1)*dim.u) = G.A^(k-1-i)*G.B;
    end
end

predMat.P = P;
predMat.S = S;
predMat.C = kron(eye(dim.N+1),G.C);