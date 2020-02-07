clear all 
close all
clc

%%
H = 2;
c = 4;

A = -1;
b = -3;

a = 15;
n = 101;
x = linspace(-a,a,n);

f0 = zeros(n,1);
g = zeros(n,1);

for i = 1:n
    f0(i) = .5*x(i)*H*x(i) + c'*x(i);
    g(i) = -.5*(A*x(i)+c)*(1/H)*(A*x(i)+c)-b*x(i);
end



%%
figure(1); clf
plot(x,f0); hold on
plot([b/A b/A],[-150 300]);
plot(x,g)
xlabel('x'); ylabel('f')
grid on
title('Dual optimization')
