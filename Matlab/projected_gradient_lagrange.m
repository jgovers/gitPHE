clear all 
clc

%%
H = 2;
c = -4;

A = 1;
b = 5;

a = 15;
n = 101;
N = 20;
eta = .5;
x = linspace(-a,a,n);

f0 = zeros(1,n);
g = zeros(1,n);
Dg = zeros(1,n);

mu = zeros(1,N);
mu_bar = zeros(1,N-1);
Dg_opt = zeros(1,N-1);
x_star = zeros(1,N-1);
f_star = zeros(1,N-1);
g_star = zeros(1,N-1);

mu0 = 0;
mu(1) = mu0;

for i = 1:n
    f0(i) = .5*x(i)*H*x(i) + c*x(i);
    g(i) = -.5*(A*x(i)+c)*(1/H)*(A*x(i)+c)-b*x(i);
    Dg(i) = -A/H*(A*x(i)+c)-b;
end

for i = 1:N-1
    g_star(i) = -.5*(A*mu(i)+c)*(1/H)*(A*mu(i)+c)-b*mu(i);
    x_star(i) = -1/H*(A*mu(i)+c);
    f_star(i) = .5*x_star(i)*H*x_star(i) + c*x_star(i);

    Dg_opt(i) = -A/H*(A*mu(i)+c)-b;
    mu_bar(i) = mu(i)+eta*Dg_opt(i);
    mu(i+1) = max(0,mu_bar(i));
end

%%
figure(1); clf
plot(x,f0); hold on
plot([b/A b/A],[-150 300]);
plot(x,g); plot(x,Dg)
ylim([-100 100])
xlabel('x'); ylabel('f');
legend('f0','constraint','g','\Delta g')
grid on
title('Dual optimization')
for i = 1:1
    plot(mu(i),g_star(i),'r*')
    plot(x_star(i),f_star(i),'k*')
%     pause
end
