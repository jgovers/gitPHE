clear all
close all
clc

%%
A = [1 0;-1 0;0 1;0 -1];
b = [2;-1;1.5;-0.5];

H = [6 2;2 4];
f = [2;4];

a = 2;
n = 101;
x = linspace(-a,a,n);
y = linspace(-a,a,n);
X = [x;y];

Z = zeros(n);
G = zeros(n);

for i = 1:n
    for j = 1:n
        Z(i,j) = .5*[x(i) y(j)]*H*[x(i);y(j)] + f'*[x(i);y(j)];
    end
end

%%
[sol,fval] = quadprog(H,f,A,b);

%%
m = 50;
mu = zeros(4,m+1);
Dg = zeros(4,m);
xs = zeros(2,m);
eta = 1;

for i = 1:m
    Dg(:,i) = -A/H*(A'*mu(:,i)+f)-b;
    mu(:,i+1) = max(zeros(4,1),mu(:,i)+eta*Dg(:,i));
    xs(:,i) = -H\(A'*mu(:,i+1)+f);
    Zs(i) = .5*xs(:,i)'*H*xs(:,i) + f'*xs(:,i);
end

%%
figure(1); clf
mesh(x,y,Z'); hold
plot3(sol(1),sol(2),fval,'*'); plot3(xs(1,:),xs(2,:),Zs)
xlabel('x'); ylabel('y')
