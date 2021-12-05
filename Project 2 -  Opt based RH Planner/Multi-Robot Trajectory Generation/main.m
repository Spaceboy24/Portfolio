clear all
close all

CONSTANTS.N = 5;

load_parameters;

x0 = init_guess(CONSTANTS);

A = []; b = []; Aeq = []; beq = []; lb = []; ub = [];

options = optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',300000);
[x,f] = fmincon(@(x)costfun(x, CONSTANTS),x0,A,b,Aeq,beq,lb,ub,@(x)nonlcon(x,CONSTANTS),options);

N = CONSTANTS.N;

x1 = x(1:N+1);
y1 = x(N+2:2*N+2);
x2 = x(2*N+3:3*N+3);
y2 = x(3*N+4:4*N+4);
x3 = x(4*N+5:5*N+5);
y3 = x(5*N+6:6*N+6); 
T = x(end);

t = 0:0.001:T;

c1 = [BernsteinPoly(x1', t); BernsteinPoly(y1', t)];
c2 = [BernsteinPoly(x2', t); BernsteinPoly(y2', t)];
c3 = [BernsteinPoly(x3', t); BernsteinPoly(y3', t)];

plot(c1(1, :), c1(2, :)), hold on
plot(c2(1, :), c2(2, :)), hold on
plot(c3(1, :), c3(2, :)), hold on
plot(CONSTANTS.pobs(1),CONSTANTS.pobs(2),'o','MarkerSize', 10)

% The following code animates the vehicles following the trajectories. This
% can be useful since it is difficult to tell whether the trajectories
% indeed guarantee that the vehicles do not collide at any point in time.
m1 = plot(c1(1, 1), c1(2, 1), 'o', 'MarkerSize', 10);
m2 = plot(c2(1, 1), c2(2, 1), 'o', 'MarkerSize', 10);
m3 = plot(c3(1,1), c3(2, 1), 'o', 'MarkerSize', 10);
for i = 1:length(t)
    m1.XData = c1(1, i);
    m1.YData = c1(2, i);
    m2.XData = c2(1, i);
    m2.YData = c2(2, i);
    m3.XData = c3(1, i);
    m3.YData = c3(2, i);
    pause(0.01)
end

