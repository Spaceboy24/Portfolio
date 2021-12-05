clear all;
close all;
clc;

P.N = 10;
N = P.N; 

load_parameters;

[x_nom,y_nom] = get_nom();  %Get Nominal Trajectory

t_f = 10;                   %Final time for nominal trajectory
delta_t = 0.05;              %Trigger time

%Plotting nominal trajectory
t = 0:delta_t:t_f;          
c2 = [BernsteinPoly(x_nom, t, 0, 10); BernsteinPoly(y_nom, t, 0, 10)];
plot(c2(1, :), c2(2, :),'LineWidth',3); hold on

i = 1;                      %Iteration variable

t_k = t(i);                 %Iteration time variable
t_h = 5;                    %Horizon end time
t_hp = 0;                   %Horizon loop control variable
t_total = 0;                %Total time for full trajectory

points = zeros(2,((t_f-t_h)/delta_t)+2)

points(:, 1) = P.pinit';
points(:, end) = P.pfin';

P.pinit = [BernsteinPoly(x_nom,t_k,0,10), BernsteinPoly(y_nom,t_k,0,10)];
P.pfin = [BernsteinPoly(x_nom, t_h,0,10), BernsteinPoly(y_nom, t_h,0,10)];

tic
while t_h ~= t_hp
    
    %Initial guess for control points and time
    x1 = linspace(P.pinit(1), P.pfin(1), N+1);  
    y1 = linspace(P.pinit(2), P.pfin(2), N+1);
    T = 3;
    x0 = [x1';y1';T];
    
    %Fmincon Optimization
    A = []; b = []; Aeq = []; beq = []; lb = []; ub = [];
    options = optimoptions(@fmincon,'Algorithm','sqp'...
        ,'MaxFunctionEvaluations',300000);
    [x,f] = fmincon(@(x)costfun(x, P),x0,A,b,Aeq,beq,lb,ub...
        ,@(x)nonlcon(x,P),options);
    
    x1 = x(1:N+1)';
    y1 = x(N+2:2*N+2)';
    T = x(end);
    
    t = t_k:delta_t:t_h;
    c3 = [BernsteinPoly(x1, t); BernsteinPoly(y1, t)];
    plot(c3(1, :), c3(2, :)); hold on
    
    %Reinitialize timing variables
    [t_k,t_h,t_hp,t_total] = get_init(t_k,t_h,t_hp,t_total, delta_t);
    
    %Reinitialize initial and final position for next horizon
    P.pinit = [BernsteinPoly(x1,delta_t,0,t_h-t_k),...
        BernsteinPoly(y1,delta_t,0,t_h-t_k)];
    P.pfin = [BernsteinPoly(x_nom, t_h,0,t_f),...
        BernsteinPoly(y_nom, t_h,0,t_f)];
    
    i = i+1
    %Store data points
    points(:,i) = [BernsteinPoly(x1,delta_t,0,t_h-t_k),...
        BernsteinPoly(y1,delta_t,0,t_h-t_k)]';
end


t_total = t_total + T

%Plot optimized trajectory
plot(points(1,:),points(2,:),'-r','LineWidth',3); hold on
title('Horizon = 5s, GR Trajectory')
xlabel('x');
ylabel('y');
legend('nominal', 'optimal');
plot(P.pobs(1,1),P.pobs(1,2),'o','LineWidth',2);
toc
