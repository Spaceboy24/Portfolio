function [c,ceq] = nonlcon(x,P)
%NONLCON Summary of this function goes here
%   Detailed explanation goes here
N = P.N; 

x1 = x(1:N+1);
y1 = x(N+2:2*N+2);
T = x(end);

[~,~,Diff] = BeBOT(N,T);
 
x1dot = x1'*Diff;
y1dot = y1'*Diff;


x1ddot = x1dot*Diff;
y1ddot = y1dot*Diff;

speed = BernsteinProduct(x1dot, x1dot) + BernsteinProduct(y1dot, y1dot);

angrate = (BernsteinProduct(x1ddot,y1dot)-BernsteinProduct(x1dot,y1ddot))./(speed);

%Obstacle Avoidance
distobs1 = BernsteinProduct(x1-P.pobs(1,1),x1-P.pobs(1,1)) + BernsteinProduct(y1-P.pobs(1,2),y1-P.pobs(1,2));
%distobs2 = BernsteinProduct(x1-P.pobs(1,1),x1-P.pobs(2,1)) + BernsteinProduct(y1-P.pobs(1,2),y1-P.pobs(2,2));

c = [
    speed' - P.vmax^2;
    -speed' + P.vmin^2;
    angrate' - P.omegamax^2;
    -angrate' - P.omegamax^2;-T;
    -distobs1' + P.mindist^2];% -distobs2' + P.mindist^2];
ceq = [x1(1) - P.pinit(1); y1(1) - P.pinit(2); x1(end) - P.pfin(1); y1(end) - P.pfin(2)];
end

