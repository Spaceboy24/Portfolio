function [c,ceq] = nonlcon(x,ITER,CONSTANTS)
%NONLCON Summary of this function goes here
%   Detailed explanation goes here
N = CONSTANTS.N; 
n = CONSTANTS.n;
h_size = CONSTANTS.h_size;
T = x(end);

speed = [];
angrate = [];
distobs = [];
for j = 0:ITER.h_size-1

    x_h(j+1,:) = x(n*j+1:n*(j+1)+1)';
    y_h(j+1,:) = x(ITER.h_size*n+n*j+2:ITER.h_size*n+n*(j+1)+2)';    
    [~,~,Diff] = BeBOT(n,T/n);
    xdot_h(j+1,:) = x_h(j+1,:)*Diff;
    ydot_h(j+1,:) = y_h(j+1,:)*Diff;
    xddot_h(j+1,:) = xdot_h(j+1,:)*Diff;
    yddot_h(j+1,:) = ydot_h(j+1,:)*Diff;
    
    speed1 = BernsteinProduct(xdot_h(j+1,:), xdot_h(j+1,:))...
        + BernsteinProduct(ydot_h(j+1,:), ydot_h(j+1,:));
    speed = [speed speed1];
   
    angrate1 = (BernsteinProduct(xddot_h(j+1,:),ydot_h(j+1,:))...
        - BernsteinProduct(xdot_h(j+1,:),yddot_h(j+1,:)))./(speed1);
    angrate = [angrate angrate1];
    
%     distobs1 = BernsteinProduct(x_h(j+1,:)-CONSTANTS.pobs(1),x_h(j+1,:)...
%         - CONSTANTS.pobs(1)) + BernsteinProduct(y_h(j+1,:)...
%         - CONSTANTS.pobs(2),y_h(j+1,:)-CONSTANTS.pobs(2))
%     distobs = [distobs distobs1];
    
end

% c = [speed' - CONSTANTS.vmax^2; -speed' + CONSTANTS.vmin^2;
%     angrate' - CONSTANTS.omegamax^2; -angrate' - CONSTANTS.omegamax^2;
%     -distobs' + CONSTANTS.mindist^2];
c = [speed' - CONSTANTS.vmax^2; -speed' + CONSTANTS.vmin^2;
    angrate' - CONSTANTS.omegamax^2; -angrate' - CONSTANTS.omegamax^2];
ceq = [x_h(1,1)-ITER.pinit(1); y_h(1,1)-ITER.pinit(2); x_h(end, end)...
    - ITER.pfin(1); y_h(end, end) - ITER.pfin(2)];
% ceq = [x_h(1,1)-ITER.pinit(1); y_h(1,1)-ITER.pinit(2); x_h(end, end)...
%     - ITER.pfin(1); y_h(end, end) - ITER.pfin(2); xdot_h(1,1) - 0.0785;
%     ydot_h(1,1) - 0.0785; xdot_h(end,end) - 0.0785; ydot_h(end, end) + 0.0219];

for j = 1:ITER.h_size-1
    
    ceq_cont = [x_h(j,end) - x_h(j+1,1); y_h(j,end) - y_h(j+1,1); 
        xdot_h(j,end) - xdot_h(j+1,1); ydot_h(j,end) - ydot_h(j+1,1);
        xddot_h(j,end) - xddot_h(j+1,1); yddot_h(j,end) - yddot_h(j+1,1)];
    ceq = [ceq; ceq_cont];
    
end
end

